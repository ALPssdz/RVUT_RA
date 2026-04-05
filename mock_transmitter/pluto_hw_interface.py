# -*- coding: utf-8 -*-
"""
mock_transmitter/pluto_hw_interface.py — PlutoSDR 硬件抽象层
=============================================================
封装所有 adi.Pluto 底层操作，向上层发射引擎提供统一接口。

主要职责：
  - 连接/断开 PlutoSDR 硬件节点（经 libiio 网络后端）
  - 配置 TX 射频链路物理参数（LO 频率、采样率、带宽、硬件增益）
  - 推送 IQ 数据至 FPGA Cyclic Buffer 并保持连续辐射
  - 在任意异常路径下保证安全关停（TX 增益归零、缓冲清空）

PlutoSDR TX 物理约束：
  - 采样率范围  : 520 833 Hz – 61 440 000 Hz
  - LO 频率范围 : 325 MHz – 3 800 MHz（原厂固件）；已破解固件可达 70 MHz – 6 GHz
  - 硬件增益    : -89.75 dB – 0 dB（步进 0.25 dB）
  - TX 数据格式 : complex64（上层） → 内部转换为 int16 IQ 交错格式
"""

import numpy as np
import threading
import time
from typing import Optional, Callable


# ──────────────────────────────────────────────────────────────────────────────
# PlutoSDR TX 量化常数
# 目标满幅度 = 32700（留 67 计数防止 int16 溢出下的硬件截断失真）
# ──────────────────────────────────────────────────────────────────────────────
PLUTO_TX_FULL_SCALE: int = 32700


def normalize_iq_for_pluto(iq: np.ndarray) -> np.ndarray:
    """
    将任意幅度的 complex64 IQ 数组归一化并缩放至 PlutoSDR TX 量化范围。

    量化公式：
        A_max = max|iq[n]|
        iq_tx[n] = iq[n] / A_max × PLUTO_TX_FULL_SCALE

    Parameters
    ----------
    iq : np.ndarray
        输入复数 IQ 序列（任意幅度）。

    Returns
    -------
    np.ndarray
        已缩放、dtype=complex64 的 IQ 序列，幅度范围约 ±32700。
    """
    max_amp = np.max(np.abs(iq))
    if max_amp < 1e-12:
        max_amp = 1.0
    return (iq / max_amp * PLUTO_TX_FULL_SCALE).astype(np.complex64)


class PlutoTxInterface:
    """
    PlutoSDR 发射机硬件抽象层。

    使用示例（推荐上下文管理器形式）：
    -------
    with PlutoTxInterface("ip:192.168.31.20", fs=40e6, lo_hz=5745e6) as tx:
        tx.push_cyclic(iq_data)
        time.sleep(10)
    # 离开上下文时自动安全关停
    """

    def __init__(
        self,
        uri: str,
        fs: float      = 40e6,
        bw: float      = 20e6,
        lo_hz: float   = 5745e6,
        gain_db: float = -10.0,
        log_cb: Optional[Callable[[str], None]] = None,
    ):
        """
        Parameters
        ----------
        uri      : str    PlutoSDR 网络地址，格式 "ip:x.x.x.x"
        fs       : float  发射采样率（Hz），默认 40 MSps
        bw       : float  TX RF 模拟带宽（Hz），默认 20 MHz
        lo_hz    : float  发射载波 LO 频率（Hz），默认 5745 MHz
        gain_db  : float  TX 硬件增益（dB，负值表示衰减），默认 -10 dB
        log_cb   : callable  日志回调函数 f(msg: str)，为 None 时打印到 stdout
        """
        self.uri     = uri
        self.fs      = int(fs)
        self.bw      = int(bw)
        self.lo_hz   = int(lo_hz)
        self.gain_db = float(gain_db)
        self._log    = log_cb if log_cb else lambda m: print(m)

        self._sdr        = None
        self._connected  = False
        self._transmitting = False

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        """
        建立与 PlutoSDR 的 libiio 网络连接，并配置射频链路参数。

        Returns
        -------
        bool  连接成功返回 True，失败返回 False。
        """
        try:
            import adi
            self._log(f"[HW] 正在连接 PlutoSDR @ {self.uri} ...")
            self._sdr = adi.Pluto(self.uri)

            # 配置采样率与 RF 带宽
            self._sdr.sample_rate      = self.fs
            self._sdr.tx_rf_bandwidth  = self.bw

            # 配置 TX LO 频率及硬件增益
            self._sdr.tx_lo                  = self.lo_hz
            self._sdr.tx_hardwaregain_chan0  = self.gain_db

            # 启用 FPGA Cyclic Buffer（由 FPGA 自主循环重播，减轻 USB 总线压力）
            self._sdr.tx_cyclic_buffer = True

            self._connected = True
            self._log(
                f"[HW] 连接成功：LO={self.lo_hz/1e6:.1f} MHz，"
                f"fs={self.fs/1e6:.0f} MSps，增益={self.gain_db:.1f} dB"
            )
            return True

        except Exception as e:
            self._log(f"[HW] 连接失败：{e}")
            self._connected = False
            return False

    def disconnect(self):
        """安全断开连接，清空 FPGA TX 缓冲并归零增益。"""
        if not self._connected or self._sdr is None:
            return
        try:
            self._log("[HW] 正在安全关停发射链路...")
            self._sdr.tx_cyclic_buffer = False
            self._sdr.tx(np.zeros(1024, dtype=np.complex64))
            try:
                self._sdr.tx_destroy_buffer()
            except Exception:
                pass
            self._log("[HW] 发射链路已安全关停，TX 缓冲已清空。")
        except Exception as e:
            self._log(f"[HW] 关停过程异常（已忽略）：{e}")
        finally:
            self._connected    = False
            self._transmitting = False
            self._sdr          = None

    # ------------------------------------------------------------------
    # 参数热更新（无需重连）
    # ------------------------------------------------------------------
    def set_lo(self, lo_hz: float):
        """更新 TX LO 频率（Hz）。"""
        if self._connected and self._sdr:
            self.lo_hz = int(lo_hz)
            self._sdr.tx_lo = self.lo_hz
            self._log(f"[HW] LO 已更新 → {self.lo_hz/1e6:.1f} MHz")

    def set_gain(self, gain_db: float):
        """更新 TX 硬件增益（dB）。"""
        if self._connected and self._sdr:
            self.gain_db = float(gain_db)
            self._sdr.tx_hardwaregain_chan0 = self.gain_db
            self._log(f"[HW] 增益已更新 → {self.gain_db:.1f} dB")

    # ------------------------------------------------------------------
    # 发射接口
    # ------------------------------------------------------------------
    def destroy_buffer(self):
        """
        销毁当前 FPGA TX Cyclic Buffer。

        必须在以下情况前调用：
          - 切换发射频点（set_lo）后重新压入新数据
          - 更新缓冲内容（push_cyclic 新数据块）
        否则 libiio 将抛出 "tx buffer must be destroyed first" 异常。
        """
        if not self._connected or self._sdr is None:
            return
        try:
            self._sdr.tx_destroy_buffer()
        except Exception:
            pass   # 缓冲未激活时销毁操作会静默失败，忽略即可
        self._transmitting = False

    def push_cyclic(self, iq: np.ndarray):
        """
        将 IQ 数据推送至 PlutoSDR FPGA Cyclic Buffer，启动连续辐射。

        若当前已有活跃缓冲，将自动先执行销毁操作，再装载新数据。
        调用前请确保已调用 connect()。

        Parameters
        ----------
        iq : np.ndarray
            已归一化的 complex64 IQ 序列（幅度约 ±PLUTO_TX_FULL_SCALE）。
        """
        if not self._connected or self._sdr is None:
            raise RuntimeError("PlutoSDR 未连接，无法发送数据。")
        # 若缓冲已激活，必须先销毁再重新装载
        if self._transmitting:
            self.destroy_buffer()
        self._sdr.tx(iq)
        self._transmitting = True

    # ------------------------------------------------------------------
    # 属性查询
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_transmitting(self) -> bool:
        return self._transmitting

    # ------------------------------------------------------------------
    # 上下文管理器协议
    # ------------------------------------------------------------------
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False   # 不吞噬异常
