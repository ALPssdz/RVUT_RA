# -*- coding: utf-8 -*-
"""
mock_transmitter/dataset_replayer.py — IQ 数据集重放引擎
=========================================================
负责从本地真实录制的无人机 IQ 数据集中读取片段，执行重采样并缓冲推送至
PlutoSDR 硬件接口，可选地在多个扇区频点间执行跳频驻留。

支持的无人机型号 / 数据集目录映射表在 DRONE_CATALOG 中集中定义，
新增机型只需扩展该字典，无需修改调用代码。

物理重采样公式
--------------
数据集录制采样率 fs_raw = 100 MSps（USRP X310）
PlutoSDR 发射采样率 fs_tx  = 40  MSps

重采样比：
    up / down = fs_tx / fs_raw = 40/100 = 2/5

调用：scipy.signal.resample_poly(x, up=2, down=5)
      使用内置多相滤波器组，保证通带平坦以及阻带抑制，避免时域混叠。

跳频驻留时序（Hopping mode）
-----------------------------
每个频点驻留时长 dwell_ms，在驻留期结束后切换 PlutoSDR TX LO 至下一扇区。
随机抖动量：dwell_ms × dither_ratio（均匀分布），使跳频时序具有随机性，
更真实地还原 OcuSync 自适应跳频行为。
    t_dwell = dwell_ms × (1 + Uniform[-dither, +dither])
"""

import os
import glob
import time
import threading
import numpy as np
from typing import List, Optional, Callable

from scipy.signal import resample_poly

# ──────────────────────────────────────────────────────────────────────────────
# 数据集目录映射表
# 结构：{ 显示名称: { "path": 目录路径, "pack": 文件前缀, "bw_mhz": 带宽(MHz) } }
# ──────────────────────────────────────────────────────────────────────────────
_DATASET_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "Drone RF Data"
)

DRONE_CATALOG = {
    # ── DJI Mini 4 Pro ──────────────────────────────────────────────────────
    "DJI Mini 4 Pro  [BW=20MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI MINI4 PRO", "VTSBW=20"),
        "pack":   "pack2", "bw_mhz": 20,
    },
    "DJI Mini 4 Pro  [BW=10MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI MINI4 PRO", "VTSBW=10"),
        "pack":   "pack1", "bw_mhz": 10,
    },
    # ── DJI Mini 3 ──────────────────────────────────────────────────────────
    "DJI Mini 3      [BW=20MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI MINI3", "VTSBW=20"),
        "pack":   "pack2", "bw_mhz": 20,
    },
    "DJI Mini 3      [BW=10MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI MINI3", "VTSBW=10"),
        "pack":   "pack2", "bw_mhz": 10,
    },
    # ── DJI Mavic 3 Pro ─────────────────────────────────────────────────────
    "DJI Mavic 3 Pro [BW=20MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI MAVIC3 PRO", "VTSBW=20"),
        "pack":   "pack2", "bw_mhz": 20,
    },
    "DJI Mavic 3 Pro [BW=10MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI MAVIC3 PRO", "VTSBW=10"),
        "pack":   "pack1", "bw_mhz": 10,
    },
    # ── DJI Avata 2 ─────────────────────────────────────────────────────────
    "DJI Avata 2     [BW=20MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI AVATA2", "VTSBW=20"),
        "pack":   "pack2", "bw_mhz": 20,
    },
    "DJI Avata 2     [BW=10MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI AVATA2", "VTSBW=10"),
        "pack":   "pack1", "bw_mhz": 10,
    },
    "DJI Avata 2     [BW=40MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI AVATA2", "VTSBW=40"),
        "pack":   "pack4", "bw_mhz": 40,
    },
    "DJI Avata 2     [BW=60MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI AVATA2", "VTSBW=60"),
        "pack":   "pack3", "bw_mhz": 60,
    },
    # ── DJI FPV Combo ───────────────────────────────────────────────────────
    "DJI FPV Combo   [BW=20MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI FPV COMBO", "VTSBW=20"),
        "pack":   "pack3", "bw_mhz": 20,
    },
    "DJI FPV Combo   [BW=10MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI FPV COMBO", "VTSBW=10"),
        "pack":   "pack1", "bw_mhz": 10,
    },
    "DJI FPV Combo   [BW=40MHz]": {
        "path":   os.path.join(_DATASET_ROOT, "DJI FPV COMBO", "VTSBW=40"),
        "pack":   "pack2", "bw_mhz": 40,
    },
}

# 数据集原始采样率（USRP X310 录制）
FS_RAW: float = 100e6
# PlutoSDR 发射目标采样率
FS_TX:  float = 40e6
# 单次推送至 FPGA Cyclic Buffer 的复数样本数（约 10 ms @40MSps）
CHUNK_SAMPLES: int = 400_000


def _find_iq_files(dataset_path: str, pack_prefix: str) -> List[str]:
    """
    在指定目录中按时序顺序枚举所有匹配 pack_prefix_*.iq 的文件。

    Parameters
    ----------
    dataset_path : str  数据集目录绝对路径
    pack_prefix  : str  文件名前缀（如 "pack2"）

    Returns
    -------
    List[str]  按文件名排序的 .iq 文件绝对路径列表
    """
    pattern = os.path.join(dataset_path, f"{pack_prefix}_*.iq")
    files = sorted(glob.glob(pattern))
    return files


def _load_and_resample_segment(
    iq_path: str,
    offset_samples: int = 0,
    n_samples: int = CHUNK_SAMPLES,
    fs_src: float = FS_RAW,
    fs_dst: float = FS_TX,
) -> np.ndarray:
    """
    从 .iq 文件读取指定偏移处的 IQ 片段并重采样至目标采样率。

    重采样比 = fs_dst / fs_src = 40/100 = 2/5，
    使用 scipy.signal.resample_poly(x, up=2, down=5)。

    Parameters
    ----------
    iq_path        : str   .iq 文件路径（complex64，每点 8 字节）
    offset_samples : int   文件内起始样本偏移
    n_samples      : int   读取原始样本数
    fs_src         : float 原始采样率（Hz）
    fs_dst         : float 目标采样率（Hz）

    Returns
    -------
    np.ndarray  重采样后的 complex64 IQ 序列
    """
    from fractions import Fraction
    ratio = Fraction(fs_dst / fs_src).limit_denominator(100)
    up, down = ratio.numerator, ratio.denominator

    raw = np.fromfile(
        iq_path,
        dtype=np.complex64,
        count=n_samples,
        offset=offset_samples * 8,   # complex64 = 8 bytes/sample
    )
    if len(raw) == 0:
        return np.zeros(int(n_samples * up / down), dtype=np.complex64)

    return resample_poly(raw, up, down).astype(np.complex64)


class DatasetReplayer:
    """
    IQ 数据集重放引擎。

    工作流程：
      1. 根据所选机型从 DRONE_CATALOG 定位数据集目录
      2. 枚举该目录下所有 .iq 文件，构建循环播放队列
      3. 逐段读取 IQ 数据，执行重采样（100→40 MSps）并归一化至 PlutoSDR 量化范围
      4. 调用 PlutoTxInterface.push_cyclic() 推送至 FPGA Cyclic Buffer
      5. 跳频模式：在驻留时间到期后调用 hw.set_lo() 切换 LO 频率

    Parameters
    ----------
    hw           : PlutoTxInterface  已初始化（但未必已连接）的硬件接口实例
    catalog_key  : str               DRONE_CATALOG 中的机型键名
    hop_sectors  : List[float]       跳频扇区 LO 频率列表（Hz），单扇区=固定频点
    dwell_ms     : float             每扇区驻留时长（ms）
    dither_ratio : float             跳频时序随机抖动比例（0=无抖动，0.1=±10%）
    log_cb       : Callable          日志回调 f(msg: str)
    """

    def __init__(
        self,
        hw,
        catalog_key: str,
        hop_sectors:   Optional[List[float]] = None,
        dwell_ms:      float = 50.0,
        dither_ratio:  float = 0.10,
        log_cb:        Optional[Callable[[str], None]] = None,
    ):
        self._hw           = hw
        self._catalog_key  = catalog_key
        self._hop_sectors  = hop_sectors or [hw.lo_hz]
        self._dwell_ms     = dwell_ms
        self._dither_ratio = dither_ratio
        self._log          = log_cb if log_cb else lambda m: print(m)

        self._stop_event   = threading.Event()
        self._thread:      Optional[threading.Thread] = None

        # 载入数据集文件列表
        info = DRONE_CATALOG.get(catalog_key)
        if info is None:
            raise ValueError(f"未知机型键名：{catalog_key}")
        self._iq_files = _find_iq_files(info["path"], info["pack"])
        if not self._iq_files:
            raise FileNotFoundError(
                f"在 {info['path']} 中未找到 {info['pack']}_*.iq 文件。"
            )
        self._log(
            f"[Replayer] 已加载 {catalog_key} 数据集，共 {len(self._iq_files)} 个 IQ 文件。"
        )

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------
    def start(self):
        """启动后台重放线程。"""
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()
        self._log("[Replayer] 重放线程已启动。")

    def stop(self):
        """停止重放线程（阻塞直至线程退出）。"""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=5.0)
        self._log("[Replayer] 重放线程已停止。")

    # ------------------------------------------------------------------
    # 内部重放逻辑
    # ------------------------------------------------------------------
    def _replay_loop(self):
        """持续循环播放 IQ 数据集，周期性切换跳频扇区。"""
        from .pluto_hw_interface import normalize_iq_for_pluto

        sector_idx = 0
        n_sectors  = len(self._hop_sectors)

        while not self._stop_event.is_set():
            # 计算本轮驻留时长（加入随机抖动）
            jitter = np.random.uniform(-self._dither_ratio, self._dither_ratio)
            dwell_s = self._dwell_ms / 1000.0 * (1.0 + jitter)

            # 切换 LO 至当前扇区
            current_lo = self._hop_sectors[sector_idx % n_sectors]
            self._hw.set_lo(current_lo)

            # 在当前扇区驻留，循环推送 IQ 数据直至驻留时间到期
            t_deadline = time.monotonic() + dwell_s
            file_idx   = 0
            file_offset = 0

            while time.monotonic() < t_deadline and not self._stop_event.is_set():
                iq_path = self._iq_files[file_idx % len(self._iq_files)]
                try:
                    chunk = _load_and_resample_segment(
                        iq_path,
                        offset_samples=file_offset,
                        n_samples=CHUNK_SAMPLES,
                    )
                    if len(chunk) == 0:
                        # 当前文件读尽，切换到下一个文件
                        file_idx    += 1
                        file_offset  = 0
                        continue

                    iq_tx = normalize_iq_for_pluto(chunk)
                    self._hw.push_cyclic(iq_tx)
                    file_offset += CHUNK_SAMPLES

                except Exception as e:
                    self._log(f"[Replayer] 读取/发送异常：{e}")
                    time.sleep(0.5)

            # 切换至下一扇区
            sector_idx += 1
            if n_sectors > 1:
                self._log(
                    f"[Replayer] 跳频 → 扇区 {sector_idx % n_sectors + 1}/"
                    f"{n_sectors}  ({self._hop_sectors[sector_idx % n_sectors]/1e6:.1f} MHz)"
                )
