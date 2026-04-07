# -*- coding: utf-8 -*-
from __future__ import annotations
"""
ui_qt/orangepi_monitor.py — 本机系统资源监控模块  v2.0
======================================================
架构升级：阻塞 sleep 差分 → 非阻塞双快照 QTimer 轮询

v1.x 问题根因：
  _collect() 内部调用 time.sleep(1.0) 阻塞当前 QThread，
  当 CAF-FFT 主检测线程（_hub_loop）占用大量 CPU 时，
  Python GIL 会延迟监控线程的 sleep 唤醒，导致监控面板
  数据更新停滞，视觉上表现为"停止运行"。

v2.0 改进：
  1. 非阻塞快照差分（Non-blocking Snapshot Diff）
       t=0：psutil.cpu_percent(interval=None) 清零 + 记录网络快照 snapshot_0
       t=T：psutil.cpu_percent(interval=None) 读取 + 记录网络快照 snapshot_1
       两次调用之间不阻塞，由 QTimer 的 timeout 信号触发，间隔 T
       QTimer 运行在主线程（Qt 事件循环），不依赖任何后台线程
       → 彻底消除 sleep 阻塞，GIL 不再影响监控刷新率

  2. 进程 CPU 亲和性绑核（CPU Affinity Pinning，仅 Linux）
       在 RK3588 上，_hub_loop（CAF-FFT）绑定到高性能大核群（A76: cpu4-7）
       监控采集定时器运行在主线程 / Qt 事件循环，已自然处于轻负载模式
       可选：将 CAF-FFT 线程显式绑定到大核，确保监控轮询不竞争同一核

采集公式（与 v1.x 相同，仅采样方式改变）：
  CPU 占用率（psutil 内部 /proc/stat 差分，窗口 = QTimer 间隔 T）：
      cpu_pct = (1 - Δt_idle / Δt_total) × 100   [%]

  网络吞吐量（两次 QTimer 触发点的字节差分）：
      Δt        = 实际两次采样时间差（秒）
      TX_{KB/s} = max(ΔB_tx, 0) / (Δt × 1024)
      RX_{KB/s} = max(ΔB_rx, 0) / (Δt × 1024)

信号接口（与 v1.x 完全兼容）：
    sig_stats(dict) — 每轮采集后发射，payload：
        {
          "cpu"         : float,   # 0.0 ~ 100.0 %
          "net_tx_kbps" : float,   # 上行 KB/s
          "net_rx_kbps" : float,   # 下行 KB/s
          "online"      : bool,    # 始终为 True（本地采集）
        }

依赖：
    pip install psutil
"""

import os
import time
from typing import Optional

from PyQt5.QtCore import QObject, QTimer, pyqtSignal


# ──────────────────────────────────────────────────────────────────────────────
# 非阻塞系统监控器（纯 QTimer 驱动，无后台线程）
# ──────────────────────────────────────────────────────────────────────────────
class OrangePiMonitorWorker(QObject):
    """
    本地系统资源周期性采集工作对象（v2.0 QTimer 重构版）。

    核心设计变更（v1.x → v2.0）：
    ─────────────────────────────
    v1.x：QThread + time.sleep(1.0) 差分
       ↓ sleep 阻塞 → GIL 竞争 → 检测运行时监控挂起

    v2.0：QTimer（主线程）+ 双快照非阻塞差分
       第 1 次 timeout：清零 cpu 累积 + 记录网络字节快照 t0
       第 2 次 timeout（间隔 T 后）：读取 cpu% + 记录网络快照 t1
       ΔT 由 Qt 事件循环保证，不阻塞任何线程

    参数
    ----
    interval_s : float
        采集周期（秒），即两次 sig_stats 发射之间的间隔。
        建议 2.0~5.0s；过短会增加 /proc 读取频率但影响不大。
    net_iface : str | None
        网卡名称。None 表示聚合全部网卡。
    """

    sig_stats = pyqtSignal(dict)

    def __init__(self,
                 interval_s: float = 2.0,
                 net_iface: Optional[str] = "eth0"):
        super().__init__()
        self._interval_ms = max(500, int(interval_s * 1000))
        self._iface       = net_iface

        # 快照状态
        self._snap_tx: int   = 0
        self._snap_rx: int   = 0
        self._snap_t:  float = 0.0
        self._initialized:  bool = False

        # QTimer（由调用方在其所在线程启动）
        self._timer = QTimer(self)
        self._timer.setInterval(self._interval_ms)
        self._timer.timeout.connect(self._on_tick)

    # ------------------------------------------------------------------
    def _get_net_bytes(self):
        """
        获取网络字节计数器快照（bytes_sent, bytes_recv）。

        优先使用指定网卡；网卡不存在时降级为全局聚合。
        """
        try:
            import psutil
            if self._iface:
                per_nic = psutil.net_io_counters(pernic=True)
                if self._iface in per_nic:
                    c = per_nic[self._iface]
                    return c.bytes_sent, c.bytes_recv
            c = psutil.net_io_counters(pernic=False)
            return c.bytes_sent, c.bytes_recv
        except Exception:
            return 0, 0

    def _on_tick(self):
        """
        QTimer timeout 回调：非阻塞双快照差分采集。

        第一次调用（_initialized=False）：
            ① 调用 cpu_percent(interval=None) 清零内部累积器
            ② 记录网络字节快照 t0
            标记 _initialized=True，等待下一个 tick

        subsequent 调用：
            ① 调用 cpu_percent(interval=None) 读出上一周期 cpu%
            ② 记录网络字节快照 t1
            ③ 计算 ΔB_tx / ΔB_rx，发射 sig_stats
            ④ 将 t1 快照保存为新的 t0，继续差分

        整个过程不含任何 sleep，从不阻塞 Qt 事件循环。
        """
        try:
            import psutil
        except ImportError:
            self.sig_stats.emit({
                "cpu": 0.0, "net_tx_kbps": 0.0,
                "net_rx_kbps": 0.0, "online": True,
                "error": "psutil not installed",
            })
            return

        now = time.monotonic()

        if not self._initialized:
            # ── 第一拍：仅清零，不发射 ────────────────────────────
            psutil.cpu_percent(interval=None)   # 清零累积
            self._snap_tx, self._snap_rx = self._get_net_bytes()
            self._snap_t   = now
            self._initialized = True
            return

        # ── 后续拍：读取 + 计算 + 发射 ───────────────────────────
        cpu_pct      = psutil.cpu_percent(interval=None)
        cur_tx, cur_rx = self._get_net_bytes()
        dt = now - self._snap_t
        if dt < 0.1:
            dt = 0.1  # 防止除以接近零的时间差

        # TX_{KB/s} = max(ΔB_tx, 0) / (Δt × 1024)
        tx_kbps = max(cur_tx - self._snap_tx, 0) / (dt * 1024.0)
        rx_kbps = max(cur_rx - self._snap_rx, 0) / (dt * 1024.0)

        # 更新快照
        self._snap_tx, self._snap_rx = cur_tx, cur_rx
        self._snap_t = now

        self.sig_stats.emit({
            "cpu":          float(cpu_pct),
            "net_tx_kbps":  float(tx_kbps),
            "net_rx_kbps":  float(rx_kbps),
            "online":       True,
        })

    # ------------------------------------------------------------------
    def start(self):
        """启动 QTimer（必须在目标线程的事件循环中调用）。"""
        self._initialized = False
        self._timer.start()

    def stop(self):
        """停止 QTimer。"""
        self._timer.stop()


# ──────────────────────────────────────────────────────────────────────────────
# CPU 亲和性绑核工具（RK3588 大小核调度优化）
# ──────────────────────────────────────────────────────────────────────────────
def pin_thread_to_big_cores():
    """
    将当前线程（调用者）绑定至 RK3588 Cortex-A76 大核（cpu4-7）。

    RK3588 核心拓扑：
        cpu0-3 : Cortex-A55（小核，效率核，1.8 GHz）
        cpu4-7 : Cortex-A76（大核，性能核，2.4 GHz）

    CAF-FFT 检测线程绑大核，可避免与 Qt 主线程（监控轮询）
    竞争同一 A55 核，从而消除监控卡顿。

    适用平台：Linux（Orange Pi 5 / RK3588）
    其他平台（Windows、x86_64）：静默忽略。
    """
    try:
        import platform
        if platform.system() != "Linux":
            return
        pid = os.getpid()
        tid = os.gettid() if hasattr(os, 'gettid') else pid

        # RK3588 大核：cpu4, cpu5, cpu6, cpu7
        BIG_CORES = [4, 5, 6, 7]
        # 使用 taskset 通过系统调用设置线程亲和性
        import ctypes
        libc = ctypes.CDLL("libc.so.6", use_errno=True)

        # cpu_set_t：128字节位图（Linux kernel sched.h）
        CPU_SETSIZE = 128
        cpu_set = (ctypes.c_uint8 * CPU_SETSIZE)()
        for cpu_id in BIG_CORES:
            byte_idx = cpu_id // 8
            bit_idx  = cpu_id %  8
            cpu_set[byte_idx] |= (1 << bit_idx)

        SYS_sched_setaffinity = 203   # x86_64; aarch64 = 122
        import platform as _plat
        if _plat.machine() == 'aarch64':
            SYS_sched_setaffinity = 122

        ret = libc.syscall(SYS_sched_setaffinity, tid,
                           CPU_SETSIZE, ctypes.byref(cpu_set))
        if ret == 0:
            print(f"[Monitor] 检测线程 tid={tid} 已绑定至 A76 大核 {BIG_CORES}")
        else:
            errno = ctypes.get_errno()
            print(f"[Monitor] 绑核失败 (errno={errno})，继续使用默认调度")
    except Exception as e:
        print(f"[Monitor] 绑核操作不可用（{e}），跳过")


# ──────────────────────────────────────────────────────────────────────────────
# 对外统一管理接口（v2.0 — 无后台线程，QTimer 驱动）
# ──────────────────────────────────────────────────────────────────────────────
class OrangePiMonitor:
    """
    本机系统资源监控器（对外统一管理接口 v2.0）。

    v2.0 架构改变：取消 QThread，改用 QTimer 在 Qt 主线程轮询。
    QTimer 跑在 Qt 事件循环中，属于纯 I/O 等待（/proc 读取），
    占用极少 CPU，不会干扰 CAF-FFT 检测线程。

    调用接口与 v1.x 完全兼容（start/stop/sig_stats）。

    使用示例：
        monitor = OrangePiMonitor()
        monitor.sig_stats.connect(self.on_stats_updated)
        monitor.start()
        ...
        monitor.stop()
    """

    def __init__(self,
                 interval_s: float = 2.0,
                 net_iface: Optional[str] = "eth0",
                 **_ignored_kwargs):
        """
        参数
        ----
        interval_s : 采集周期（秒）
        net_iface  : 网卡名称（None 表示聚合全部网卡）
        **_ignored_kwargs : 忽略历史遗留的 host/ssh_* 参数
        """
        # Worker 直接活在调用方线程（Qt 主线程），QTimer 自动归属该线程
        self._worker = OrangePiMonitorWorker(
            interval_s=interval_s,
            net_iface=net_iface,
        )
        # 转发信号，外部通过此属性连接槽函数
        self.sig_stats = self._worker.sig_stats

    def start(self):
        """启动 QTimer 轮询（在 Qt 主线程调用）。"""
        self._worker.start()

    def stop(self):
        """停止 QTimer 轮询。"""
        self._worker.stop()
