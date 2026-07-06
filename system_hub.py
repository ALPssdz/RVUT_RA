#!/usr/bin/env python3
import sys
import os
import time
import threading
import numpy as np
from typing import Optional

# Optional preload: on some Windows/PyQt5 environments importing torch before Qt
# avoids C++ runtime initialization failures. RKNN deployments may not ship torch.
try:
    import torch  # noqa: F401
except ImportError:
    torch = None

PROJ_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJ_ROOT not in sys.path:
    sys.path.insert(0, PROJ_ROOT)

from PyQt5.QtCore import Qt, QEvent
from PyQt5.QtGui import QImage, QPixmap, QFont, QKeySequence
from PyQt5.QtWidgets import QApplication, QShortcut
from PyQt5.QtCore import QObject, pyqtSignal

from backend_rk3588 import config as cfg
from backend_rk3588.main_rf_pipeline import RFToolchain
from ui_qt.gui_host import MainWindow
from database.db_manager import DBManager
from protocol.ra8p1_link import RA8P1Link
from diagnostics.recorder import DiagnosticsRecorder


# ==============================================================================
# stdout/stderr 重定向器
# 功能：将系统中所有 print() 输出路由到 GUI 日志框
# 分两阶段工作：
#   ① 预 GUI 阶段（hub 初始化期间）：存入内部缓冲区，并同步写入原 stdout
#   ② GUI 就绪后：flush 缓冲区 + 直接 emit 到 signal_log
# ==============================================================================
class _GuiLogRedirector:
    """
    sys.stdout 重定向器：将 print() 输出路由至 Qt signal_log 信号。

    使用方法：
        redirector = _GuiLogRedirector()
        sys.stdout = redirector
        # ... 创建 hub / GUI ...
        redirector.attach_signal(hub.signal_log.emit)
    """

    def __init__(self):
        self._lock      = threading.Lock()
        self._buf       = []           # 预 GUI 阶段缓冲区
        self._emit_fn   = None         # GUI 就绪后的 signal.emit 函数
        self._orig      = sys.__stdout__   # 保留原 stdout 用于 debug

    def attach_signal(self, emit_fn):
        """
        绑定 Qt 信号 emit 函数，并将缓冲区内容 flush 至 GUI 日志框。
        在 CentralHubEngine.__init__() 完成后（GUI 已构建）立即调用。
        """
        with self._lock:
            self._emit_fn = emit_fn
            for line in self._buf:
                try:
                    emit_fn(line)
                except Exception:
                    pass
            self._buf.clear()

    def write(self, text: str):
        # 始终写入原 stdout（保留终端可见性）
        if self._orig:
            try:
                self._orig.write(text)
            except Exception:
                pass

        with self._lock:
            # 按行切分，过滤纯空行避免日志框刷屏
            for line in text.split('\n'):
                stripped = line.rstrip()
                if not stripped:
                    continue
                if self._emit_fn:
                    try:
                        self._emit_fn(stripped)
                    except Exception:
                        self._buf.append(stripped)
                else:
                    self._buf.append(stripped)

    def flush(self):
        if self._orig:
            try:
                self._orig.flush()
            except Exception:
                pass

    def fileno(self):
        """兼容需要 fd 的库（如 tqdm）"""
        return self._orig.fileno() if self._orig else 1

class CentralHubEngine(QObject):
    """
    系统级中央调度引擎（Central Orchestration Engine）

    基于事件总线（Event Bus）架构，负责协调射频检测子系统、RA8P1 主控
    裁决链路与 HDMI 大屏显示终端，将告警事件持久化写入数据库，并通过
    Qt 信号机制向 GUI 表现层广播实时状态。

    子系统组成：
      - RFToolchain   : 三级射频检测流水线（S1-RSSI / S2-YOLO / S3-CycloAudit）
      - RA8P1 Link    : JDBG VCOM / SCI9 主控链路（RA8P1 负责最终裁决）
      - DBManager     : SQLite 告警事件持久化引擎
      - MainWindow    : PyQt5 GUI 表现层
    """
    signal_rf_frame = pyqtSignal(object)
    signal_log = pyqtSignal(str)
    signal_system_status = pyqtSignal(dict)
    signal_db_updated = pyqtSignal()
    # 标定完成通知：True=成功，False=出错；正式模式下失败不允许启动检测
    signal_calibration_done = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        
        # 初始化各子系统节点
        self.rf_toolchain = RFToolchain()
        self.ra8p1_link = RA8P1Link(
            port=cfg.RA8P1_LINK_PORT,
            baudrate=cfg.RA8P1_LINK_BAUDRATE,
        )
        self.ra8p1_online = self.ra8p1_link.start()
        self.db_engine = DBManager()
        self.diag_recorder = DiagnosticsRecorder(
            root_dir=os.path.join(PROJ_ROOT, cfg.DIAG_CAPTURE_ROOT),
            enabled=cfg.DIAG_CAPTURE_ENABLED,
            save_iq=cfg.DIAG_SAVE_IQ,
            max_iq_samples=cfg.DIAG_MAX_IQ_SAMPLES,
            max_event_records=cfg.DIAG_MAX_EVENT_RECORDS,
            max_root_bytes=cfg.DIAG_MAX_CAPTURE_ROOT_BYTES,
        )
        self.signal_log.connect(self._record_runtime_log)

        self.running        = False
        self._master_thread = None
        self._control_alive = True
        self._ra8p1_control_thread = None
        self._tick_lock     = threading.Lock()  # 防止 tick() 在 stop/start 切换时重入
        self._last_heartbeat_ts = 0.0
        self._last_master_seq = None
        self._ra8p1_scan_authorized = False
        self._calibration_thread = None
        self._calibration_running = False
        self._calibration_ok = False

        # ── 时序持久化滤波器 v4.0（Tri-Level Elastic Confirmation TPF）──────────
        #
        # 三级弹性确认窗口（N_confirm 动态由 SDS 得分区间决定）：
        #   强信号（score ≥ BYPASS_RATIO × th）：N=1，单 tick 直通
        #   中等信号（MED_RATIO × th ≤ score < BYPASS_RATIO × th）：N=2，快速确认
        #   弱信号（th ≤ score < MED_RATIO × th）：N=3，严格确认
        #
        # 虚警率模型：P_fa_final = P_fa_tick^N
        #   N=1（强）: 由 AFS+PSR+CFS 联合保证 Pfa < 0.1%
        #   N=2（中）: P_fa = 0.05² = 0.25%
        #   N=3（弱）: P_fa = 0.05³ = 0.0125%
        #
        # streak 衰减机制（v4.0 新增，替代归零）：
        #   信号消失后 streak 不立即归零，而是按 delta_decay=0.5/tick 线性衰减
        #   streak[t+1] = max(0, streak[t] - delta_decay)
        #   物理含义：信号短暂中断后 2 tick 内重现无需重新积累，降低探测延迟
        self._rf_confirm_streak     = 0.0  # v4.0: 改为 float 以支持小数衰减
        self._rf_confirm_info       = {}   # 缓存最近一次有效告警信息
        self.RF_CONFIRM_REQUIRED    = 3    # 弱信号分支：连续 3 tick 才入库报警
        self.RF_STRONG_BYPASS_RATIO = 3.0  # 强信号：score ≥ 3×th → N=1 直通
        self.RF_MED_BYPASS_RATIO    = 1.8  # 中等信号：score ≥ 1.8×th → N=2 快速确认
        self.RF_STREAK_DECAY        = 0.5  # 每 tick 衰减量（信号中断时）
        #
        # YOLO 补充分注入（v4.0 新增）：
        #   当 S3 SDS 得分在 [SDS_RESCUE_LO, SDS_RESCUE_HI) 区间（接近但未达阈值）
        #   且 YOLO bbox_score ≥ YOLO_INJ_THRESH 时，向 SDS 分注入 YOLO_INJ_WEIGHT
        #   最终判决：S_final = S_sds + YOLO_INJ_WEIGHT（若满足注入条件）
        #   注意：YOLO 单独不触发任何告警，仅作为弱信号救援补充证据
        self.YOLO_INJ_THRESH  = 0.60   # YOLO 置信度注入门限（≥60% 才有效）
        self.YOLO_INJ_WEIGHT  = 0.15   # YOLO 注入分值（使 0.85→1.00 跨越检测线）
        self.SDS_RESCUE_LO    = 0.85   # SDS 救援区间下限（低于此不注入，证据不足）
        self.SDS_RESCUE_HI    = 1.00   # SDS 救援区间上限（≥此已自行通过，无需注入）


        # RF 最新帧缓存（供证据图生成使用）
        self.cache_rf  = np.zeros((640, 640, 3),  dtype=np.uint8)
        self.ra8p1_status = {
            "link": self.ra8p1_link.summary(),
            "raw_decision": "PENDING",
            "raw_reason": "等待 RA8P1 回包",
            "final_decision": "PENDING",
            "final_reason": "等待 RA8P1 START_SCAN 与 RF 证据",
            "rf_progress": "未开始",
            "rf_metrics": "Freq -- | NCC -- | SDS --",
        }

        # 实例化 GUI 表现层并注入中央事件总线引用
        self.ui_window = MainWindow(hub=self)
        self.signal_log.emit("系统初始化完成：各子系统节点已就绪，中央事件路由建立。")
        if cfg.DIAG_CAPTURE_ENABLED:
            self.signal_log.emit(f"误报诊断记录已启用：{self.diag_recorder.session_dir}")
        if self.ra8p1_online:
            self.signal_log.emit(f"RA8P1 主控链路已连接：{self.ra8p1_link.summary()}")
        else:
            mode_text = "系统锁定，等待 RA8P1 主控接入。" if cfg.RA8P1_REQUIRED else "启用 RF 本地降级模式。"
            self.signal_log.emit(f"RA8P1 主控链路未接入：{self.ra8p1_link.summary()}，{mode_text}")

        self._ra8p1_control_thread = threading.Thread(target=self._ra8p1_control_loop, daemon=True)
        self._ra8p1_control_thread.start()

    def start_sensing(self, source: str = "local") -> bool:
        if self.running:
            return True
        if not self.ra8p1_link.online:
            self.ra8p1_online = self.ra8p1_link.start()
        if source == "ra8p1":
            self._ra8p1_scan_authorized = True
        if self._calibration_running:
            self.signal_system_status.emit({
                "system": "系统模式: [标定中] 等待标定完成",
                "color": "#f1c40f",
                "ra8p1": self.ra8p1_link.summary(),
                "final_decision": self.ra8p1_status.get("final_decision", "PENDING"),
                "final_reason": "RA8P1 START_SCAN 已记录，标定完成后自动放行采集",
                "rf_progress": "标定中",
                "pipeline_running": False,
            })
            self.signal_log.emit(f"采集启动延后：source={source}，当前正在执行背景标定。")
            return False
        if cfg.FORCE_CALIBRATION_REQUIRED and not self._calibration_ok:
            self.signal_system_status.emit({
                "system": "系统模式: [强制标定锁定]",
                "color": "#f1c40f",
                "ra8p1": self.ra8p1_link.summary(),
                "final_decision": "PENDING",
                "final_reason": "正式模式要求背景标定成功后才允许启动检测",
                "rf_progress": "等待强制标定",
                "pipeline_running": False,
            })
            self.signal_log.emit(
                f"采集管道未启动：FORCE_CALIBRATION_REQUIRED=True，当前背景标定未成功。source={source}"
            )
            return False
        if cfg.RA8P1_REQUIRED and not self.ra8p1_link.online:
            self.signal_system_status.emit({
                "system": "系统模式: [锁定] RA8P1 主控未接入",
                "color": "#f1c40f",
                "ra8p1": self.ra8p1_link.summary(),
                "final_decision": "PENDING",
                "final_reason": "RA8P1_REQUIRED=True，等待 JDBG VCOM 主控链路接入",
                "rf_progress": "链路未接入",
            })
            self.signal_log.emit(
                f"采集管道未启动：RA8P1_REQUIRED=True，但主控链路未接入：{self.ra8p1_link.summary()}"
            )
            return False
        if cfg.RA8P1_REQUIRED and source != "ra8p1":
            self.signal_system_status.emit({
                "system": "系统模式: [主控锁定] 等待 RA8P1 START_SCAN",
                "color": "#f1c40f",
                "ra8p1": self.ra8p1_link.summary(),
                "final_decision": self.ra8p1_status.get("final_decision", "PENDING"),
                "final_reason": "正式模式下采集只能由 RA8P1 主控授权启动",
                "rf_progress": "等待主控启动",
                "pipeline_running": False,
            })
            self.signal_log.emit("本地启动请求被拒绝：正式模式要求 RA8P1 下发 START_SCAN。")
            return False

        self.running = True
        self.signal_system_status.emit({
            "system": "[ACTIVE] 主管道全速轮询中...", 
            "color": "#27ae60",
            "sdr": "SDR 节点: [RX] IQ 数据采集中",
            "ra8p1": self.ra8p1_link.summary(),
            "final_decision": "CLEAR",
            "final_reason": "RF Agent 扫描中，尚无最终告警",
            "rf_progress": "扫描中",
            "ra8p1_raw_decision": self.ra8p1_status.get("raw_decision", "PENDING"),
            "ra8p1_raw_reason": self.ra8p1_status.get("raw_reason", "等待 RA8P1 回包"),
            "pipeline_running": True,
        })
        self.signal_log.emit(f"采集管道启动：source={source}，SDR 前端已进入工作状态，RA8P1 主控链路状态已同步。")
        self._master_thread = threading.Thread(target=self._hub_loop, daemon=True)
        self._master_thread.start()
        return True
        
    def _get_bypass_threshold(self, freq_mhz: float) -> float:
        """
        计算强信号旁路阈值（bypass threshold）。

        原理：
          当 S3 NCC 得分超过检测阈值 RF_STRONG_BYPASS_RATIO 倍时，
          统计上 PSR+CFS 双重门限已足够排除误报，TPF 2-tick 等待无需介入。

          bypass_th = max(th_30k, th_15k) × RF_STRONG_BYPASS_RATIO

          优先使用 calibrate_s3 写入的每扇区标定阈值；
          JSON 缺失时回退到类级别硬编码下限 × BYPASS_RATIO。

        Parameters
        ----------
        freq_mhz : float — 当前扇区中心频率（MHz）

        Returns
        -------
        float — bypass NCC 绝对阈值
        """
        try:
            thresholds = self.rf_toolchain.stage3_audit._sector_thresholds
            if thresholds:
                freq_hz = freq_mhz * 1e6
                key = min(thresholds, key=lambda k: abs(k - int(freq_hz)))
                th_30k, th_15k = thresholds[key]
                return max(th_30k, th_15k) * self.RF_STRONG_BYPASS_RATIO
        except Exception:
            pass
        # 回退：使用硬编码下限（1.8% × 3.0 = 5.4%）
        return 0.018 * self.RF_STRONG_BYPASS_RATIO

    def _record_runtime_log(self, text: str):
        try:
            self.diag_recorder.log_text(text)
        except Exception:
            pass

    def _current_iq_buffer(self):
        try:
            return self.rf_toolchain.stage2_vision.last_buffer_iq
        except Exception:
            return None

    def _format_rf_metrics(self, rf_info: dict) -> str:
        if not rf_info:
            return "Freq -- | NCC -- | SDS -- | YOLO --"
        sds_detail = rf_info.get("sds_detail", {}) or {}
        freq = float(rf_info.get("freq_mhz", 0.0))
        ncc = float(rf_info.get("score", 0.0))
        sds = float(sds_detail.get("composite", 0.0))
        yolo = float(rf_info.get("yolo_score", 0.0))
        freq_text = f"{freq:.0f} MHz" if freq > 0 else "--"
        return f"Freq {freq_text} | NCC {ncc*100:.2f}% | SDS {sds:.3f} | YOLO {yolo:.2f}"

    def _record_rf_diagnostic_event(
        self,
        event_type: str,
        rf_alert: bool,
        rf_info: dict,
        rf_log: str,
        frame_bgr,
        extra: Optional[dict] = None,
        save_iq: bool = True,
    ):
        if not cfg.DIAG_CAPTURE_ENABLED:
            return
        metadata = {
            "cycle": getattr(self.rf_toolchain, "cycle_count", None),
            "rf_alert": bool(rf_alert),
            "rf_info": rf_info or {},
            "ra8p1_link": self.ra8p1_link.summary(),
            "ra8p1_last_decision": dict(self.ra8p1_link.last_decision or {}),
            "ra8p1_status": dict(self.ra8p1_status),
            "rf_confirm_streak": self._rf_confirm_streak,
            "rf_log": rf_log,
            "config": {
                "sdr_uri": cfg.SDR_URI,
                "sdr_gain_db": cfg.SDR_GAIN_DB,
                "sample_rate": cfg.SAMPLE_RATE,
                "yolo_assist_enabled": cfg.YOLO_ASSIST_ENABLED,
                "ra8p1_required": cfg.RA8P1_REQUIRED,
            },
        }
        if extra:
            metadata.update(extra)

        self.diag_recorder.record_event(
            event_type=event_type,
            metadata=metadata,
            frame_bgr=frame_bgr,
            iq_buffer=self._current_iq_buffer() if save_iq else None,
        )

    def stop_sensing(self, source: str = "local") -> bool:
        self.running = False
        if source == "ra8p1":
            self._ra8p1_scan_authorized = False
        self.ra8p1_status["final_decision"] = "PENDING"
        self.ra8p1_status["final_reason"] = "采集管道已停止"
        self.ra8p1_status["rf_progress"] = "已停止"
        self.ra8p1_status["link"] = self.ra8p1_link.summary()
        self.signal_system_status.emit({
            "system": "系统模式: [挂起] 任务挂起",
            "color": "#f1c40f",
            "ra8p1": self.ra8p1_status["link"],
            "final_decision": "PENDING",
            "final_reason": "采集管道已停止",
            "rf_progress": "已停止",
            "pipeline_running": False,
        })
        self.signal_log.emit(f"系统中央主循环进程已安全退出执行。source={source}")
        return True

    def _pump_ra8p1_messages(self):
        """Drain RA8P1 serial messages and reflect master state in the GUI."""
        for msg in self.ra8p1_link.poll():
            msg_type = msg.get("type", "")
            self.ra8p1_status["link"] = self.ra8p1_link.summary()
            if msg_type == "MASTER_DECISION":
                decision = msg.get("decision", "PENDING")
                reason = msg.get("reason", "")
                self.ra8p1_status["raw_decision"] = decision
                self.ra8p1_status["raw_reason"] = reason
                self.signal_log.emit(
                    f"[RA8P1] MASTER_DECISION: {decision} ({reason})"
                )
                self.signal_system_status.emit({
                    "ra8p1": self.ra8p1_link.summary(),
                    "ra8p1_raw_decision": decision,
                    "ra8p1_raw_reason": reason,
                })
                if decision in {"CANDIDATE", "ALERT"}:
                    self._record_rf_diagnostic_event(
                        event_type=f"ra8p1_{decision.lower()}",
                        rf_alert=(decision == "ALERT"),
                        rf_info=self._rf_confirm_info,
                        rf_log="",
                        frame_bgr=self.cache_rf,
                        extra={"ra8p1_message": msg},
                        save_iq=(decision == "ALERT"),
                    )
            elif msg_type in {"START_SCAN", "STOP_SCAN", "RESET_ALERT", "GET_STATUS", "RUN_CALIBRATION"}:
                self._handle_ra8p1_command(msg)
            elif msg_type:
                self.signal_log.emit(f"[RA8P1] RX {msg_type}: {msg}")

    def _handle_ra8p1_command(self, msg: dict):
        msg_type = msg.get("type", "")
        seq = msg.get("seq")
        if seq is not None and seq == self._last_master_seq:
            return
        self._last_master_seq = seq

        if msg_type == "START_SCAN":
            self._ra8p1_scan_authorized = True
            if not self.running:
                self.signal_log.emit(f"[RA8P1] CMD START_SCAN: {msg}")
                self.start_sensing(source="ra8p1")
            else:
                self.ra8p1_status["link"] = self.ra8p1_link.summary()
            return

        self.signal_log.emit(f"[RA8P1] CMD {msg_type}: {msg}")
        if msg_type == "STOP_SCAN":
            self._ra8p1_scan_authorized = False
            self.stop_sensing(source="ra8p1")
        elif msg_type == "RESET_ALERT":
            self._rf_confirm_streak = 0.0
            self._rf_confirm_info = {}
            self.ra8p1_status["final_decision"] = "CLEAR"
            self.ra8p1_status["final_reason"] = "RA8P1 RESET_ALERT 已清除本地告警状态"
            self.ra8p1_status["raw_decision"] = "CLEAR"
            self.ra8p1_status["raw_reason"] = "RESET_ALERT"
            self.signal_system_status.emit({
                "ra8p1": self.ra8p1_link.summary(),
                "final_decision": "CLEAR",
                "final_reason": "RA8P1 RESET_ALERT 已清除本地告警状态",
                "ra8p1_raw_decision": "CLEAR",
                "ra8p1_raw_reason": "RESET_ALERT",
            })
        elif msg_type == "GET_STATUS":
            self.signal_system_status.emit({
                "ra8p1": self.ra8p1_link.summary(),
                "final_decision": self.ra8p1_status.get("final_decision", "PENDING"),
                "final_reason": self.ra8p1_status.get("final_reason", "状态已同步"),
                "ra8p1_raw_decision": self.ra8p1_status.get("raw_decision", "PENDING"),
                "ra8p1_raw_reason": self.ra8p1_status.get("raw_reason", "等待 RA8P1 回包"),
            })
        elif msg_type == "RUN_CALIBRATION":
            self.signal_log.emit("[RA8P1] RUN_CALIBRATION 已收到；停止采集并由 RA8P1 授权启动标定。")
            if self.running:
                self.stop_sensing(source="ra8p1")
            self.request_calibration(source="ra8p1")

    def request_calibration(self, source: str = "local") -> bool:
        if cfg.RA8P1_REQUIRED and source not in {"ra8p1", "system_boot"}:
            self.signal_log.emit("本地标定请求被拒绝：正式模式要求 RA8P1 或系统启动流程授权。")
            return False
        if self._calibration_thread and self._calibration_thread.is_alive():
            self.signal_log.emit("标定请求忽略：后台标定任务已经在运行。")
            return False

        self._calibration_running = True
        self._calibration_ok = False
        self._calibration_thread = threading.Thread(
            target=self._calibration_worker,
            args=(source,),
            daemon=True,
        )
        self._calibration_thread.start()
        return True

    def _calibration_worker(self, source: str):
        self.signal_log.emit("=" * 58)
        self.signal_log.emit(f"  [RF-Vision] 后台执行 S3 CAF-FFT 环境底噪校准，source={source}")
        self.signal_log.emit("  强制标定模式：标定成功前禁止启动检测；采集仍由 RA8P1 START_SCAN 放行")
        self.signal_log.emit("=" * 58)
        ok = True
        try:
            from rf_zynq.calibrate_s3 import main as calibrate
            calibrate()
            self.signal_log.emit("  [RF-Vision] 强制背景标定完成，新阈值已生效。")
        except Exception as exc:
            ok = False
            self.signal_log.emit(f"  [RF-Vision] 标定出错: {exc}")
            self.signal_log.emit("  -> 强制标定失败，检测管道保持锁定，不使用旧阈值启动。")
        finally:
            self._calibration_running = False
            self._calibration_ok = ok
            self.signal_calibration_done.emit(ok)
            if ok and cfg.RA8P1_REQUIRED and self._ra8p1_scan_authorized and self.ra8p1_link.online:
                self.signal_log.emit("标定完成：检测到 RA8P1 START_SCAN 授权，自动启动采集。")
                self.start_sensing(source="ra8p1")

    def _maybe_send_ra8p1_heartbeat(self):
        now = time.time()
        if now - self._last_heartbeat_ts >= 1.0:
            self.ra8p1_link.send_heartbeat()
            self._last_heartbeat_ts = now

    def _ra8p1_control_loop(self):
        """
        常驻 RA8P1 主控监听线程。

        采集管道未启动时也要处理 START_SCAN / STOP_SCAN / RESET_ALERT 等主控命令，
        否则 RA8P1 已经发出启动指令但 Hub 仍停留在待机界面。采集线程运行后，
        该线程继续负责轻量心跳和离线重连；检测报告仍由 RF 采集循环发送。
        """
        last_reconnect_ts = 0.0
        while self._control_alive:
            now = time.time()
            if not self.ra8p1_link.online and now - last_reconnect_ts >= 2.0:
                self.ra8p1_online = self.ra8p1_link.start()
                last_reconnect_ts = now
                if self.ra8p1_online:
                    self.signal_log.emit(f"RA8P1 主控链路已恢复：{self.ra8p1_link.summary()}")

            if self.ra8p1_link.online:
                self._maybe_send_ra8p1_heartbeat()
                self._pump_ra8p1_messages()

            time.sleep(0.05)

    def _send_ra8p1_detection_report(self, rf_alert: bool, rf_info: dict):
        if not self.ra8p1_link.online or not rf_info:
            return
        sds_detail = rf_info.get("sds_detail", {}) if rf_info else {}
        sds_score = float(sds_detail.get("composite", 0.0))
        freq_mhz = float(rf_info.get("freq_mhz", 0.0))
        ncc = float(rf_info.get("score", 0.0))
        suggestion = "ALERT" if rf_alert else "CLEAR"
        if not rf_alert and rf_info.get("yolo_only"):
            suggestion = "CANDIDATE"
        if rf_alert:
            self.ra8p1_link.last_decision = {}
        self.ra8p1_link.send_detection_report(
            freq_mhz=freq_mhz,
            ncc=ncc,
            sds=sds_score,
            rf_detected=bool(rf_alert),
            suggestion=suggestion,
        )
        

    def _trigger_composite_save(self, reason_tag, freq_mhz, score):
        """
        生成射频证据图像并写入告警数据库。

        将当前射频频谱帧（cache_rf）与 RA8P1 裁决信息面板水平拼接，
        叠加告警标注后，调用 DBManager 完成持久化存储。
        """
        import cv2
        panel = np.zeros((640, 640, 3), dtype=np.uint8)
        panel[:] = (12, 17, 28)
        cv2.putText(panel, "RA8P1 MASTER DECISION", (34, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (80, 210, 255), 2)
        cv2.putText(panel, f"Final    : {self.ra8p1_status.get('final_decision', 'PENDING')}", (34, 145),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.82, (80, 255, 120), 2)
        cv2.putText(panel, f"Reason   : {reason_tag}", (34, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (220, 220, 220), 2)
        cv2.putText(panel, f"Freq     : {freq_mhz:.0f} MHz", (34, 265),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (220, 220, 220), 2)
        cv2.putText(panel, f"Score    : {score:.4f}", (34, 325),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (220, 220, 220), 2)
        cv2.putText(panel, f"Link     : {self.ra8p1_status.get('link', 'JDBG UART /dev/ttyACM0')}", (34, 385),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (180, 180, 180), 2)

        fused_evidence = np.hstack([self.cache_rf, panel])
        cv2.putText(fused_evidence, f"ALARM REASON: {reason_tag}", (20, 600), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        
        new_id = self.db_engine.log_alert(freq_mhz, score, fused_evidence)
        self.signal_log.emit(f"持久化操作：事件证据标的 [REC-{new_id}] 标签属性 ({reason_tag}) 已完成写入。")
        
        # 通知异步界面的前端模型重新加载历史缓存
        self.signal_db_updated.emit()

    def _hub_loop(self):
        """
        主监测循环（在独立守护线程中运行）。

        管线一（RF）：调用 RFToolchain.tick() 执行一次三级检测周期，
                      处理告警结果并更新 GUI 状态。
        RA8P1 主控裁决链路通过 JDBG VCOM / SCI9 接入。比赛默认配置要求
        RA8P1 在线，RF Agent 只负责外置算力检测与结果上报。
        """
        while self.running:
            # === [处理管线一：软件无线电跳频解调] ===
            if not self._tick_lock.acquire(blocking=False):
                time.sleep(0.01)  # 上一次 tick 尚未完成，等待后重试
                continue
            try:
                rf_frame, rf_log, rf_alert, rf_info = self.rf_toolchain.tick()
                self.cache_rf = rf_frame
                self.signal_rf_frame.emit(rf_frame)

                if rf_log.strip():
                    self.signal_log.emit(rf_log)

                # ── 自适应时序持久化滤波器 v4.0（Tri-Level Elastic TPF）────────────
                # 分支 A — 强信号直通（N=1）:
                #   score ≥ RF_STRONG_BYPASS_RATIO × th → 单 tick 直通
                #   AFS+PSR+CFS 联合保证 Pfa < 0.1%
                # 分支 B — 中等信号（N=2）:
                #   RF_MED_BYPASS_RATIO × th ≤ score < RF_STRONG_BYPASS_RATIO × th
                #   P_fa_final = P_fa_tick² ≈ 0.25%
                # 分支 C — 弱信号（N=3）:
                #   th ≤ score < RF_MED_BYPASS_RATIO × th
                #   P_fa_final = P_fa_tick³ ≈ 0.0125%
                # 分支 D — 噪声：streak 衰减（不归零，保留历史积累）
                #   streak[t+1] = max(0, streak[t] − RF_STREAK_DECAY)
                # ────────────────────────────────────────────────────────────────

                # YOLO 补充分注入（v4.0）
                # 受 cfg.YOLO_ASSIST_ENABLED 开关控制：
                #   False（当前默认）→ 整段逻辑跳过，S2 完全不参与 SDS 判决
                #   True            → 开启弱信号救援注入（需先完成 5.8GHz 数据集重训）
                # 启用条件见 config.py YOLO_ASSIST_ENABLED 注释。
                if cfg.YOLO_ASSIST_ENABLED:
                    yolo_score    = rf_info.get("yolo_score", 0.0) if rf_info else 0.0
                    sds_detail    = rf_info.get("sds_detail", {}) if rf_info else {}
                    sds_composite = sds_detail.get("composite", 0.0)
                    yolo_inject   = (
                        (not rf_alert)
                        and (self.SDS_RESCUE_LO <= sds_composite < self.SDS_RESCUE_HI)
                        and (yolo_score >= self.YOLO_INJ_THRESH)
                    )
                    if yolo_inject:
                        sds_final = sds_composite + self.YOLO_INJ_WEIGHT
                        self.signal_log.emit(
                            f"[RF-YOLO] SDS 救援注入：SDS={sds_composite:.3f} + "
                            f"YOLO({yolo_score:.2f})×{self.YOLO_INJ_WEIGHT} "
                            f"→ S_final={sds_final:.3f} ≥ 1.0 → 强制触发告警"
                        )
                        rf_alert = True  # 覆盖 S3 的 False，允许进入 TPF 确认流程
                        if not rf_info:
                            rf_info = {}
                        rf_info["score"] = sds_final  # 用修正后的综合分替代原始 NCC

                self._send_ra8p1_detection_report(rf_alert, rf_info)
                self._pump_ra8p1_messages()
                if rf_info:
                    sds_detail = rf_info.get("sds_detail", {}) if rf_info else {}
                    sds_composite = float(sds_detail.get("composite", 0.0))
                    should_capture = (
                        rf_alert
                        or rf_info.get("yolo_only")
                        or sds_composite >= 0.75
                    )
                    if should_capture:
                        if rf_alert:
                            event_type = "rf_s3_pass"
                        elif rf_info.get("yolo_only"):
                            event_type = "rf_yolo_only"
                        else:
                            event_type = "rf_sds_near_miss"
                        self._record_rf_diagnostic_event(
                            event_type=event_type,
                            rf_alert=rf_alert,
                            rf_info=rf_info,
                            rf_log=rf_log,
                            frame_bgr=rf_frame,
                            save_iq=bool(rf_alert),
                        )

                if rf_alert:
                    freq_mhz  = rf_info.get("freq_mhz", 0.0)
                    score     = rf_info.get("score",    0.0)
                    rf_metrics = self._format_rf_metrics(rf_info)
                    bypass_th = self._get_bypass_threshold(freq_mhz)
                    med_th    = bypass_th / self.RF_STRONG_BYPASS_RATIO * self.RF_MED_BYPASS_RATIO

                    if score >= bypass_th:
                        # ── 分支 A：强信号直通（N=1）────────────────────────────
                        n_required = 1
                        self._rf_confirm_streak = float(self.RF_CONFIRM_REQUIRED)  # 置满
                        self._rf_confirm_info   = rf_info
                        self.signal_log.emit(
                            f"[RF-PRE] 强信号直通（N=1）"
                            f" @ {freq_mhz:.0f} MHz  "
                            f"NCC={score*100:.2f}%（≥ {bypass_th*100:.2f}% 旁路阈值）")
                        rf_progress = f"RF 强证据确认 {self._rf_confirm_streak:.1f}/{n_required}"
                    elif score >= med_th:
                        # ── 分支 B：中等信号（N=2）────────────────────────────
                        n_required = 2
                        self._rf_confirm_streak = min(
                            self._rf_confirm_streak + 1.0,
                            float(self.RF_CONFIRM_REQUIRED)
                        )
                        self._rf_confirm_info = rf_info
                        streak = self._rf_confirm_streak
                        self.signal_log.emit(
                            f"[RF-PRE] 中等信号（N=2）({streak:.1f}/{n_required})"
                            f" @ {freq_mhz:.0f} MHz  "
                            f"NCC={score*100:.1f}%")
                        rf_progress = f"RF 中等证据确认 {streak:.1f}/{n_required}"
                    else:
                        # ── 分支 C：弱信号（N=3）──────────────────────────────
                        n_required = 3
                        self._rf_confirm_streak = min(
                            self._rf_confirm_streak + 1.0,
                            float(self.RF_CONFIRM_REQUIRED)
                        )
                        self._rf_confirm_info = rf_info
                        streak = self._rf_confirm_streak
                        self.signal_log.emit(
                            f"[RF-PRE] 弱信号（N=3）({streak:.1f}/{n_required})"
                            f" @ {freq_mhz:.0f} MHz  "
                            f"NCC={score*100:.1f}%  等待第 {n_required} 次确认...")
                        rf_progress = f"RF 弱证据确认 {streak:.1f}/{n_required}"

                    self.ra8p1_status["rf_progress"] = rf_progress
                    self.ra8p1_status["rf_metrics"] = rf_metrics
                    self.signal_system_status.emit({
                        "ra8p1": self.ra8p1_link.summary(),
                        "final_decision": self.ra8p1_status.get("final_decision", "CLEAR"),
                        "final_reason": self.ra8p1_status.get("final_reason", "尚未形成最终告警"),
                        "ra8p1_raw_decision": self.ra8p1_status.get("raw_decision", "PENDING"),
                        "ra8p1_raw_reason": self.ra8p1_status.get("raw_reason", "等待 RA8P1 回包"),
                        "rf_progress": rf_progress,
                        "rf_metrics": rf_metrics,
                    })

                    if self._rf_confirm_streak >= n_required:
                        # 达标 → 发出最终告警
                        decision_reason = f"RF_LOCAL_CONFIRMED_N{n_required}"
                        final_decision = "ALERT"
                        if self.ra8p1_link.online and self.ra8p1_link.last_decision:
                            final_decision = self.ra8p1_link.last_decision.get("decision", "CANDIDATE")
                            decision_reason = self.ra8p1_link.last_decision.get("reason", decision_reason)
                        elif self.ra8p1_link.online:
                            final_decision = "CANDIDATE"
                            decision_reason = "等待 RA8P1 MASTER_DECISION"

                        display_final = "ALERT" if final_decision == "ALERT" else "CLEAR"
                        final_reason = (
                            decision_reason
                            if final_decision == "ALERT"
                            else f"RF已达本地确认，但RA8P1未给出最终告警：{decision_reason}"
                        )
                        self.ra8p1_status["link"] = self.ra8p1_link.summary()
                        self.ra8p1_status["final_decision"] = display_final
                        self.ra8p1_status["final_reason"] = final_reason
                        self.signal_system_status.emit({
                            "system": "[!] Alert: OcuSync RF Detected!" if final_decision == "ALERT" else "RA8P1 主控未确认最终告警",
                            "color":  "#e74c3c" if final_decision == "ALERT" else "#f1c40f",
                            "alert":  final_decision == "ALERT",
                            "ra8p1": self.ra8p1_link.summary(),
                            "final_decision": display_final,
                            "final_reason": final_reason,
                            "ra8p1_raw_decision": self.ra8p1_status.get("raw_decision", "PENDING"),
                            "ra8p1_raw_reason": self.ra8p1_status.get("raw_reason", "等待 RA8P1 回包"),
                            "rf_progress": rf_progress,
                            "rf_metrics": rf_metrics,
                        })
                        freq      = self._rf_confirm_info.get("freq_mhz", 0.0)
                        score_out = self._rf_confirm_info.get("score",    0.0)
                        yolo_out  = self._rf_confirm_info.get("yolo_score", 0.0)
                        yolo_tag  = f"  YOLO={yolo_out:.2f}" if yolo_out > 0 else ""
                        if final_decision == "ALERT":
                            self.signal_log.emit(
                                f'<span style="color:#ef4444; font-weight:bold;">'
                                f'⚠ [RF-ALARM] RA8P1 主控确认 OcuSync 告警（N={n_required}）！'
                                f' @ {freq:.0f} MHz  NCC={score_out*100:.2f}%{yolo_tag}'
                                f' — 告警已入库，证据图像正在写入...</span>')
                            self._trigger_composite_save("SDR_RA8P1_MASTER_TRIGGER", freq, score_out)
                        else:
                            self.signal_log.emit(
                                f"[RF-CANDIDATE] RF 本地已确认，但 RA8P1 当前裁决为 "
                                f"{final_decision}：{decision_reason}"
                            )
                        # 不重置 streak，维持告警状态直到信号消失

                else:
                    # ── 分支 D：S3+YOLO 均未通过，streak 衰减（v4.0：不归零）────
                    prev_streak = self._rf_confirm_streak
                    self._rf_confirm_streak = max(
                        0.0, self._rf_confirm_streak - self.RF_STREAK_DECAY
                    )
                    if prev_streak > 0 and self._rf_confirm_streak <= 0:
                        self.signal_log.emit(
                            f"[RF-SCAN] streak 衰减完毕 "
                            f"({prev_streak:.1f} → 0)，继续扫描")
                    elif prev_streak > self._rf_confirm_streak + 0.01:
                        self.signal_log.emit(
                            f"[RF-SCAN] streak 衰减中 "
                            f"({prev_streak:.1f} → {self._rf_confirm_streak:.1f})")
                    freq_display = rf_info.get('freq_mhz', 0) if rf_info else 0
                    if self._rf_confirm_streak <= 0:
                        self.ra8p1_status["final_decision"] = "CLEAR"
                        self.ra8p1_status["final_reason"] = "RF 未确认告警，持续扫描"
                        self.ra8p1_status["rf_progress"] = "扫描中"
                        self.ra8p1_status["rf_metrics"] = self._format_rf_metrics(rf_info)
                    self.ra8p1_status["link"] = self.ra8p1_link.summary()
                    rf_progress = "扫描中" if self._rf_confirm_streak <= 0 else f"RF 候选目标确认中 {self._rf_confirm_streak:.1f}/3"
                    self.signal_system_status.emit({
                        "system": f"系统状态: [扫描] ({freq_display:.0f}MHz)" if freq_display else "系统状态: 主管道全速扫描中...",
                        "color": "#27ae60",
                        "ra8p1": self.ra8p1_link.summary(),
                        "final_decision": "CLEAR",
                        "final_reason": "RF 未确认告警，持续扫描" if self._rf_confirm_streak <= 0 else "RF 候选目标确认中，未形成最终告警",
                        "ra8p1_raw_decision": self.ra8p1_status.get("raw_decision", "PENDING"),
                        "ra8p1_raw_reason": self.ra8p1_status.get("raw_reason", "等待 RA8P1 回包"),
                        "rf_progress": rf_progress,
                        "rf_metrics": self._format_rf_metrics(rf_info),
                    })

            except Exception as e:
                self.signal_log.emit(f"SDR 射频传感器寻址异常: {e}")
            finally:
                self._tick_lock.release()

            self._maybe_send_ra8p1_heartbeat()
            self._pump_ra8p1_messages()
            time.sleep(0.001)  # 1ms 让出 CPU 给 Qt 事件循环，避免 UI 卡顿

    def shutdown(self):
        self._control_alive = False
        self.stop_sensing()
        if self._ra8p1_control_thread:
            self._ra8p1_control_thread.join(timeout=0.5)
        self.ra8p1_link.stop()

if __name__ == "__main__":
    # ── Step 1: 安装 stdout 重定向器 ──────────────────────────────────────────
    # 在 QApplication 和 hub 创建之前安装，确保 RFToolchain 初始化期间的
    # 所有 print() 输出（如 S3 阈值加载信息）都被捕获，最终显示在 GUI 日志框。
    _redirector = _GuiLogRedirector()
    sys.stdout  = _redirector
    sys.stderr  = _redirector   # 同时捕获异常 traceback 与警告信息

    # ── Step 2: 启动 Qt 应用与 Hub（GUI 优先弹出，不等待标定）────────────────
    app = QApplication(sys.argv)
    hub = CentralHubEngine()       # 内部创建 MainWindow 及 RFToolchain
                                   # 期间的 print() 进入 _redirector 缓冲区
    hub.ui_window.showFullScreen()  # 全屏显示，立即弹出无需等待标定完成

    # ── Step 3: 绑定信号，flush 缓冲区至 GUI 日志框 ───────────────────────────
    # attach_signal 调用后，已缓冲的所有初始化日志立即出现在日志框，
    # 后续任何 print() 也将实时路由到 GUI。
    _redirector.attach_signal(hub.signal_log.emit)

    # ── Step 4: 后台线程执行环境底噪校准（非阻塞，GUI 保持响应）─────────────
    # 统一走 Hub 的正式控制入口；正式模式下后续采集仍需 RA8P1 START_SCAN 放行。
    hub.request_calibration(source="system_boot")


    # ── Step 5: 进入 Qt 主事件循环 ───────────────────────────────────────────
    exit_code = app.exec_()

    # ── Step 6: 恢复标准流并关闭子系统 ──────────────────────────────────────
    sys.stdout = sys.__stdout__
    sys.stderr = sys.__stderr__
    hub.shutdown()
    sys.exit(exit_code)


# ==============================================================================
# [DEBUG ONLY 临时隔离运行区块]: 关闭主控时自动清理历史调试污染数据
# 注意：在完成所有早期开发与算法测试后，请直接注释下方的 atexit 注册块。
# 本代码块确保即使主进程被异常强杀（Ctrl+C 或报错闪退），也能坚定触发垃圾强制回收！
# ==============================================================================
""" import atexit
import subprocess
import os

def _auto_clean_debug_traces():
    print("\n[HOOK] 主控引擎开始降下帷幕，正在自动拉起清道夫 (clean_debug_data.bat)...")
    bat_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "clean_debug_data.bat")
    
    if os.path.exists(bat_path):
        try:
            # 采用静默的 shell 衍生执行，防止终端阻塞
            subprocess.run(f'cmd /c "{bat_path}"', shell=True)
            print("[HOOK] 数据库垃圾强制排空完毕，主控安全释放所有内存！")
        except Exception as e:
            print(f"[!] 清除脚本应急调用断帧故障: {e}")

atexit.register(_auto_clean_debug_traces) """
# ==============================================================================
