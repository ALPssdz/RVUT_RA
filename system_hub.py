#!/usr/bin/env python3
import torch # [Patch]: 规避 PyTorch 与 PyQt5 的 C++ 动态链接库初始化冲突 (WinError 1114)
import sys
import os
import time
import threading
import numpy as np

PROJ_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJ_ROOT not in sys.path:
    sys.path.insert(0, PROJ_ROOT)

from PyQt5.QtCore import Qt, QEvent
from PyQt5.QtGui import QImage, QPixmap, QFont, QKeySequence
from PyQt5.QtWidgets import QApplication, QShortcut
from PyQt5.QtCore import QObject, pyqtSignal

from backend_rk3588 import config as cfg
from backend_rk3588.main_rf_pipeline import RFToolchain
from vision_k230.k230_client import K230NetworkClient
from ui_qt.gui_host import MainWindow
from database.db_manager import DBManager

class CentralHubEngine(QObject):
    """
    系统级中央调度引擎（Central Orchestration Engine）

    基于事件总线（Event Bus）架构，负责协调射频检测子系统与光电视觉子系统
    的并发数据流，执行跨模态特征融合对齐，将告警事件持久化写入数据库，
    并通过 Qt 信号机制向 GUI 表现层广播实时状态。

    子系统组成：
      - RFToolchain   : 三级射频检测流水线（S1-RSSI / S2-YOLO / S3-CycloAudit）
      - K230Client    : K230 边缘端光电视觉流接收与带外信令解析
      - DBManager     : SQLite 告警事件持久化引擎
      - MainWindow    : PyQt5 GUI 表现层
    """
    signal_rf_frame = pyqtSignal(object)
    signal_k230_frame = pyqtSignal(object)
    signal_log = pyqtSignal(str)
    
    # 针对表现层的聚合状态负载
    signal_system_status = pyqtSignal(dict)
    signal_db_updated = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        
        # 初始化各子系统节点
        self.rf_toolchain = RFToolchain()
        self.k230_client  = K230NetworkClient(
            rtsp_url=cfg.K230_RTSP_URL, udp_port=cfg.K230_UDP_PORT
        )
        self.k230_client.start()
        self.db_engine = DBManager()

        self.running        = False
        self._master_thread = None
        self._tick_lock     = threading.Lock()  # 防止 tick() 在 stop/start 切换时重入

        # ── 时序持久化滤波器（Temporal Persistence Filter）──────────────────
        # 原理：
        #   单次 SMPS 瞬态可偶发通过 PSR+CFS 门限（1 tick 误报）
        #   真实 OcuSync 信号持续传输，连续多个 tick 均会确认
        # 实现：
        #   _rf_confirm_streak  : 当前连续 S3 确认 tick 数
        #   RF_CONFIRM_REQUIRED : 发出最终告警所需的连续确认数（默认 2）
        # 延迟代价：增加 1 个检测周期（约 1~2 秒），对告警响应速度影响极小
        self._rf_confirm_streak  = 0
        self._rf_confirm_info    = {}   # 缓存最近一次有效告警信息
        self.RF_CONFIRM_REQUIRED = 2    # 连续 2 tick 确认才入库报警

        # RF 与视觉通道的最新帧缓存（供多模态证据融合使用）
        self.cache_rf  = np.zeros((640, 640, 3),  dtype=np.uint8)
        self.cache_vis = np.zeros((640, 1137, 3), dtype=np.uint8)

        # 实例化 GUI 表现层并注入中央事件总线引用
        self.ui_window = MainWindow(hub=self)
        self.signal_log.emit("系统初始化完成：各子系统节点已就绪，中央事件路由建立。")

    def start_sensing(self):
        if self.running: return
        self.running = True
        self.signal_system_status.emit({
            "system": "[ACTIVE] 主管道全速轮询中...", 
            "color": "#27ae60",
            "sdr": "SDR 节点: [RX] IQ 数据采集中",
            "vision": "视频节点: [RX] 画面及信令监听中"
        })
        self.signal_log.emit("采集管道启动：SDR 前端及视觉网络客户端已进入工作状态。")
        self._master_thread = threading.Thread(target=self._hub_loop, daemon=True)
        self._master_thread.start()
        # 在 Linux/RK3588 上将检测线程绑定至 A76 大核（cpu4-7），
        # Qt 主线程（监控 QTimer）自然运行在 A55 小核，双方不竞争同一核
        try:
            from ui_qt.orangepi_monitor import pin_thread_to_big_cores
            # 需在线程启动后稍等片刻，确保 OS 已分配 tid
            import time as _t; _t.sleep(0.05)
            self._master_thread_pin = threading.Thread(
                target=pin_thread_to_big_cores, daemon=True)
            self._master_thread_pin.start()
        except Exception:
            pass
        
    def stop_sensing(self):
        self.running = False
        self.signal_system_status.emit({"system": "系统模式: [挂起] 任务挂起", "color": "#f1c40f"})
        self.signal_log.emit("系统中央主循环进程已安全退出执行。")
        

    def _trigger_composite_save(self, reason_tag, freq_mhz, score):
        """
        生成多模态证据融合图像并写入告警数据库。

        将当前射频频谱帧（cache_rf）与视觉图像帧（cache_vis）水平拼接，
        叠加告警标注后，调用 DBManager 完成持久化存储。
        """
        import cv2
        fused_evidence = np.hstack([self.cache_rf, self.cache_vis])
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
        管线二（光电视觉）：从 K230 客户端获取视频帧与带外信令，
                           若检测到目标则生成告警记录。
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

                # ── 时序持久化滤波器 ────────────────────────────────────────
                # 数学模型：
                #   设每个 tick S3 的虚警概率为 P_fa_tick（单次）
                #   N 次连续均虚警的概率：P_fa_final = P_fa_tick^N
                #   N=2: P_fa_final = P_fa_tick²（大幅抑制偶发虚报）
                #   对 OcuSync（P_detect_tick ≈ 0.95）：
                #     P_detect_final = 1-(1-0.95^2) ≈ 90%  （2连续）
                # ────────────────────────────────────────────────────────────
                if rf_alert:
                    self._rf_confirm_streak += 1
                    self._rf_confirm_info    = rf_info   # 缓存当前告警信息
                    streak = self._rf_confirm_streak
                    self.signal_log.emit(
                        f"[TPF] S3 确认 {streak}/{self.RF_CONFIRM_REQUIRED} tick "
                        f"(freq={rf_info.get('freq_mhz',0):.0f}MHz "
                        f"score={rf_info.get('score',0)*100:.1f}%)")

                    if streak >= self.RF_CONFIRM_REQUIRED:
                        # 连续确认达标 → 发出最终告警
                        self.signal_system_status.emit(
                            {"system": "[!] Alert: OcuSync RF Detected!", "color": "#e74c3c"})
                        freq  = self._rf_confirm_info.get("freq_mhz", 0.0)
                        score = self._rf_confirm_info.get("score",    0.0)
                        self._trigger_composite_save("SDR_OMNI_TRIGGER", freq, score)
                        # 不重置 streak，维持告警状态直到信号消失
                else:
                    # S3 未通过：重置连续计数器
                    if self._rf_confirm_streak > 0:
                        self.signal_log.emit(
                            f"[TPF] 连续确认中断（streak 重置: "
                            f"{self._rf_confirm_streak} → 0）")
                    self._rf_confirm_streak = 0
                    self.signal_system_status.emit({
                        "system": f"系统状态: [扫描] ({rf_info.get('freq_mhz', 0):.0f}MHz)" if rf_info else "系统状态: 主管道全速扫描中...",
                        "color": "#27ae60"
                    })
            except Exception as e:
                self.signal_log.emit(f"SDR 射频传感器寻址异常: {e}")
            finally:
                self._tick_lock.release()
                
            # === [处理管线二：带外信令网络及边缘端光学流] ===
            try:
                k_frame, k_telemetry = self.k230_client.get_synced_data()
                
                if k_telemetry.get("alert", False):
                    bbox = k_telemetry.get("bbox", [])
                    if len(bbox) == 4:
                        import cv2
                        cv2.rectangle(k_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 0, 255), 8)
                        cv2.putText(k_frame, "OOB JSON LOCK", (bbox[0], bbox[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                        
                    self.signal_system_status.emit({"system": "特征命中：目标物理轮廓验证通过", "color": "#e74c3c"})
                    self.signal_log.emit("OOB 触发器：高速网络侧带外接收到正向标定数据包。")
                    self._trigger_composite_save("K230_ZENITH_TRIGGER", 0.0, 1.0)
                
                self.cache_vis = k_frame
                self.signal_k230_frame.emit(k_frame)
            except Exception as e:
                self.signal_log.emit(f"边缘侧光学流推流挂起异常: {e}")
                
            time.sleep(0.001)  # 1ms 让出 CPU 给 Qt 事件循环，避免 UI 卡顿

    def shutdown(self):
        self.stop_sensing()
        self.k230_client.stop()

if __name__ == "__main__":
    # ── 每次启动前自动测量环境底噪并更新 S3 阈值 ──────────────────────────────
    print("[RF-Vision] 正在执行 S3 CAF-FFT 环境底噪自动校准...")
    try:
        from rf_zynq.calibrate_s3 import main as _calibrate
        _calibrate()
        print("[RF-Vision] 底噪校准完成，正在启动主系统...\n")
    except Exception as _e:
        print(f"[RF-Vision] 校准过程出错（{_e}），使用上次保存的阈值继续启动。\n")

    app = QApplication(sys.argv)
    hub = CentralHubEngine()
    hub.ui_window.showFullScreen()
    
    exit_code = app.exec_()
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
