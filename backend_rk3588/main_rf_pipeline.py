"""
主射频检测流水线 v4.0（RF Detection Pipeline）
==============================================
实现三级级联检测架构（S1 → S2 → S3），正式版采用纯 RF 物理证据链：

  Stage 1 (S1) v4.0: 峰度加权快速预扫（Kurtosis-Weighted Pre-scan）
    — 引入信号峰度 κ = E[|x|⁴]/(E[|x|²])² 加权扇区排名
    — 有效感知低占空比 OcuSync 突发帧的弱信号扇区（κ≫3→权重提升 40~80%）
    — S1_BUFFER_SIZE=524288（13.1ms），3帧中值滤波，smooth_alpha=0.35

  Stage 2 (S2): 短时驻留与频谱成像（Dwell & Waterfall Display）
    — 在 S1 选定扇区采集高时间分辨率 IQ 数据，生成 640×640 短时傅里叶变换瀑布图
    — 瀑布图只用于大屏展示和人工观察，不参与最终判决

  Stage 3 (S3) v4.0: 四重互验证 + 软判决评分融合
    — CAF-FFT 循环谱 + PSR + CFS + AFS（α 域频率稳定性）四重正交验证
    — SDS（软判决评分）代替纯硬串联：S = 0.45·NCC + 0.25·PSR + 0.20·CFS + 0.10·AFS
    — 软下限 0.80×th 允许微弱信号在其他证据充分时被综合评分救援
    — CHUNK_SIZE=160000（4ms），OVERLAP=0.80，PEAK_WEIGHT=0.65

平台支持：
  - 开发机 / Orange Pi 5: 均使用 S1 + S2 瀑布图 + S3 循环谱确认
"""

import os
import sys
import time
import numpy as np
import cv2

# 项目根目录加入模块搜索路径
PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJ_ROOT not in sys.path:
    sys.path.insert(0, PROJ_ROOT)

from backend_rk3588 import config as cfg
from rf_zynq.rf_stage1_rssi_scan import RF_Stage1_RSSIScan
from rf_zynq.rf_stage2_waterfall import RF_Stage2_Dwell
from rf_zynq.rf_stage3_cyclostationary import RF_Stage3_CycloAudit


class RFToolchain:
    """
    三级射频检测流水线主控类（S1-RSSI → S2-Waterfall → S3-CycloAudit）

    tick() 方法执行一次完整的检测周期：
      1. S1 快速 RSSI 预扫，获取各扇区信号功率排名
      2. 将 SDR 调谐至功率最强扇区
      3. S2 采集 IQ 数据、生成频谱瀑布图（大屏展示）
      4. S3 对同一段 IQ 数据执行循环谱审计，输出最终告警判决

    硬件配置（来自 config.py）：
      SDR  : AD9364 + ZYNQ7020，采样率 40 MSps，增益 {cfg.SDR_GAIN_DB} dB
      频段 : 5725~5845 MHz（DJI OcuSync 5.8 GHz 全频段）
      缓冲 : 2,621,440 采样点 = 65.5 ms 连续时间切片
    """

    def __init__(self):
        import adi
        try:
            self.sdr = adi.Pluto(cfg.SDR_URI)
            self.sample_rate             = cfg.SAMPLE_RATE
            self.sdr.sample_rate         = self.sample_rate
            self.sdr.rx_rf_bandwidth     = self.sample_rate
            self.sdr.rx_buffer_size      = 2621440
            self.sdr.rx_hardwaregain_control_mode = 'manual'
            self.sdr.rx_hardwaregain_chan0        = cfg.SDR_GAIN_DB
            print(f"[INFO] RFToolchain: AD9364 初始化完成 "
                  f"(URI={cfg.SDR_URI}, Gain={cfg.SDR_GAIN_DB} dB, "
                  f"Backend=S1+S2-Waterfall+S3)")
        except Exception as e:
            print(f"[ERROR] RFToolchain: SDR 初始化失败: {e}")
            raise

        self.stage2_vision = RF_Stage2_Dwell(self.sdr)
        self.stage3_audit  = RF_Stage3_CycloAudit(sample_rate=self.sample_rate)
        self.stage1_rssi   = RF_Stage1_RSSIScan(self.sdr, cfg.SWEEP_SECTORS, self.sample_rate)
        self.cycle_count   = 0

    def tick(self):
        """
        执行一次完整的三级检测周期（v4.0）。

        Returns
        -------
        (ndarray, str, bool, dict) :
          annotated_frame : 带标注的频谱瀑布图（640×640×3 BGR）
          log_text        : 本周期诊断日志
          alert_flag      : S3 初步告警标志（由 system_hub TPF 二次确认后才发出最终告警）
          alert_info      : 告警附属信息，v4.0 新增字段：
            · freq_mhz    : 当前扇区中心频率（MHz）
            · score       : S3 NCC 联合统计量
            · sds_detail  : S3 内部 SDS 分项评分（诊断用）
        """
        self.cycle_count += 1
        log_lines = []

        # ── Stage 1：峰度加权快速预扫 v4.0 ──────────────────────────────────
        ranked_sectors        = self.stage1_rssi.scan_and_rank()
        active_freq, s1_score = ranked_sectors[0]
        log_lines.append(
            f"\n===== [Cycle {self.cycle_count}] "
            f"S1 Priority: {active_freq/1e6:.0f} MHz "
            f"(κ-weighted score = {s1_score*1e6:.2f} μW-eq) ====="
        )

        # ── Stage 2：LO 调谐与频谱成像 ────────────────────────────────────────
        t0 = time.time()
        self.sdr.rx_lo = int(active_freq)
        time.sleep(0.04)               # PLL 锁定等待（40 ms）
        self.sdr.rx_destroy_buffer()
        waterfall_tensor = self.stage2_vision.generate_waterfall_tensor(active_freq)

        annotated_frame = waterfall_tensor.copy()
        cost_s2 = time.time() - t0

        cv2.putText(annotated_frame, f"SECTOR: {active_freq/1e6:.0f} MHz",
                    (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(annotated_frame,
                    "WATERFALL DISPLAY",
                    (10, 620), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (170, 220, 255), 2)

        log_lines.append(
            f"[S2] Freq={active_freq/1e6:.0f} MHz | "
            f"Cost={cost_s2:.3f} s | "
            f"Waterfall=ON | Inference=DISABLED"
        )

        # ── Stage 3：循环谱物理层审计 v4.0 ────────────────────────────────────
        t1 = time.time()
        confirm_flag, audit_score = self.stage3_audit.run_spectral_audit(
            self.stage2_vision.last_buffer_iq,
            sector_hz=active_freq,
        )
        sds_detail = dict(self.stage3_audit.last_sds_detail)  # 取一份诊断副本
        cost_s3 = time.time() - t1

        # SDS 分项摘要（仅在有评分时显示）
        sds_log = ""
        if sds_detail:
            sds_log = (
                f" | SDS: NCC={sds_detail.get('ncc_ratio',0):.2f} "
                f"PSR={sds_detail.get('psr_score',0):.2f} "
                f"CFS={sds_detail.get('cfs_score',0):.2f} "
                f"AFS={sds_detail.get('afs_score',0):.1f} "
                f"→ S={sds_detail.get('composite',0):.3f}"
            )
        log_lines.append(
            f"[S3] Cost={cost_s3:.3f} s | "
            f"Result={'DETECTED' if confirm_flag else 'NEGATIVE'} "
            f"(NCC={audit_score:.4f}){sds_log}"
        )

        alert_flag = False
        alert_info = {}

        if confirm_flag:
            log_lines.append(
                f"[S3-PASS] OcuSync 协议特征吻合 "
                f"@ {active_freq/1e6:.0f} MHz  "
                f"NCC={audit_score:.4f}  → 提交 TPF 确认"
            )
            alert_flag = True
            alert_info = {
                "freq_mhz":   active_freq / 1e6,
                "score":      audit_score,
                "sds_detail": sds_detail,           # v4.0 新增：SDS 分项诊断
            }
            cv2.putText(annotated_frame, f"S3 PASS  NCC={audit_score:.3f}",
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

        return annotated_frame, "\n".join(log_lines), alert_flag, alert_info
