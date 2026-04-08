"""
主射频检测流水线 v4.0（RF Detection Pipeline）
==============================================
实现三级级联检测架构（S1 → S2 → S3），v4.0 引入跨级信息融合：

  Stage 1 (S1) v4.0: 峰度加权快速预扫（Kurtosis-Weighted Pre-scan）
    — 引入信号峰度 κ = E[|x|⁴]/(E[|x|²])² 加权扇区排名
    — 有效感知低占空比 OcuSync 突发帧的弱信号扇区（κ≫3→权重提升 40~80%）
    — S1_BUFFER_SIZE=524288（13.1ms），3帧中值滤波，smooth_alpha=0.35

  Stage 2 (S2): 短时驻留与频谱成像（Dwell & Spectrogram Generation）
    — 在 S1 选定扇区采集高时间分辨率 IQ 数据，生成 640×640 短时傅里叶变换瀑布图
    — 运行 YOLOv8 目标检测（平台自适应：x86_64 用 torch，aarch64 用 RKNN NPU）
    — v4.0：YOLO bbox_score 注入 alert_info，供 system_hub SDS 评分融合使用

  Stage 3 (S3) v4.0: 四重互验证 + 软判决评分融合
    — CAF-FFT 循环谱 + PSR + CFS + AFS（α 域频率稳定性）四重正交验证
    — SDS（软判决评分）代替纯硬串联：S = 0.45·NCC + 0.25·PSR + 0.20·CFS + 0.10·AFS
    — 软下限 0.80×th 允许微弱信号在其他证据充分时被综合评分救援
    — CHUNK_SIZE=160000（4ms），OVERLAP=0.80，PEAK_WEIGHT=0.65

平台支持：
  - 开发机 (x86_64)  : YOLO 使用 ultralytics + PyTorch CPU 推理
  - Orange Pi 5 (aarch64): YOLO 使用 rknn_toolkit_lite2 + RK3588 NPU 推理
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
from rf_zynq.rf_stage2_waterfall_yolo import RF_Stage2_Dwell
from rf_zynq.rf_stage3_cyclostationary import RF_Stage3_CycloAudit


def load_yolo_model():
    """
    平台自适应 YOLO 模型加载函数。

    推理后端选择逻辑：
      config.YOLO_BACKEND == "rknn"  → RKNNLiteInfer（RK3588 NPU，约 20~40 ms/帧）
      config.YOLO_BACKEND == "torch" → ultralytics YOLO（CPU，约 150~300 ms/帧）

    RKNN 模型路径：config.YOLO_RKNN_PATH（相对于项目根目录）
    若 .rknn 文件不存在，自动降级至 torch 推理并输出警告。
    """
    if cfg.YOLO_BACKEND == "rknn":
        rknn_abs = os.path.join(PROJ_ROOT, cfg.YOLO_RKNN_PATH)
        if os.path.exists(rknn_abs):
            from rf_zynq.rknn_infer import RKNNLiteInfer
            return RKNNLiteInfer(rknn_abs, conf_thresh=cfg.YOLO_CONF_THRESH)
        else:
            print(f"[WARN] RKNN 模型未找到: {rknn_abs}")
            print(f"       请在 WSL2/Linux 上运行 tools/convert_yolo_to_rknn.py 完成转换。")
            print(f"       本次自动降级为 PyTorch CPU 推理。")

    # torch 推理路径（默认 / 降级）
    from ultralytics import YOLO
    import glob

    runs_root = os.path.join(PROJ_ROOT, "rf_zynq", "yolo", "runs")
    patterns  = [
        os.path.join(runs_root, "*", "weights", "best.pt"),
        os.path.join(runs_root, "detect", "*", "weights", "best.pt"),
    ]
    matches = []
    for p in patterns:
        matches.extend(glob.glob(p))

    if not matches:
        print(f"[WARN] 未找到 YOLO 权重文件（{runs_root}），S2 视觉推理将被跳过。")
        print(f"       系统将以纯 RF 模式运行（S1 + S3），告警逻辑不受影响。")
        return None

    best_pt = sorted(matches, key=os.path.getmtime)[-1]
    print(f"[YOLO] 已加载 PyTorch 权重: {best_pt}")
    return YOLO(best_pt)


def active_yolo_inference(model, tensor_bgr: np.ndarray):
    """
    对输入频谱张量执行 YOLOv8 目标检测推理。

    兼容 ultralytics YOLO 和 RKNNLiteInfer 两种后端的统一接口。

    Parameters
    ----------
    model       : YOLO 或 RKNNLiteInfer 实例
    tensor_bgr  : 640×640×3 BGR 格式频谱瀑布图张量（uint8）

    Returns
    -------
    (bool, float, ndarray) : (检测标志, 最高置信度, 标注图像)

    v4.0 说明：
      bbox_score 现在被传递至 alert_info["yolo_score"]，
      供 system_hub 的 SDS 评分融合作为第五个正交证据维度。
      YOLO 单独不触发告警，仅在 S3 SDS 评分接近门限时提供补充分值。
    """
    try:
        results = model.predict(source=tensor_bgr, verbose=False)

        # 防护：aarch64 daemon 线程中 PyTorch 首次推理可能返回 None 或空列表
        if not results:
            return False, 0.0, tensor_bgr.copy()

        highest_score = 0.0
        for r in results:
            boxes = r.boxes
            if boxes is not None and len(boxes) > 0:
                confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, 'cpu') else boxes.conf.numpy()
                highest_score = float(np.max(confs))

        is_detected     = highest_score > cfg.YOLO_CONF_THRESH
        annotated_frame = results[0].plot()
        return is_detected, highest_score, annotated_frame

    except Exception as e:
        # 推理失败时返回原始张量，不阻断主循环
        print(f"[WARN] YOLO inference error (suppressed): {e}")
        return False, 0.0, tensor_bgr.copy()


class RFToolchain:
    """
    三级射频检测流水线主控类（S1-RSSI → S2-YOLO → S3-CycloAudit）

    tick() 方法执行一次完整的检测周期：
      1. S1 快速 RSSI 预扫，获取各扇区信号功率排名
      2. 将 SDR 调谐至功率最强扇区
      3. S2 采集 IQ 数据、生成频谱瀑布图，YOLO 推理（辅助显示）
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
                  f"Backend={'RKNN-NPU' if cfg.YOLO_BACKEND == 'rknn' else 'PyTorch-CPU'})")
        except Exception as e:
            print(f"[ERROR] RFToolchain: SDR 初始化失败: {e}")
            raise

        self.brain_yolo    = load_yolo_model()
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
            · yolo_score  : S2 YOLO 最高置信度（system_hub SDS 融合用）
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

        yolo_flag, bbox_score, annotated_frame = False, 0.0, waterfall_tensor.copy()
        if self.brain_yolo is not None:
            yolo_flag, bbox_score, annotated_frame = active_yolo_inference(
                self.brain_yolo, waterfall_tensor
            )
            backend_tag = 'RKNN-NPU' if cfg.YOLO_BACKEND == 'rknn' else 'CPU'
        else:
            backend_tag = 'DISABLED'
        cost_s2 = time.time() - t0

        cv2.putText(annotated_frame, f"SECTOR: {active_freq/1e6:.0f} MHz",
                    (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 2)

        # v4.0: YOLO 评分标注到频谱图（绿=命中，灰=未达阈值）
        if self.brain_yolo is not None:
            yolo_color = (0, 255, 0) if yolo_flag else (120, 120, 120)
            cv2.putText(annotated_frame,
                        f"YOLO: {'HIT' if yolo_flag else 'NEG'} ({bbox_score:.2f})",
                        (10, 620), cv2.FONT_HERSHEY_SIMPLEX, 0.7, yolo_color, 2)

        log_lines.append(
            f"[S2] Freq={active_freq/1e6:.0f} MHz | "
            f"Cost={cost_s2:.3f} s | "
            f"YOLO={'SKIP' if self.brain_yolo is None else f'{yolo_flag}({bbox_score:.3f})'} "
            f"[{backend_tag}]"
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
                f"NCC={audit_score:.4f}  YOLO={bbox_score:.3f}  → 提交 TPF 确认"
            )
            alert_flag = True
            alert_info = {
                "freq_mhz":   active_freq / 1e6,
                "score":      audit_score,
                "yolo_score": bbox_score,          # v4.0 新增：YOLO 得分供 TPF/SDS 融合
                "sds_detail": sds_detail,           # v4.0 新增：SDS 分项诊断
            }
            cv2.putText(annotated_frame, f"S3 PASS  NCC={audit_score:.3f}",
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

        else:
            # S3 未触发但 YOLO 命中：记录辅助信息（供 system_hub 统计）
            if yolo_flag:
                alert_info = {
                    "freq_mhz":   active_freq / 1e6,
                    "score":      audit_score,       # S3 NCC（未超阈值）
                    "yolo_score": bbox_score,        # YOLO 命中作为辅助证据
                    "yolo_only":  True,              # 标记：S3 未确认，仅 YOLO
                    "sds_detail": sds_detail,
                }

        return annotated_frame, "\n".join(log_lines), alert_flag, alert_info

