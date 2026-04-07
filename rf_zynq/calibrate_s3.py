# -*- coding: utf-8 -*-
"""
calibrate_s3.py -- S3 CAF-FFT Auto Background Calibration  v2.0
================================================================
统计方法升级：单点采样 + max/avg → 全量分块 + 百分位估计

核心改进（相较 v1.x）：
  1. 全量 chunk 提取（Full-Buffer Chunking）
       每个 65ms IQ buffer 被切分为所有合法 200k 块（约 13 块/buffer）
       16 buffers × 13 chunks = 208 NCC 样本/扇区
       相比 v1.x 的 8 样本，统计精度约提升 5 倍

  2. 百分位阈值推导（Percentile-Based Threshold Derivation）
       去掉对 buf.max 的依赖（单点极端值主导）
       改用 P95（第 95 百分位）+ P99（第 99 百分位）的加权组合：
         bg_eff = P99_WEIGHT * p99 + (1 - P99_WEIGHT) * p95
       理论依据：
         - 纯高斯噪声 NCC 的 CDF 近似 Rayleigh 分布
         - E[NCC] ≈ σ√(π/2)，其中 σ = 1/√N = 1/√200000 ≈ 0.224%
         - p99 ≈ 3.03σ ≈ 0.68%（纯噪声）
         - SMPS 谐波存在时 p99 通常升至 1.5~4%
         - p99 × NOISE_MARGIN 形成的阈值对 OcuSync（NCC ≈ 12~25%）
           留出 > 3× 余量，同时自适应跟踪环境底噪

  3. 环境质量评估（Calibration Quality Assessment）
       计算变异系数 CV = σ/μ（coefficient of variation）
       CV 过高说明环境存在间歇性强干扰或 SMPS 谐波
       如果 p99 > 5% 发出重度干扰告警

  4. PSR 背景参考测量（Background PSR Profiling）
       对最强干扰帧额外计算 τ 域峰值旁瓣比（PSR）
       输出环境 PSR 参考值，帮助用户确定 PSR_THRESHOLD 是否合适

  5. LO 预热稳定（LO Warm-up）
       调谐后等待 500ms + 额外 5 次 buffer flush，确保 PLL 完全锁定
       避免因 LO 瞬态抖动污染前几个 buffer 的 NCC 读数

校准产物：
  rf_zynq/s3_thresholds.json：按扇区保存 th_30k, th_15k（向下兼容）
  database/alert_images/s3_calibration_<ts>.png：可视化报告

Phases:
  Phase 1 -- 背景底噪基线测量（UAV 必须关机）
  Phase 2 -- 阈值推导 + 写入 JSON
"""

import sys
import os
import time
import json
import numpy as np
from datetime import datetime

# -- Project root (this file lives in rf_zynq/, root is two levels up) --------
_PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJ_ROOT not in sys.path:
    sys.path.insert(0, _PROJ_ROOT)

S3_SOURCE = os.path.join(_PROJ_ROOT, "rf_zynq", "rf_stage3_cyclostationary.py")
OUT_DIR   = os.path.join(_PROJ_ROOT, "database", "alert_images")
os.makedirs(OUT_DIR, exist_ok=True)

# -- SDR parameters (read from config.py -- same values used at runtime) ------
from backend_rk3588.config import SDR_URI, SAMPLE_RATE, SDR_GAIN_DB
RX_GAIN     = SDR_GAIN_DB
BUFFER_SIZE = 2_621_440   # 65 ms @ 40 MSps
SECTORS_HZ  = [5745e6, 5785e6, 5825e6]
N_CAPTURES  = 16          # IQ buffer 数（每扇区）；v1.x 为 8

# -- CAF scan parameters (identical to RF_Stage3_CycloAudit) ------------------
CHUNK_SIZE       = 200_000
TAU_30K, TAU_15K = 1333, 2667
ALPHA_SCAN_30K   = (22_000.0, 30_000.0)
ALPHA_SCAN_15K   = (10_500.0, 14_500.0)
MIN_POWER_GATE   = 1e-5

# -- Threshold derivation parameters (v2.0) ------------------------------------
#
# 新公式：bg_eff = P99_WEIGHT * p99 + (1-P99_WEIGHT) * p95
#         th     = max(HARD_FLOOR, bg_eff * NOISE_MARGIN)
#
# 从 v1.x 的 max/avg 切换到 p99/p95 的原因：
#   max  受单点 SMPS 瞬态主导，N=8 时方差极大（±50%）
#   p99  在 N=200+ 时估计误差 < ±10%，且剔除尾部极端值
#
# NOISE_MARGIN = 2.5（从 2.0 略微上调）：
#   在百分位统计更精确的前提下，适度上调裕量补偿残余不确定性
#   对 OcuSync 信号（15~25%）：分离比 = 18% / (4%×2.5) = 1.8× → 仍有充裕检测裕度
#
P99_WEIGHT     = 0.70    # p99 权重（抗极端值优先）
NOISE_MARGIN   = 2.5     # 阈值相对于 bg_eff 的倍数（v1.x 为 2.0）
HARD_FLOOR_30K = 0.018   # 1.8%（8× 理论噪声底  1/√200000 = 0.224%）
HARD_FLOOR_15K = 0.014   # 1.4%（6× 理论噪声底）

# 环境干扰告警阈值
CV_WARN_THRESHOLD  = 1.5   # 变异系数 > 1.5 → 高变异环境警告
P99_HEAVY_INTERF   = 0.05  # p99 > 5% → 重度干扰警告


# =============================================================================
# Core CAF-FFT metric (identical algorithm to RF_Stage3_CycloAudit)
# =============================================================================
def _caf_ncc_peak(chunk_raw, tau, alpha_range):
    """
    单帧 CAF-FFT 归一化 NCC 峰值。

    R_x^alpha(tau) 经由时延乘积 z[n]=x[n]·conj(x[n-tau]) 的 FFT 得到。
    NCC[alpha] = |Z[k]| / (N_z · P_x)

    Returns
    -------
    (peak_ncc, best_alpha_hz)
    """
    x = chunk_raw.astype(np.complex64) / 32768.0
    x -= x.mean()
    power = float(np.mean(np.abs(x) ** 2))
    if power < MIN_POWER_GATE:
        return 0.0, alpha_range[0]

    z   = x[tau:] * np.conj(x[:-tau])
    N_z = len(z)
    Z   = np.fft.fft(z)
    ncc = np.abs(Z) / (N_z * (power + 1e-12))

    f_res = SAMPLE_RATE / N_z
    k_lo  = max(1,      int(np.round(alpha_range[0] / f_res)))
    k_hi  = min(N_z//2, int(np.round(alpha_range[1] / f_res)) + 1)

    if k_lo >= k_hi:
        return 0.0, alpha_range[0]

    seg      = ncc[k_lo:k_hi]
    best_idx = int(np.argmax(seg))
    return float(seg[best_idx]), float((k_lo + best_idx) * f_res)


def _compute_background_psr(chunk_raw, tau, alpha_best_hz,
                             n_side=10, half_w=200, delta_guard=25):
    """
    计算单帧底噪 PSR（峰值旁瓣比），用于评估 PSR_THRESHOLD 设置是否合适。

    PSR = NCC(alpha_best, tau) / median{ NCC(alpha_best, tau_sidelobe) }

    环境底噪的 PSR 代表了在"无无人机"状态下门限能达到的最高误判水准，
    若底噪 PSR 接近或超过 PSR_THRESHOLD，说明阈值设置不够保守。

    Returns
    -------
    float : PSR 值（< 1 时返回 1.0）
    """
    x = chunk_raw.astype(np.complex64) / 32768.0
    x -= x.mean()
    pwr = float(np.mean(np.abs(x) ** 2))
    if pwr < MIN_POWER_GATE:
        return 1.0

    def single_caf(t):
        if len(x) <= t:
            return 0.0
        z = x[t:] * np.conj(x[:-t])
        pw = float(np.mean(np.abs(x[t:]) ** 2))
        if pw < MIN_POWER_GATE:
            return 0.0
        n = np.arange(len(z), dtype=np.float32)
        demod = np.exp(-1j * 2.0 * np.pi * alpha_best_hz / SAMPLE_RATE * n)
        return float(np.abs(np.mean(z * demod))) / (pw + 1e-12)

    peak = single_caf(tau)

    lo = max(delta_guard + 20, tau - half_w)
    hi = min(len(x) // 3,     tau + half_w)
    cands = [
        int(t) for t in np.linspace(lo, hi, n_side * 3)
        if abs(int(t) - tau) > delta_guard
    ][:n_side]

    if not cands:
        return 1.0
    sidelobes = [single_caf(t) for t in cands]
    return peak / (float(np.median(sidelobes)) + 1e-12)


# =============================================================================
# SDR capture
# =============================================================================
def _init_sdr(freq_hz):
    """
    初始化 SDR 并调谐至指定频率，含 LO 预热稳定流程。

    改进（v2.0）：
      - flush buffer 数量从 3 增至 5
      - 调谐后等待 500ms（v1.x 无等待），使 AD9364 PLL 完全锁定
    """
    try:
        import adi
        sdr = adi.Pluto(SDR_URI)
        sdr.sample_rate                  = SAMPLE_RATE
        sdr.rx_rf_bandwidth              = SAMPLE_RATE
        sdr.rx_hardwaregain_control_mode = 'manual'
        sdr.rx_hardwaregain_chan0        = RX_GAIN
        sdr.rx_buffer_size               = BUFFER_SIZE
        sdr.rx_lo                        = int(freq_hz)

        # LO 预热稳定：等待 PLL 锁定
        time.sleep(0.5)
        for _ in range(5):      # 5 次 flush（v1.x 为 3 次）
            sdr.rx()
        time.sleep(0.1)         # 额外静置
        return sdr
    except Exception as e:
        print(f"  [!] SDR init failed: {e}")
        return None


def _capture_buffers(freq_hz, n):
    """Capture n IQ buffers at freq_hz. Returns empty list on failure."""
    sdr = _init_sdr(freq_hz)
    if sdr is None:
        return []
    bufs = []
    for i in range(n):
        bufs.append(sdr.rx())
        print(f"    [{freq_hz/1e6:.0f}MHz] captured {i+1}/{n}")
    return bufs


def _extract_all_chunks(buf):
    """
    从单个 IQ buffer 中提取所有合法非重叠 CHUNK_SIZE 块。

    v1.x 仅取 buf 中段单点，约 1 chunk / buffer。
    v2.0 取全量，每个 65ms buffer 可提供约 13 个 chunk。

    Returns
    -------
    list of np.ndarray : 每个元素为长度 CHUNK_SIZE 的原始 int16/complex IQ 块
    """
    chunks = []
    n = len(buf)
    i = 0
    while i + CHUNK_SIZE <= n:
        chunks.append(buf[i: i + CHUNK_SIZE])
        i += CHUNK_SIZE
    return chunks


# =============================================================================
# Phase 1: Background noise baseline
# =============================================================================
def phase1_background():
    """
    测量各扇区 CAF-NCC 环境底噪分布（UAV 必须关机）。

    统计输出（v2.0 新增）：
      n_chunks : 本次有效参与统计的 chunk 总数
      p50/p95/p99 : NCC 的第 50/95/99 百分位
      mean/std    : NCC 均值与标准差
      cv          : 变异系数（= std/mean），衡量底噪稳定性
      max_psr     : 底噪最强帧的 PSR（参考 PSR_THRESHOLD 设置）

    Returns
    -------
    dict : {freq_hz: {各统计量}}
    """
    print("\n" + "=" * 64)
    print("  Phase 1 -- Background Noise Baseline  (UAV **MUST** be OFF)")
    print("=" * 64)
    print(f"  采集配置: {N_CAPTURES} buffers/sector × "
          f"~{BUFFER_SIZE // CHUNK_SIZE} chunks/buffer "
          f"≈ {N_CAPTURES * (BUFFER_SIZE // CHUNK_SIZE)} chunks/sector")

    results = {}
    for freq in SECTORS_HZ:
        print(f"\n  [Sector {freq/1e6:.0f} MHz]")
        bufs = _capture_buffers(freq, N_CAPTURES)

        if not bufs:
            print(f"  SDR 离线 -- 扇区 {freq/1e6:.0f}MHz 跳过（使用默认值）")
            results[freq] = {
                'n_chunks'    : 0,
                'ncc_30k_p50' : 0.010, 'ncc_30k_p95' : 0.020,
                'ncc_30k_p99' : 0.025, 'ncc_30k_max' : 0.030,
                'ncc_30k_avg' : 0.012, 'ncc_30k_std' : 0.004,
                'ncc_15k_p50' : 0.008, 'ncc_15k_p95' : 0.018,
                'ncc_15k_p99' : 0.022, 'ncc_15k_max' : 0.028,
                'ncc_15k_avg' : 0.010, 'ncc_15k_std' : 0.003,
                'bg_psr_30k'  : 1.0,
            }
            continue

        ncc30_list, ncc15_list   = [], []
        alpha30_list, alpha15_list = [], []

        # ── 全量 chunk 提取（核心改进）────────────────────────────────────
        for buf_idx, buf in enumerate(bufs):
            chunks = _extract_all_chunks(buf)
            for chunk in chunks:
                n30, a30 = _caf_ncc_peak(chunk, TAU_30K, ALPHA_SCAN_30K)
                n15, a15 = _caf_ncc_peak(chunk, TAU_15K, ALPHA_SCAN_15K)
                ncc30_list.append(n30)
                ncc15_list.append(n15)
                alpha30_list.append(a30)
                alpha15_list.append(a15)

            # 进度每 4 个 buffer 打印一次
            if (buf_idx + 1) % 4 == 0:
                print(f"    [{freq/1e6:.0f}MHz] 已提取 "
                      f"{len(ncc30_list)} chunks "
                      f"(from {buf_idx+1}/{N_CAPTURES} buffers)")

        arr30 = np.array(ncc30_list, dtype=np.float32)
        arr15 = np.array(ncc15_list, dtype=np.float32)

        # ── PSR 背景参考测量（取 30kHz NCC 最强帧）───────────────────────
        peak_idx_30 = int(np.argmax(arr30))
        alpha_best  = float(alpha30_list[peak_idx_30])
        # 找到对应 buffer 和 chunk（用于 PSR 计算）
        chunks_per_buf = BUFFER_SIZE // CHUNK_SIZE
        buf_i  = peak_idx_30 // chunks_per_buf
        chk_i  = peak_idx_30 %  chunks_per_buf
        if buf_i < len(bufs):
            chunk_for_psr = _extract_all_chunks(bufs[buf_i])
            if chk_i < len(chunk_for_psr):
                bg_psr = _compute_background_psr(
                    chunk_for_psr[chk_i], TAU_30K, alpha_best)
            else:
                bg_psr = 1.0
        else:
            bg_psr = 1.0

        r = {
            'n_chunks'    : len(arr30),
            # OcuSync 30kHz 通道统计
            'ncc_30k_p50' : float(np.percentile(arr30, 50)),
            'ncc_30k_p95' : float(np.percentile(arr30, 95)),
            'ncc_30k_p99' : float(np.percentile(arr30, 99)),
            'ncc_30k_max' : float(arr30.max()),
            'ncc_30k_avg' : float(arr30.mean()),
            'ncc_30k_std' : float(arr30.std()),
            # OcuSync 15kHz 通道统计
            'ncc_15k_p50' : float(np.percentile(arr15, 50)),
            'ncc_15k_p95' : float(np.percentile(arr15, 95)),
            'ncc_15k_p99' : float(np.percentile(arr15, 99)),
            'ncc_15k_max' : float(arr15.max()),
            'ncc_15k_avg' : float(arr15.mean()),
            'ncc_15k_std' : float(arr15.std()),
            # PSR 参考值
            'bg_psr_30k'  : bg_psr,
        }
        results[freq] = r

        # ── 统计摘要打印 ────────────────────────────────────────────────
        cv30 = r['ncc_30k_std'] / (r['ncc_30k_avg'] + 1e-12)
        cv15 = r['ncc_15k_std'] / (r['ncc_15k_avg'] + 1e-12)

        print(f"\n    ┌─ OcuSync 30kHz 频道 (τ={TAU_30K}) ─────────────────────┐")
        print(f"    │  n={r['n_chunks']} chunks  "
              f"avg={r['ncc_30k_avg']*100:.3f}%  "
              f"std={r['ncc_30k_std']*100:.3f}%  CV={cv30:.2f}")
        print(f"    │  p50={r['ncc_30k_p50']*100:.3f}%  "
              f"p95={r['ncc_30k_p95']*100:.3f}%  "
              f"p99={r['ncc_30k_p99']*100:.3f}%  "
              f"max={r['ncc_30k_max']*100:.3f}%")
        print(f"    │  环境 PSR（最强帧）= {bg_psr:.2f}×")
        print(f"    └─ OcuSync 15kHz 频道 (τ={TAU_15K}) ─────────────────────┘")
        print(f"       avg={r['ncc_15k_avg']*100:.3f}%  "
              f"p95={r['ncc_15k_p95']*100:.3f}%  "
              f"p99={r['ncc_15k_p99']*100:.3f}%")

        # ── 环境干扰质量告警 ────────────────────────────────────────────
        warnings = []
        if cv30 > CV_WARN_THRESHOLD:
            warnings.append(
                f"  [WARN] 30kHz 通道 CV={cv30:.2f} > {CV_WARN_THRESHOLD} "
                f"→ 背景波动剧烈，疑似 SMPS 谐波间歇干扰")
        if r['ncc_30k_p99'] > P99_HEAVY_INTERF:
            warnings.append(
                f"  [WARN] 30kHz p99={r['ncc_30k_p99']*100:.1f}% > 5% "
                f"→ 重度环境干扰，建议检查 SDR 近端干扰源")
        if bg_psr > 3.0:
            warnings.append(
                f"  [WARN] 底噪 PSR={bg_psr:.2f}× > 3.0 "
                f"→ 环境噪声本身 τ 域峰尖锐，"
                f"建议将 PSR_THRESHOLD 提高至 {bg_psr * 1.3:.1f}×")
        for w in warnings:
            print(w)

    return results


# =============================================================================
# Phase 2: Threshold derivation
# =============================================================================
def _derive_thresholds(bg_results):
    """
    基于百分位统计推导各扇区最优检测阈值（v2.0 升级版）。

    推导公式（改自 v1.x 的 0.4×max + 0.6×avg）：
    ─────────────────────────────────────────────────────────────
      bg_eff = P99_WEIGHT × p99 + (1 - P99_WEIGHT) × p95

      其中 P99_WEIGHT = 0.70（p99 权重），1 - 0.70 = 0.30（p95 权重）

    物理含义：
      p99 覆盖 99% 的底噪分布；p95 提供平滑作用，防止 p99 本身的估计偏差
      （N=200 时 p99 的置信区间约为 ±10%，加权 p95 后可将等效偏差降至 ±6%）

    阈值公式：
      th = max(HARD_FLOOR, bg_eff × NOISE_MARGIN)

    典型数值示例（清洁环境，SMPS 干扰弱）：
      p99 ≈ 0.8%，p95 ≈ 0.6%
      bg_eff = 0.70×0.8% + 0.30×0.6% = 0.74%
      th_30k = max(1.8%, 0.74%×2.5) = max(1.8%, 1.85%) ≈ 1.85%  → HARD_FLOOR 生效

    典型数值示例（中度干扰环境，SMPS 谐波明显）：
      p99 ≈ 2.5%，p95 ≈ 1.8%
      bg_eff = 0.70×2.5% + 0.30×1.8% = 2.29%
      th_30k = max(1.8%, 2.29%×2.5) = 5.73%  → 自适应阈值生效

    Returns
    -------
    dict: {freq_hz: {'th_30k': float, 'th_15k': float}}
    """
    per_sector = {}
    for freq in SECTORS_HZ:
        bg = bg_results.get(freq, {})

        p99_30 = bg.get('ncc_30k_p99', HARD_FLOOR_30K / NOISE_MARGIN)
        p95_30 = bg.get('ncc_30k_p95', p99_30 * 0.8)
        p99_15 = bg.get('ncc_15k_p99', HARD_FLOOR_15K / NOISE_MARGIN)
        p95_15 = bg.get('ncc_15k_p95', p99_15 * 0.8)

        # 加权背景有效值
        # bg_eff = P99_WEIGHT * p99 + (1 - P99_WEIGHT) * p95
        bg_eff_30 = P99_WEIGHT * p99_30 + (1.0 - P99_WEIGHT) * p95_30
        bg_eff_15 = P99_WEIGHT * p99_15 + (1.0 - P99_WEIGHT) * p95_15

        per_sector[freq] = {
            'th_30k': max(HARD_FLOOR_30K, bg_eff_30 * NOISE_MARGIN),
            'th_15k': max(HARD_FLOOR_15K, bg_eff_15 * NOISE_MARGIN),
            # 附加诊断字段（写入 JSON 供离线分析）
            'bg_eff_30': round(bg_eff_30, 6),
            'bg_eff_15': round(bg_eff_15, 6),
            'bg_psr_30k': round(bg.get('bg_psr_30k', 1.0), 3),
            'n_chunks'  : bg.get('n_chunks', 0),
        }
    return per_sector


# =============================================================================
# Phase 2: Write JSON
# =============================================================================
THRESHOLD_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "s3_thresholds.json")


def phase2_apply(per_sector_th):
    """
    将各扇区校准阈值及诊断信息持久化至 s3_thresholds.json。

    JSON schema（v2.0，向下兼容 v1.x 的 th_30k / th_15k 键）：
    {
      "calibration_version": "2.0",
      "noise_margin": 2.5,
      "sectors": {
        "5745000000": {
          "th_30k"     : 0.050,
          "th_15k"     : 0.030,
          "bg_eff_30"  : 0.020,   # 参考：推导阈值所用的 bg_eff
          "bg_eff_15"  : 0.015,
          "bg_psr_30k" : 1.85,    # 参考：底噪最强帧 PSR
          "n_chunks"   : 208      # 参考：参与统计的 chunk 数
        },
        ...
      },
      "calibrated_at": "2026-04-07T20:45:00"
    }
    """
    print("\n" + "=" * 64)
    print("  Phase 2 -- 写入扇区阈值 → s3_thresholds.json")
    print(f"  公式: bg_eff = {P99_WEIGHT}×p99 + {1-P99_WEIGHT}×p95")
    print(f"         th = max(HARD_FLOOR, bg_eff × {NOISE_MARGIN})")
    print("-" * 64)
    for freq, th in per_sector_th.items():
        print(f"  {freq/1e6:.0f} MHz :")
        print(f"    bg_eff_30={th['bg_eff_30']*100:.3f}%  "
              f"→ TH_30k={th['th_30k']*100:.3f}%  "
              f"[分离比 ≈ {0.18 / th['th_30k']:.1f}× vs OcuSync@18%]")
        print(f"    bg_eff_15={th['bg_eff_15']*100:.3f}%  "
              f"→ TH_15k={th['th_15k']*100:.3f}%")
        print(f"    底噪 PSR 参考 = {th['bg_psr_30k']:.2f}×  "
              f"（n_chunks={th['n_chunks']}）")
    print(f"  File : {THRESHOLD_JSON}")
    print("=" * 64)

    payload = {
        "calibration_version": "2.0",
        "noise_margin"       : NOISE_MARGIN,
        "p99_weight"         : P99_WEIGHT,
        "sectors": {
            str(int(freq)): {
                "th_30k"     : round(th["th_30k"],      6),
                "th_15k"     : round(th["th_15k"],      6),
                "bg_eff_30"  : round(th["bg_eff_30"],   6),
                "bg_eff_15"  : round(th["bg_eff_15"],   6),
                "bg_psr_30k" : round(th.get("bg_psr_30k", 1.0), 3),
                "n_chunks"   : th.get("n_chunks", 0),
            }
            for freq, th in per_sector_th.items()
        },
        "calibrated_at": datetime.now().isoformat(timespec='seconds'),
    }
    with open(THRESHOLD_JSON, 'w') as f:
        json.dump(payload, f, indent=2)
    print(f"  OK -- thresholds saved.")


# =============================================================================
# Calibration report plot (v2.0: 包含 p95/p99 标柱)
# =============================================================================
def _save_report(bg_results, per_sector_th):
    """
    生成可视化校准报告（每扇区一栏）。

    v2.0 新增：
      - 以箱线图展示 NCC 分布的 p50/p95/p99/max
      - 标注环境 PSR 参考值
      - 在图标题显示校准版本和统计样本数
    """
    try:
        import matplotlib
        matplotlib.use('Agg')
        matplotlib.rcParams['font.family'] = ['DejaVu Sans']
        matplotlib.rcParams['axes.unicode_minus'] = False
        import matplotlib.pyplot as plt

        n    = len(SECTORS_HZ)
        fig, axes = plt.subplots(1, n, figsize=(7 * n, 6), sharey=False)
        if n == 1:
            axes = [axes]

        for ax, freq in zip(axes, SECTORS_HZ):
            bg = bg_results.get(freq, {})
            th = per_sector_th.get(freq, {})
            th_30k = th.get('th_30k', HARD_FLOOR_30K)
            th_15k = th.get('th_15k', HARD_FLOOR_15K)
            n_chunks = bg.get('n_chunks', 0)

            # ── 30kHz 通道百分位柱状图 ─────────────────────────────────
            labels = ['p50\n30k', 'p95\n30k', 'p99\n30k', 'max\n30k',
                      'p50\n15k', 'p95\n15k', 'p99\n15k', 'max\n15k']
            values = [
                bg.get('ncc_30k_p50', 0) * 100,
                bg.get('ncc_30k_p95', 0) * 100,
                bg.get('ncc_30k_p99', 0) * 100,
                bg.get('ncc_30k_max', 0) * 100,
                bg.get('ncc_15k_p50', 0) * 100,
                bg.get('ncc_15k_p95', 0) * 100,
                bg.get('ncc_15k_p99', 0) * 100,
                bg.get('ncc_15k_max', 0) * 100,
            ]
            colors = [
                '#78909C', '#FF7043', '#EF5350', '#B71C1C',
                '#80CBC4', '#26A69A', '#00838F', '#006064',
            ]
            bars = ax.bar(labels, values, color=colors, alpha=0.85, width=0.6)

            # 阈值线
            ax.axhline(th_30k * 100, color='#1565C0', linestyle='--',
                       linewidth=1.8, label=f'TH_30k={th_30k*100:.2f}%')
            ax.axhline(th_15k * 100, color='#2E7D32', linestyle='--',
                       linewidth=1.8, label=f'TH_15k={th_15k*100:.2f}%')
            ax.axhline(HARD_FLOOR_30K * 100, color='gray', linestyle=':',
                       linewidth=1.0, label=f'HardFloor={HARD_FLOOR_30K*100:.1f}%')

            # 数值标注
            for bar, val in zip(bars, values):
                if val > 0:
                    ax.text(bar.get_x() + bar.get_width() / 2,
                            val + max(values) * 0.02,
                            f'{val:.3f}%',
                            ha='center', va='bottom', fontsize=7.5)

            # bg_eff 参考标注
            bg_eff_30 = th.get('bg_eff_30', 0)
            bg_psr    = bg.get('bg_psr_30k', 1.0)
            ax.set_title(
                f"{freq/1e6:.0f} MHz Sector\n"
                f"bg_eff_30={bg_eff_30*100:.3f}%  "
                f"PSR_bg={bg_psr:.2f}×  "
                f"n={n_chunks}",
                fontsize=10
            )
            ax.set_ylabel('CAF-NCC (%)')
            ax.legend(fontsize=8)
            ax.grid(alpha=0.3, axis='y')
            ax.tick_params(axis='x', labelsize=8)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fig.suptitle(
            f'S3 CAF-FFT Calibration Report v2.0\n'
            f'NOISE_MARGIN={NOISE_MARGIN}×  '
            f'bg_eff = {P99_WEIGHT}×p99 + {1-P99_WEIGHT}×p95',
            fontsize=12, fontweight='bold'
        )
        plt.tight_layout()
        path = os.path.join(OUT_DIR, f's3_calibration_{ts}.png')
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"  Report saved: {path}")
    except Exception as e:
        print(f"  [!] Report plot failed (non-critical): {e}")


# =============================================================================
# Main
# =============================================================================
def main():
    print()
    print("=" * 66)
    print("  RF-Vision S3 CAF-FFT Auto Calibration  v2.0")
    print("  全量分块采样 + 百分位阈值推导 + 环境质量评估")
    print("=" * 66)
    print(f"  SDR      : {SDR_URI}")
    print(f"  Gain     : {RX_GAIN} dB")
    print(f"  Fs       : {SAMPLE_RATE/1e6:.0f} MSps")
    print(f"  Sectors  : {[int(f/1e6) for f in SECTORS_HZ]} MHz")
    print(f"  Buffers  : {N_CAPTURES} × {BUFFER_SIZE/1e6:.2f}M samples = "
          f"{N_CAPTURES * BUFFER_SIZE / SAMPLE_RATE * 1000:.0f} ms / sector")
    est_chunks = N_CAPTURES * (BUFFER_SIZE // CHUNK_SIZE)
    print(f"  统计样本量: ~{est_chunks} chunks/sector  "
          f"（v1.x 为 {N_CAPTURES} chunks，提升 {est_chunks // N_CAPTURES}×）")
    print()

    bg_results = phase1_background()
    per_sector_th = _derive_thresholds(bg_results)

    print("\n  +" + "-" * 58 + "+")
    print(f"  | 各扇区推导阈值  "
          f"NOISE_MARGIN={NOISE_MARGIN}×  "
          f"公式: p99×{P99_WEIGHT}+p95×{1-P99_WEIGHT}"
          f"{'':14}|")
    for freq, th in per_sector_th.items():
        line = (f"  |  {freq/1e6:.0f} MHz:  "
                f"TH_30k={th['th_30k']*100:6.3f}%   "
                f"TH_15k={th['th_15k']*100:6.3f}%   "
                f"bg_PSR={th.get('bg_psr_30k',1.0):.2f}×")
        print(line.ljust(60) + "|")
    print("  +" + "-" * 58 + "+")

    phase2_apply(per_sector_th)
    _save_report(bg_results, per_sector_th)
    print("\n  校准完成，新阈值已激活。\n")


if __name__ == '__main__':
    main()
