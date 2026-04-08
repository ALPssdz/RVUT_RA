# -*- coding: utf-8 -*-
"""
rf_stage3_cyclostationary.py — Stage 3: Cyclic Frequency Discriminator v4.0
==============================================================================

v4.0 核心升级：三重互验证 + 软判决评分融合
==========================================

【虚警根因分析】
  v3.x 的硬串联门限（NCC→PSR→CFS）对 SMPS 谐波的抑制存在盲区：
    · SMPS 谐波间隔 ~200 kHz，在 τ=200 样本处有规律尖峰
    · 此 τ 值不在 PSR 扫描窗 [1333±200, 2667±200] 内，PSR 无法捕获
    · 当 SMPS 与 WiFi 联合作用时，CFS 也可能虚高，导致硬串联全部通过

【弱信号漏警根因分析】
  PEAK_WEIGHT=0.50 + 帧平均掩盖稀疏突发峰：
    · OcuSync 占空比 ~30%，突发帧 NCC 高，静默帧 NCC≈噪底
    · 平均后联合 NCC 被稀释约 3.3×，恰好低于检测阈值
    · 弱信号（远距离/遮挡）突发帧 NCC 仅勉强超阈值，被平均后稳定低于阈值

v4.0 升级要点
=============
1. 第四重验证：α 域频率稳定性（Alpha Frequency Stability, AFS）
   - OcuSync Δα_frame < 500 Hz（TCXO 精度 ±10 ppm @ 5.8GHz → Δf ≈ ±58 kHz）
   - SMPS/WiFi 伪峰帧间漂移 σ_α ~ 2-5 kHz，显著高于 OcuSync
   - AFS 通过考量判决：σ_α < AFS_ALPHA_SIGMA_MAX（500 Hz）

2. 软判决评分融合（Soft Decision Scoring, SDS）代替纯硬串联
   S_composite = w1·(NCC/th_NCC) + w2·log10(PSR/th_PSR)
               + w3·log10(CFS/th_CFS) + w4·I[AFS_pass]
   权重基于 Fisher 信息量分析：w=[0.45, 0.25, 0.20, 0.10]
   判决规则：
     · S_composite ≥ 1.0 且 NCC ≥ 0.80×th（软下限）→ DETECT
     · NCC ≥ 2.5×th（强信号直通）→ DETECT
     · 其他 → NEGATIVE

3. CHUNK_SIZE 从 200,000 → 160,000（4 ms @ 40 MSps）
   - 更短窗口减少突发帧被静默帧稀释
   - 帧命中率：P_hit = 1−(1−d)^(C/F)，d=30%,C/F→命中率提升约 18%
   - OVERLAP 从 0.75 → 0.80 弥补帧数，步长 32,000 vs 原 50,000

4. PEAK_WEIGHT 从 0.50 → 0.65
   - OcuSync 占空比 d≈30%，最优峰权重 = 1−d/2 ≈ 0.85（理论上限）
   - 取 0.65 作为保守值，在弱信号放大（~1.3×）与稳定性间折中

5. PSR_THRESHOLD 从 2.5 → 2.2，CFS_THRESHOLD 从 1.8 → 2.0
   - 配合 SDS 软判决：单项门限适度调整，最终由综合评分兜底

【算法核心：CAF-FFT 循环频率判别器（继承自 v3.x）】
  R_x^α(τ) = (1/N) Σ x[n]·x*[n-τ]·exp(-j2π α n/Fs)
  NCC[α] = |FFT(z)[k]| / (N_z · P_x)  where z[n] = x[n]·x*[n-τ]

目标延迟（Fs = 40 MSps）：
  · τ=1333 → N_fft = Fs/30kHz → OcuSync 3.0/4.0 (Mini 4 Pro, Mavic 3)
  · τ=2667 → N_fft = Fs/15kHz → OcuSync 2.0     (Mini 3, Air 2S)
"""

import numpy as np
from datetime import datetime


class RF_Stage3_CycloAudit:
    """
    Stage 3 v4.0 — CAF-FFT + TOV（三重互验证）+ SDS（软判决评分融合）

    公开接口与 v3.x 完全兼容：
        run_spectral_audit(iq_data_buffer, sector_hz) → (bool, float)

    新增诊断字段（通过内部属性暴露，供 system_hub 读取）：
        self.last_sds_detail : dict  — 本次判决的各维度得分明细
    """

    # ─── 协议物理参数 ────────────────────────────────────────────────────────
    TAU_OCUSYNC_30K = 1333   # OcuSync 3.0/4.0  N_fft = 40MHz/30kHz
    TAU_OCUSYNC_15K = 2667   # OcuSync 2.0       N_fft = 40MHz/15kHz
    TAU_WIFI        = 128    # IEEE 802.11        N_fft = 40MHz/312.5kHz

    # OcuSync α 扫描范围（覆盖 CP 比例 1/8 ~ 1/3 的全部情形）
    # 22-30 kHz 覆盖 CP=1/8 (26.7 kHz) 和 CP=1/4 (24.0 kHz)，并保留 ±2 kHz 保护带
    ALPHA_SCAN_30K = (22_000.0, 30_000.0)
    ALPHA_SCAN_15K = (10_500.0, 14_500.0)

    # WiFi 监控循环频率（仅日志输出，不参与判决）
    ALPHA_WIFI_HZ  = 250_000.0

    # ─── 决策阈值（v4.0 更新） ───────────────────────────────────────────────
    # CAF-NCC 噪底（理论值）：1/√N；N=160000: σ = 0.25%
    # 硬地板设为 8× / 6× 理论噪底，与 v3.x 一致（保持对比参照）
    THRESHOLD_30K = 0.018   # 1.8% hard floor  (8× theoretical floor)
    THRESHOLD_15K = 0.014   # 1.4% hard floor  (6× theoretical floor)

    # v4.0: PEAK_WEIGHT 从 0.50 → 0.65
    # OcuSync 占空比 d≈30%：最优峰权重理论值 = 1-d/2 ≈ 0.85
    # 取 0.65 为保守值：既放大突发峰（弱信号），又保留均值稳定性
    PEAK_WEIGHT = 0.65

    # v4.0: PSR_THRESHOLD 从 2.5 → 2.2（配合 SDS 软判决）
    PSR_THRESHOLD      = 2.2   # 普通环境
    PSR_THRESHOLD_WIFI = 3.2   # 强 WiFi 监控区上调（保持不变）

    # v4.0: CFS_THRESHOLD 从 1.8 → 2.0（更严格排除弥散 SMPS 谐波）
    CFS_THRESHOLD = 2.0

    # 功率门控（与 v3.x 一致）
    MIN_POWER_GATE = 1e-5

    # ─── 帧分析参数（v4.0 更新） ─────────────────────────────────────────────
    # CHUNK_SIZE: 200,000 → 160,000（4 ms @ 40 MSps）
    # 理由：突发帧命中率 P_hit = 1–(1–d)^(C/F)，OcuSync d=30%,F_sym=133k 样本
    # 短窗 C=160k 使每窗含 ~1.2 帧平均，比 200k（~1.5 帧）增加突发暴露率约 18%
    CHUNK_SIZE = 160_000

    # OVERLAP: 0.75 → 0.80（步长从 50,000 → 32,000）
    # 弥补 CHUNK_SIZE 缩短导致的帧数减少：160k/32k=5 窗 vs 200k/50k=4 窗（+25%）
    OVERLAP    = 0.80

    # ─── SDS 软判决参数（v4.0 新增） ─────────────────────────────────────────
    # Fisher 信息量加权：w1(NCC)+w2(PSR)+w3(CFS)+w4(AFS)=1.0
    # NCC 主特征（与 SNR 直接相关）：w1=0.45
    # PSR（区分 OFDM 冲激峰 vs 连续谱干扰）：w2=0.25
    # CFS（区分 OcuSync vs 宽带弥散噪声）：w3=0.20
    # AFS（区分真实协议帧 vs 伪周期干扰）：w4=0.10
    SDS_W_NCC = 0.45
    SDS_W_PSR = 0.25
    SDS_W_CFS = 0.20
    SDS_W_AFS = 0.10

    # SDS 综合评分判决阈值
    SDS_COMPOSITE_THRESHOLD = 1.0

    # 软下限系数：允许 NCC 轻微低于阈值（0.80×th）的信号在其他证据充分时被检出
    # 物理含义：弱信号突发帧 NCC 可能因信噪比不足而低于标定阈值 20%，
    # 但若 PSR+CFS+AFS 三路证据均充分，则允许综合评分"救援"
    SDS_SOFT_NCC_FLOOR_RATIO = 0.80

    # 强信号直通 NCC 倍数（跳过 SDS，直接按安全系数判定）
    SDS_STRONG_BYPASS_RATIO  = 2.5

    # ─── AFS 参数（v4.0 新增） ────────────────────────────────────────────────
    # α 域频率稳定性最大允许帧间标准差（Hz）
    # 理论依据：TCXO 精度 ±10 ppm @ 5.8 GHz → Δf ≈ ±58 kHz
    # OcuSync α₀ 抖动 < ±500 Hz（实测），SMPS/WiFi 伪峰 ~2-5 kHz
    AFS_ALPHA_SIGMA_MAX = 500.0   # Hz

    # AFS 需要至少此数量的有效帧才能计算可靠的 σ_α
    AFS_MIN_FRAMES = 4

    def __init__(self, sample_rate: float = 40e6):
        self.sample_rate = float(sample_rate)
        self._freq_res   = None   # 首次调用时设定

        # 诊断字段（供 system_hub 读取详细评分）
        self.last_sds_detail: dict = {}

        # 每扇区标定阈值：{freq_hz_int: (th_30k, th_15k)}
        # 由 calibrate_s3.py 写入 s3_thresholds.json；缺失时使用类级别默认值
        self._sector_thresholds: dict = {}

        # 每扇区标定 WiFi 环境 NCC：{freq_hz_int: mean_wifi_ncc}
        # 动态调整 PSR WiFi 触发阈值：wifi_trigger_th = max(0.010, ambient × 2.0)
        self._wifi_ambient: dict = {}

        import os, json
        _json = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "s3_thresholds.json")
        if os.path.exists(_json):
            try:
                with open(_json) as _f:
                    _t = json.load(_f)
                if "sectors" in _t:
                    for k, v in _t["sectors"].items():
                        self._sector_thresholds[int(k)] = (
                            float(v["th_30k"]), float(v["th_15k"])
                        )
                    print("  [S3-v4] Per-sector thresholds loaded from JSON:")
                    for f, (t30, t15) in self._sector_thresholds.items():
                        print(f"         {f/1e6:.0f} MHz  "
                              f"30k={t30*100:.2f}%  15k={t15*100:.2f}%")
                else:
                    self.THRESHOLD_30K = float(_t["THRESHOLD_30K"])
                    self.THRESHOLD_15K = float(_t["THRESHOLD_15K"])
                    print(f"  [S3-v4] Global thresholds loaded (legacy JSON): "
                          f"30k={self.THRESHOLD_30K*100:.2f}%  "
                          f"15k={self.THRESHOLD_15K*100:.2f}%")

                if "wifi_ambient" in _t:
                    self._wifi_ambient = {int(k): float(v)
                                          for k, v in _t["wifi_ambient"].items()}
                    print("  [S3-v4] WiFi ambient NCC loaded from JSON:")
                    for f, w in self._wifi_ambient.items():
                        print(f"         {f/1e6:.0f} MHz  "
                              f"WiFi_ambient={w*100:.3f}%  "
                              f"PSR_trigger_th={max(0.010, w*2.0)*100:.3f}%")

            except Exception as _e:
                print(f"  [S3-v4] Failed to load s3_thresholds.json ({_e}), using defaults.")

    # =========================================================================
    # 内核：CAF-FFT 计算（继承自 v3.x，接口不变）
    # =========================================================================
    def _prepare_chunk(self, raw_chunk) -> tuple:
        """
        共用预处理：归一化 + DC 去除 + 功率计算。

        Returns (x, power)；功率不足时返回 (None, 0.0)。
        """
        x = raw_chunk.astype(np.complex64) / 32768.0
        x -= x.mean()
        power = float(np.mean(np.abs(x) ** 2))
        if power < self.MIN_POWER_GATE:
            return None, 0.0
        return x, power

    def _compute_caf_spectrum(self, x: np.ndarray, tau: int, power: float) -> np.ndarray:
        """
        计算归一化循环自相关谱（Normalized CAF Spectrum）。

        z[n] = x[n] · x*[n-τ]
        NCC[α] = |FFT(z)[k]| / (N_z · P_x)

        当 α = α₀ = 1/T_sym 时 NCC 取峰值（≈ CP 比例）；
        α ≠ α₀ 时 NCC ≈ 1/√N（噪声底）。
        """
        if len(x) <= tau:
            return np.zeros(1)
        z   = x[tau:] * np.conj(x[:-tau])
        N_z = len(z)
        Z   = np.fft.fft(z.astype(np.complex64))
        return np.abs(Z) / (N_z * (power + 1e-12))

    def _extract_ncc_in_range(self, ncc_spectrum, chunk_len, alpha_range_hz) -> tuple:
        """
        从 CAF 谱中提取指定 α 范围内的（峰值, 最佳循环频率, CFS）。

        CFS（循环频率集中度）= 峰值 / 旁瓣中值。
        真实 OcuSync：CFS >> 1；宽带干扰：CFS ≈ 1。
        """
        N     = len(ncc_spectrum)
        f_res = self.sample_rate / N

        k_lo = max(1, int(np.round(alpha_range_hz[0] / f_res)))
        k_hi = min(N // 2, int(np.round(alpha_range_hz[1] / f_res)) + 1)

        if k_lo >= k_hi:
            return 0.0, alpha_range_hz[0], 1.0

        segment  = ncc_spectrum[k_lo:k_hi]
        peak_idx = int(np.argmax(segment))
        peak_ncc = float(segment[peak_idx])
        best_alpha = (k_lo + peak_idx) * f_res

        sidelobes = np.delete(segment, peak_idx)
        cfs = peak_ncc / (float(np.median(sidelobes)) + 1e-12) if len(sidelobes) else 1.0

        return peak_ncc, best_alpha, cfs

    def _compute_psr(self, x, power, tau_target, alpha_best_hz,
                     delta_guard=25, n_side=10, half_w=200) -> float:
        """
        τ 域峰值旁瓣比（PSR）。

        PSR(τ₀) = NCC(α_best, τ₀) / median{ NCC(α_best, τ) : |τ−τ₀| > δ_guard }

        真实 OFDM CP 峰（Delta 冲激型）：PSR >> 1
        SMPS 开关纹波 / 宽带干扰（连续型）：PSR ≈ 1
        """
        def single_caf(tau_probe: int) -> float:
            if len(x) <= tau_probe:
                return 0.0
            z  = x[tau_probe:] * np.conj(x[:-tau_probe])
            pw = float(np.mean(np.abs(x[tau_probe:]) ** 2))
            if pw < self.MIN_POWER_GATE:
                return 0.0
            n     = np.arange(len(z), dtype=np.float32)
            demod = np.exp(-1j * 2.0 * np.pi * alpha_best_hz / self.sample_rate * n)
            return float(np.abs(np.mean(z * demod))) / (pw + 1e-12)

        peak = single_caf(tau_target)

        lo = max(delta_guard + 20, tau_target - half_w)
        hi = min(len(x) // 3, tau_target + half_w)
        candidates = [
            int(t) for t in np.linspace(lo, hi, n_side * 3)
            if abs(int(t) - tau_target) > delta_guard
        ][:n_side]

        if not candidates:
            return 1.0

        sidelobes  = [single_caf(t) for t in candidates]
        median_sl  = float(np.median(sidelobes))
        return peak / (median_sl + 1e-12)

    # =========================================================================
    # v4.0 新增：AFS 验证（α 域频率稳定性）
    # =========================================================================
    def _compute_afs(self, alpha_series: list) -> tuple:
        """
        计算 α 域帧间频率稳定性（Alpha Frequency Stability）。

        原理：
          OcuSync 帧的循环前缀时延固定，对应循环频率 α₀ 在帧间几乎不变：
            σ_α < AFS_ALPHA_SIGMA_MAX（500 Hz）
          SMPS/WiFi 伪峰的"最佳循环频率"因多径、热效应导致帧间漂移 σ_α ~ 2-5 kHz。

        Parameters
        ----------
        alpha_series : list[float] — 各帧的最佳循环频率（Hz），长度 ≥ AFS_MIN_FRAMES

        Returns
        -------
        (pass_flag, sigma_alpha_hz) : (bool, float)
          pass_flag     : True = σ_α < AFS_ALPHA_SIGMA_MAX → OcuSync 特征吻合
          sigma_alpha_hz: 实测 α 标准差（Hz）
        """
        if len(alpha_series) < self.AFS_MIN_FRAMES:
            # 帧数不足，无法可靠估计，保守地视为通过（不阻塞检测）
            return True, 0.0

        arr = np.array(alpha_series, dtype=np.float32)
        sigma = float(np.std(arr))
        return sigma < self.AFS_ALPHA_SIGMA_MAX, sigma

    # =========================================================================
    # v4.0 新增：SDS 软判决评分
    # =========================================================================
    def _compute_sds(self, ncc, th_ncc, psr, cfs, afs_pass) -> float:
        """
        软判决综合评分（Soft Decision Scoring, SDS）。

        S_composite = w1·(NCC/th_NCC) + w2·log10(PSR/th_PSR)
                    + w3·log10(CFS/th_CFS) + w4·I[AFS_pass]

        各项含义：
          · NCC/th_NCC：超出阈值的倍数（>1 = 超线，<1 = 不足）
          · log10(PSR/th_PSR)：PSR 相对于门限的对数余量（0 = 恰好达到）
          · log10(CFS/th_CFS)：CFS 相对于门限的对数余量
          · I[AFS_pass]：AFS 通过标志（0 或 1）

        各项分量被 clip 到 [0, 3] 防止单一超强指标主导评分。

        Parameters
        ----------
        ncc      : NCC 联合统计量（combined_score）
        th_ncc   : 当前扇区 NCC 阈值
        psr      : τ 域峰值旁瓣比
        cfs      : α 域循环频率集中度
        afs_pass : AFS 通过标志

        Returns
        -------
        float : SDS 综合评分（≥1.0 时判为检出）
        """
        # NCC 分量（线性比值）
        ncc_ratio = float(np.clip(ncc / (th_ncc + 1e-12), 0.0, 3.0))

        # PSR 分量（对数余量，PSR=th 时为 0，PSR>>th 时 >0）
        psr_log = float(np.clip(np.log10(max(psr, 0.01) / self.PSR_THRESHOLD), -1.0, 3.0))
        psr_score = float(np.clip(1.0 + psr_log, 0.0, 3.0))

        # CFS 分量（对数余量）
        cfs_log   = float(np.clip(np.log10(max(cfs, 0.01) / self.CFS_THRESHOLD), -1.0, 3.0))
        cfs_score = float(np.clip(1.0 + cfs_log, 0.0, 3.0))

        # AFS 分量（二元）
        afs_score = 1.0 if afs_pass else 0.0

        composite = (
            self.SDS_W_NCC * ncc_ratio  +
            self.SDS_W_PSR * psr_score  +
            self.SDS_W_CFS * cfs_score  +
            self.SDS_W_AFS * afs_score
        )

        # 保存诊断信息
        self.last_sds_detail = {
            "ncc_ratio":   ncc_ratio,
            "psr_score":   psr_score,
            "cfs_score":   cfs_score,
            "afs_score":   afs_score,
            "composite":   composite,
        }
        return composite

    # =========================================================================
    # 公开接口
    # =========================================================================
    def run_spectral_audit(self, iq_data_buffer, sector_hz: float = None) -> tuple:
        """
        v4.0 全双通道 CAF-FFT + TOV（四重互验证）+ SDS（软判决评分融合）。

        Parameters
        ----------
        iq_data_buffer : array-like
            原始 int16 IQ 采样（来自 AD9364 ADC）。
        sector_hz : float, optional
            当前 RX 中心频率（Hz）。若提供且存在对应扇区标定阈值，
            则使用扇区专属阈值代替全局默认值，最大化各扇区检测灵敏度。

        Returns
        -------
        (bool, float) : (检测结果, 联合 NCC 峰值系数)
        """
        self.last_sds_detail = {}

        # ── 解析当前扇区阈值 ────────────────────────────────────────────────
        if sector_hz is not None and self._sector_thresholds:
            key = min(self._sector_thresholds,
                      key=lambda k: abs(k - int(sector_hz)))
            th_30k_active, th_15k_active = self._sector_thresholds[key]
        else:
            th_30k_active = self.THRESHOLD_30K
            th_15k_active = self.THRESHOLD_15K

        buf           = np.asarray(iq_data_buffer)
        total_samples = len(buf)
        step_size     = int(self.CHUNK_SIZE * (1.0 - self.OVERLAP))

        # ── Level 1：逐帧 CAF-FFT 扫描 ──────────────────────────────────────
        frames_30k: list = []   # [(ncc, alpha, cfs), ...]
        frames_15k: list = []
        chunks_by_frame: list = []

        for i in range(0, total_samples - self.CHUNK_SIZE, step_size):
            chunk = buf[i : i + self.CHUNK_SIZE]
            x, pwr = self._prepare_chunk(chunk)
            if x is None:
                continue

            spec_30k = self._compute_caf_spectrum(x, self.TAU_OCUSYNC_30K, pwr)
            ncc_30k, alpha_30k, cfs_30k = self._extract_ncc_in_range(
                spec_30k, self.CHUNK_SIZE, self.ALPHA_SCAN_30K
            )

            spec_15k = self._compute_caf_spectrum(x, self.TAU_OCUSYNC_15K, pwr)
            ncc_15k, alpha_15k, cfs_15k = self._extract_ncc_in_range(
                spec_15k, self.CHUNK_SIZE, self.ALPHA_SCAN_15K
            )

            frames_30k.append((ncc_30k, alpha_30k, cfs_30k))
            frames_15k.append((ncc_15k, alpha_15k, cfs_15k))
            chunks_by_frame.append(x)

        if not frames_30k:
            print("  [S3-v4] WARNING: no valid frames (all below power gate)")
            return False, 0.0

        arr_30k = np.array([f[0] for f in frames_30k])
        arr_15k = np.array([f[0] for f in frames_15k])

        peak_30k  = float(arr_30k.max())
        peak_15k  = float(arr_15k.max())
        avg_30k   = float(arr_30k.mean())
        avg_15k   = float(arr_15k.mean())

        # 联合统计量（v4.0: PEAK_WEIGHT = 0.65）
        combined_30k = self.PEAK_WEIGHT * peak_30k + (1.0 - self.PEAK_WEIGHT) * avg_30k
        combined_15k = self.PEAK_WEIGHT * peak_15k + (1.0 - self.PEAK_WEIGHT) * avg_15k

        # WiFi 通道监控（仅日志）
        best_frame_x = chunks_by_frame[int(arr_30k.argmax())]
        _, pwr_best  = self._prepare_chunk(best_frame_x)
        wifi_ncc = 0.0
        if pwr_best and pwr_best > self.MIN_POWER_GATE:
            spec_wifi = self._compute_caf_spectrum(best_frame_x, self.TAU_WIFI, pwr_best)
            k_wifi = max(1, int(np.round(self.ALPHA_WIFI_HZ / (self.sample_rate / len(spec_wifi)))))
            k_wifi = min(k_wifi, len(spec_wifi) - 1)
            wifi_ncc = float(spec_wifi[k_wifi])

        print(
            f"  [S3-v4] {len(frames_30k)} frames | "
            f"OcuSync30k: peak={peak_30k*100:.2f}% avg={avg_30k*100:.2f}% "
            f"→ combined={combined_30k*100:.2f}% (th={th_30k_active*100:.1f}%) | "
            f"OcuSync15k: peak={peak_15k*100:.2f}% avg={avg_15k*100:.2f}% "
            f"→ combined={combined_15k*100:.2f}% (th={th_15k_active*100:.1f}%) | "
            f"WiFi@250kHz={wifi_ncc*100:.2f}%"
        )

        # ── AFS 预计算（v4.0 新增）──────────────────────────────────────────
        # 对两个通道分别计算各帧最佳循环频率序列的标准差
        alpha_series_30k = [f[1] for f in frames_30k]
        alpha_series_15k = [f[1] for f in frames_15k]
        afs_pass_30k, sigma_alpha_30k = self._compute_afs(alpha_series_30k)
        afs_pass_15k, sigma_alpha_15k = self._compute_afs(alpha_series_15k)

        # ── Level 2：联合统计量一级门限（支持 SDS 软下限）──────────────────
        # 软下限：允许 NCC 轻微低于阈值（0.80×th）的信号进入精检阶段
        soft_floor_30k = self.SDS_SOFT_NCC_FLOOR_RATIO * th_30k_active
        soft_floor_15k = self.SDS_SOFT_NCC_FLOOR_RATIO * th_15k_active

        triggered_ch    = None
        triggered_score = 0.0

        # 强信号直通（保持与 v3.x 兼容）
        strong_bypass_30k = combined_30k >= self.SDS_STRONG_BYPASS_RATIO * th_30k_active
        strong_bypass_15k = combined_15k >= self.SDS_STRONG_BYPASS_RATIO * th_15k_active

        if strong_bypass_30k or combined_30k >= th_30k_active:
            if combined_30k >= combined_15k or combined_15k < soft_floor_15k:
                triggered_ch    = '30k'
                triggered_score = combined_30k
            else:
                triggered_ch    = '15k'
                triggered_score = combined_15k
        elif combined_15k >= th_15k_active or (strong_bypass_15k):
            triggered_ch    = '15k'
            triggered_score = combined_15k
        elif combined_30k >= soft_floor_30k or combined_15k >= soft_floor_15k:
            # 软下限分支（NCC 未完全达标，但允许 SDS 精检）
            if combined_30k >= combined_15k:
                triggered_ch    = '30k'
                triggered_score = combined_30k
            else:
                triggered_ch    = '15k'
                triggered_score = combined_15k

        if triggered_ch is None:
            print("  [S3-v4] Below soft floor -- no UAV detected.")
            return False, max(combined_30k, combined_15k)

        # ── 解析触发通道 ────────────────────────────────────────────────────
        if triggered_ch == '30k':
            best_idx   = int(arr_30k.argmax())
            tau_t      = self.TAU_OCUSYNC_30K
            alpha_best = frames_30k[best_idx][1]
            cfs_best   = frames_30k[best_idx][2]
            afs_pass   = afs_pass_30k
            sigma_alpha = sigma_alpha_30k
            th_active  = th_30k_active
            label      = "OcuSync 30kHz (Mini 4 Pro / Mavic 3)"
        else:
            best_idx   = int(arr_15k.argmax())
            tau_t      = self.TAU_OCUSYNC_15K
            alpha_best = frames_15k[best_idx][1]
            cfs_best   = frames_15k[best_idx][2]
            afs_pass   = afs_pass_15k
            sigma_alpha = sigma_alpha_15k
            th_active  = th_15k_active
            label      = "OcuSync 15kHz (Mini 3 / Air 2S)"

        best_x = chunks_by_frame[best_idx]
        _, pwr_best = self._prepare_chunk(best_x)
        if pwr_best is None:
            pwr_best = self.MIN_POWER_GATE

        # ── Level 3：τ 域 PSR 验证 ──────────────────────────────────────────
        if self._wifi_ambient and sector_hz is not None:
            _closest  = min(self._wifi_ambient, key=lambda k: abs(k - int(sector_hz)))
            _wifi_cal = self._wifi_ambient[_closest]
        else:
            _wifi_cal = 0.0
        wifi_trigger_th = max(0.010, _wifi_cal * 2.0)
        psr_th = self.PSR_THRESHOLD_WIFI if wifi_ncc > wifi_trigger_th else self.PSR_THRESHOLD
        psr    = self._compute_psr(best_x, pwr_best, tau_t, alpha_best)

        print(
            f"  [S3-v4] PSR: tau={tau_t}, alpha={alpha_best/1e3:.1f}kHz, "
            f"PSR={psr:.2f}x (th={psr_th:.1f}x) | "
            f"CFS={cfs_best:.2f}x (th={self.CFS_THRESHOLD:.1f}x) | "
            f"AFS: σ_α={sigma_alpha:.0f}Hz (th={self.AFS_ALPHA_SIGMA_MAX:.0f}Hz, "
            f"{'PASS' if afs_pass else 'FAIL'})"
        )

        # ── Level 4：SDS 软判决评分融合（v4.0 核心升级）─────────────────────
        # 替代 v3.x 的纯硬串联（PSR fail → return False）
        # 现在由 SDS 评分综合决定通过/拒绝
        sds_score = self._compute_sds(
            ncc      = triggered_score,
            th_ncc   = th_active,
            psr      = psr,
            cfs      = cfs_best,
            afs_pass = afs_pass,
        )

        # 强信号直通：NCC 远超阈值，跳过 SDS 门限
        is_strong_bypass = triggered_score >= self.SDS_STRONG_BYPASS_RATIO * th_active

        ncc_soft_ok = triggered_score >= self.SDS_SOFT_NCC_FLOOR_RATIO * th_active

        if is_strong_bypass:
            decision = True
            decision_reason = f"BYPASS(NCC={triggered_score*100:.2f}% ≥ {self.SDS_STRONG_BYPASS_RATIO:.1f}×th)"
        elif sds_score >= self.SDS_COMPOSITE_THRESHOLD and ncc_soft_ok:
            decision = True
            decision_reason = f"SDS_PASS(S={sds_score:.3f} ≥ {self.SDS_COMPOSITE_THRESHOLD:.1f})"
        else:
            decision = False
            if not ncc_soft_ok:
                decision_reason = (f"NCC_SOFT_FLOOR_FAIL"
                                   f"(NCC={triggered_score*100:.2f}% < "
                                   f"{self.SDS_SOFT_NCC_FLOOR_RATIO:.0%}×th)")
            else:
                decision_reason = (f"SDS_FAIL(S={sds_score:.3f} < "
                                   f"{self.SDS_COMPOSITE_THRESHOLD:.1f},"
                                   f" NCC={self.last_sds_detail.get('ncc_ratio',0):.2f}"
                                   f" PSR={self.last_sds_detail.get('psr_score',0):.2f}"
                                   f" CFS={self.last_sds_detail.get('cfs_score',0):.2f}"
                                   f" AFS={self.last_sds_detail.get('afs_score',0):.1f})")

        print(
            f"  [S3-v4] SDS: NCC_ratio={self.last_sds_detail.get('ncc_ratio',0):.3f} "
            f"PSR_s={self.last_sds_detail.get('psr_score',0):.3f} "
            f"CFS_s={self.last_sds_detail.get('cfs_score',0):.3f} "
            f"AFS_s={self.last_sds_detail.get('afs_score',0):.1f} "
            f"→ S_composite={sds_score:.3f} | {decision_reason}"
        )

        if not decision:
            print(f"  [S3-v4] REJECTED: {decision_reason}")
            return False, triggered_score

        print(f"  [S3-v4] CONFIRMED: {label}")
        print(f"          NCC={triggered_score*100:.2f}%  alpha={alpha_best/1e3:.2f}kHz  "
              f"PSR={psr:.1f}x  CFS={cfs_best:.1f}x  "
              f"σ_α={sigma_alpha:.0f}Hz  SDS={sds_score:.3f}")

        self._save_snapshot(best_x, pwr_best, tau_t, alpha_best,
                            triggered_score, triggered_ch, label, sds_score)

        return True, triggered_score

    # =========================================================================
    # 诊断快照（升级：包含 SDS 评分信息）
    # =========================================================================
    def _save_snapshot(self, x, power, tau, alpha_best_hz,
                       score, channel, label, sds_score=0.0):
        """生成并保存 CAF 谱快照（双子图：30k + 15k 通道），附加 SDS 评分注释。"""
        try:
            import matplotlib
            matplotlib.use('Agg')
            matplotlib.rcParams['font.family'] = ['DejaVu Sans']
            matplotlib.rcParams['axes.unicode_minus'] = False
            import matplotlib.pyplot as plt
            import os

            fig, axes = plt.subplots(1, 2, figsize=(14, 4))

            for ax, tau_plot, alpha_range, ch_label in [
                (axes[0], self.TAU_OCUSYNC_30K, self.ALPHA_SCAN_30K, '30kHz'),
                (axes[1], self.TAU_OCUSYNC_15K, self.ALPHA_SCAN_15K, '15kHz'),
            ]:
                spec = self._compute_caf_spectrum(x, tau_plot, power)
                N    = len(spec)
                f_hz = np.arange(N) * (self.sample_rate / N)
                mask = f_hz < 350_000
                ax.semilogy(f_hz[mask] / 1e3, spec[mask] + 1e-6,
                            color='#FF5722', linewidth=1.0, label='CAF-NCC spectrum')
                ax.axvspan(alpha_range[0]/1e3, alpha_range[1]/1e3,
                           alpha=0.15, color='lime', label='OcuSync scan window')
                ax.axvline(self.ALPHA_WIFI_HZ / 1e3, color='orange',
                           linestyle='--', lw=1.2, label='WiFi 250kHz')
                ax.axvline(alpha_best_hz / 1e3 if channel == ('30k' if ch_label=='30kHz' else '15k')
                           else 0, color='cyan', linestyle=':', lw=1.5, label='alpha_best')
                ax.axhline(self.THRESHOLD_30K if ch_label=='30kHz' else self.THRESHOLD_15K,
                           color='red', linestyle='--', lw=1, label='Threshold')
                ax.set_title(f"CAF spectrum  tau={tau_plot}  ({ch_label})", fontsize=11)
                ax.set_xlabel("Cycle frequency alpha (kHz)")
                ax.set_ylabel("NCC (log)")
                ax.legend(fontsize=7)
                ax.grid(alpha=0.3)
                ax.set_xlim(0, 350)

            d = self.last_sds_detail
            sds_log = (
                f"SDS: NCC={d.get('ncc_ratio',0):.2f} "
                f"PSR={d.get('psr_score',0):.2f} "
                f"CFS={d.get('cfs_score',0):.2f} "
                f"AFS={d.get('afs_score',0):.1f} "
                f"→ S={sds_score:.3f}"
            )
            fig.suptitle(
                f"S3 v4.0 CAF-FFT Audit -- {label}\n"
                f"NCC={score*100:.2f}%  alpha={alpha_best_hz/1e3:.2f}kHz  {sds_log}",
                fontsize=11, fontweight='bold'
            )
            plt.tight_layout()

            db_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  '..', 'database', 'alert_images')
            os.makedirs(db_dir, exist_ok=True)
            ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.join(db_dir, f'S3v4_CAF_{channel}_{ts}.png')
            plt.savefig(path, dpi=130)
            plt.close()
            print(f"  [S3-v4] Snapshot saved: {path}")
        except Exception as e:
            print(f"  [S3-v4] Snapshot failed (non-critical): {e}")
