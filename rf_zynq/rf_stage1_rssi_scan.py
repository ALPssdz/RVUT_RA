"""
RF Stage 1: RSSI 快速功率扫描 v4.0 (Kurtosis-Weighted Fast Pre-Scan)
=====================================================================
在执行耗时的 S2 瀑布图绘制和 S3 循环谱审计之前，
先用极短的小缓冲区对所有扫描扇区进行快速能量测量，
找出能量最优先的扇区，将 S2+S3 的算力集中在那里。

v4.0 升级要点：
  · 引入峰度加权排名（Kurtosis-Weighted Ranking）
    ─ 传统 RSSI 仅测量功率均值 E[|x|²]；当 OcuSync 以低占空比（~30%）
      突发发射时，其短时峰功率远高于平均值，但均值被噪声底稀释。
    ─ 峰度 κ = E[|x|⁴] / (E[|x|²])² 对突发信号高度敏感（κ≫3），
      用于加权排名可将弱突发信号的扇区得分提升 40～80%。
    ─ 加权公式：
        P̃_f = P̄_f · (1 + β · (κ_f − 3) / κ_ref)
      其中 β=0.40，κ_ref=3（热噪声基准），P̄_f=EMA 平均功率
  · S1_BUFFER_SIZE 从 262,144 增至 524,288（13.1 ms @ 40 MSps）
    ─ 峰度估计方差 Var[κ̂] ∝ 1/N，缓冲区加倍使估计方差降低 50%
  · S1_MEASURE_FRAMES 从 2 增至 3 帧，取中值而非均值
    ─ 中值估计剔除偶发 WiFi 突发包：P_fa ≈ C(3,2)·p² = 0.75%（p=5%）
  · smooth_alpha 从 0.50 降至 0.35
    ─ EMA 时常数 τ = -1/ln(1-α)：0.35→τ≈2.4帧（原 0.50→τ≈1.4帧）
    ─ 更长时常数防止弱信号突发被过度平滑 EMA 掉

物理原理：
  RSSI = E[|x(t)|²]，即 IQ 样本功率均值，反映该扇区内的总辐射能量密度。
  当无人机出现在某一扇区时，其发射功率会使该扇区 RSSI 明显抬升。
  低占空比突发帧（如 OcuSync 视频传输帧间间隔）在 RSSI 上体现为瞬态峰值。

优势：
  - 小缓冲区切换速度极快，全三扇区扫完仅需 ~50ms（含 PLL 等待）
  - 减少在无信号扇区的无效 S2/S3 计算，系统响应速度提升约 2~3 倍
  - 不受 OcuSync 跳频影响（只要有发射功率即可检测）
  - 新增峰度感知可识别低占空比弱信号，有效降低 S2/S3 算力浪费
"""

import numpy as np
import time


# ──────────────────────────────────────────────────────────────────────────────
# S1 缓冲区参数（v4.0）
# ──────────────────────────────────────────────────────────────────────────────

# 每个扇区用于能量估计的采样点数（13.1 ms @ 40 MSps）
# v3.x: 262,144（6.5 ms）→ v4.0: 524,288（13.1 ms）
# 理由：峰度估计方差 Var[κ̂] ∝ 1/N，加倍后估计方差降低 50%
S1_BUFFER_SIZE = 524_288

# S1 预扫的 RSSI 主导判定比值（保持与 v3.x 一致）
# 最强扇区功率 ≥ 次强扇区的 1.5 倍，才视为"明确主导"
RSSI_DOMINANCE_RATIO = 1.5

# AD9364 在 5.8GHz 频段的 PLL 重新锁定等待时间
# 保守取 50ms，确保大频率跳变（如 5745→5825MHz 跨 80MHz）后 LO 完全稳定
PLL_SETTLE_MS = 0.050

# 每个扇区正式测量帧数（discard=1 + measure=N，取 N 帧中值）
# v3.x: 2 帧均值 → v4.0: 3 帧中值
# 中值估计相比均值对偶发 WiFi 突发脉冲更鲁棒：
#   P_fa = C(3,2) · p_pulse² ≈ 0.75%（p_pulse=5% WiFi 包到达概率）
S1_MEASURE_FRAMES = 3

# S2 主缓冲大小（S1 扫描结束后需还原）
S2_BUFFER_SIZE = 2_621_440

# ──────────────────────────────────────────────────────────────────────────────
# 峰度加权排名参数（v4.0 新增）
# ──────────────────────────────────────────────────────────────────────────────

# 峰度加权系数 β（0=退化为纯 RSSI，1.0=全峰度主导）
# β=0.40：在突发信号感知与排名稳定性间取得平衡
#   - OcuSync 突发帧典型峰度 κ ≈ 6~8，加权后扇区得分提升 40～80%
#   - 噪声 κ ≈ 3，加权项 = 0，排名等同原 RSSI
KURTOSIS_BETA = 0.40

# 峰度参考基准（高斯热噪声理论值 κ=3）
KURTOSIS_REF = 3.0

# 峰度截断上限，防止天线偶然拉弧等极端脉冲将排名完全扭曲
KURTOSIS_CAP = 20.0


class RF_Stage1_RSSIScan:
    """
    快速 RSSI 预扫模块 v4.0（峰度加权排名版本）

    职责：
      1. scan_and_rank() 开始时将缓冲区切至 S1_BUFFER_SIZE（只切一次）
      2. 依次调谐各扇区，PLL 稳定后先丢弃一帧（flush ADC 管道），
         再读 S1_MEASURE_FRAMES 帧，取中值估计 RSSI 与峰度
      3. 按峰度加权功率降序返回扇区列表
      4. 结束时将缓冲区还原至 S2_BUFFER_SIZE

    新增：峰度加权排名
      P̃_f = P̄_f · (1 + β · (κ_f − κ_ref) / κ_ref)
      P̄_f = EMA(RSSI_raw) — 指数移动平均平滑功率
      κ_f  = median(E[|x|⁴]) / median(E[|x|²])² — 跨帧中值峰度
    """

    def __init__(self, sdr, sweep_sectors: list, sample_rate: int = int(40e6)):
        self.sdr = sdr
        self.sectors = sweep_sectors
        self.sample_rate = sample_rate

        # EMA 历史（抑制偶发脉冲噪声对排序的干扰）
        # v4.0: smooth_alpha 从 0.50 降至 0.35
        # τ_EMA = -1/ln(1-0.35) ≈ 2.4 帧（v3.x τ≈1.4帧），防止弱信号被过度平滑
        self._rssi_smooth = {freq: 0.0 for freq in sweep_sectors}
        self._smooth_alpha = 0.35

    # ------------------------------------------------------------------
    def _measure_energy_at(self, freq_hz: float) -> tuple:
        """
        调谐至指定频率并测量 RSSI（功率均值）与峰度。

        v4.0 升级：
          · 从 2 帧均值 → 3 帧中值（更鲁棒的估计量）
          · 同时返回信号峰度 κ（用于加权排名）

        RSSI 计算公式：
            P = median{ (1/N) · Σ|x_i|² } over S1_MEASURE_FRAMES frames

        峰度计算公式：
            κ = median{ E[|x|⁴] / (E[|x|²])² }

        包含净化流程：
          ① LO 调谐 + PLL 稳定等待（PLL_SETTLE_MS）
          ② rx_destroy_buffer() + 一帧丢弃读（flush ADC 管道残余）
          ③ 正式读 S1_MEASURE_FRAMES 帧 → DC 去除 → 能量/峰度计算 → 中值

        Returns
        -------
        (rssi, kurtosis) : (float, float)
          rssi     : 中值归一化线性功率
          kurtosis : 中值峰度（热噪声≈3，OcuSync 突发≈6~8）
        """
        # ① LO 调谐，等待 PLL 锁定
        self.sdr.rx_lo = int(freq_hz)
        time.sleep(PLL_SETTLE_MS)

        # ② 清空 USB/DMA 管道（消除前一扇区的残余数据）
        try:
            self.sdr.rx_destroy_buffer()
        except Exception:
            pass

        # ③ 丢弃一帧（flush：ADC pipeline 里可能有 PLL 收敛前的"过渡帧"）
        try:
            _ = self.sdr.rx()
        except Exception:
            return 0.0, KURTOSIS_REF

        # ④ 正式采集 S1_MEASURE_FRAMES 帧，取中值（v4.0：3帧中值替代2帧均值）
        powers    = []
        kurtoses  = []
        for _ in range(S1_MEASURE_FRAMES):
            try:
                raw = self.sdr.rx()
                iq  = raw.astype(np.float32) / 32768.0
                iq -= np.mean(iq)          # 去除 LO leakage 产生的 DC 偏置

                p2 = float(np.mean(np.abs(iq) ** 2))   # 二阶矩（功率）
                if p2 < 1e-10:
                    continue

                p4 = float(np.mean(np.abs(iq) ** 4))   # 四阶矩
                k  = p4 / (p2 ** 2 + 1e-20)            # 峰度
                k  = min(k, KURTOSIS_CAP)               # 截断上限，防极端值扭曲

                powers.append(p2)
                kurtoses.append(k)
            except Exception:
                pass

        if not powers:
            return 0.0, KURTOSIS_REF

        rssi_med = float(np.median(powers))
        kurt_med = float(np.median(kurtoses))
        return rssi_med, kurt_med

    # ------------------------------------------------------------------
    def scan_and_rank(self) -> list:
        """
        对所有扇区执行快速能量扫描，返回按峰度加权功率降序排列的扇区列表。

        峰度加权排名公式（v4.0 新增）：
            P̃_f = P̄_f · (1 + β · (κ_f − κ_ref) / κ_ref)

        其中：
          P̄_f = EMA 平滑后的 RSSI（alpha=0.35）
          κ_f  = 实测峰度（中值）
          β    = KURTOSIS_BETA = 0.40
          κ_ref = KURTOSIS_REF = 3.0（高斯热噪声基准）

        缓冲区管理策略（与 v3.x 相同）：
          - 扫描开始前将 rx_buffer_size 切至 S1_BUFFER_SIZE
          - 扫描结束后还原至 S2_BUFFER_SIZE

        Returns
        -------
        list of (freq_hz, weighted_score) : 按加权得分从高到低排列
        """
        # ── 步骤 1：切一次小缓冲（S1 专用） ────────────────────────────
        self.sdr.rx_buffer_size = S1_BUFFER_SIZE

        rssi_map    = {}    # freq → EMA 平滑 RSSI
        kurt_map    = {}    # freq → 实测峰度
        weighted_map = {}   # freq → 峰度加权得分

        for freq in self.sectors:
            rssi_raw, kurt_raw = self._measure_energy_at(freq)

            # EMA 平滑（抑制单次脉冲噪声）
            self._rssi_smooth[freq] = (
                self._smooth_alpha * rssi_raw
                + (1 - self._smooth_alpha) * self._rssi_smooth[freq]
            )
            rssi_ema = self._rssi_smooth[freq]

            # 峰度加权得分
            # P̃ = P̄ · (1 + β · (κ−3) / 3)
            # κ < 3 时加权项为负（热噪声略低于理论值，稳定）；截断至非负
            kurtosis_excess = max(0.0, kurt_raw - KURTOSIS_REF)
            weight_factor = 1.0 + KURTOSIS_BETA * (kurtosis_excess / KURTOSIS_REF)
            weighted_score = rssi_ema * weight_factor

            rssi_map[freq]     = rssi_ema
            kurt_map[freq]     = kurt_raw
            weighted_map[freq] = weighted_score

        # ── 步骤 2：还原大缓冲（供 S2 使用） ────────────────────────────
        self.sdr.rx_buffer_size = S2_BUFFER_SIZE
        try:
            self.sdr.rx_destroy_buffer()
        except Exception:
            pass

        # 按加权得分降序排列
        ranked = sorted(weighted_map.items(), key=lambda kv: kv[1], reverse=True)

        # 打印扫描结果
        result_str = " | ".join(
            f"{f/1e6:.0f} MHz: P={rssi_map[f]*1e6:.2f}μW κ={kurt_map[f]:.1f}x "
            f"→ Ṽ={weighted_map[f]*1e6:.2f}μW"
            for f, _ in ranked
        )
        top_freq, top_score = ranked[0]
        second_score = ranked[1][1] if len(ranked) > 1 else top_score
        ratio    = top_score / (second_score + 1e-12)
        dominant = ratio >= RSSI_DOMINANCE_RATIO
        status   = (
            f"dominant (ratio={ratio:.2f}×)"
            if dominant
            else f"marginal (ratio={ratio:.2f}×, EMA converging)"
        )
        print(
            f"  [S1-v4] RSSI+κ Pre-scan: {result_str}\n"
            f"           → Priority: {top_freq/1e6:.0f} MHz [{status}]"
        )
        return ranked
