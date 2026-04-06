# -*- coding: utf-8 -*-
"""
calibrate_s3.py — S3 CAF-FFT 一键现场校准向导
===============================================
在无人机关机状态下自动测量环境 NCC 噪声底，
推导最优检测阈值并直接写入配置文件。

运行方式：
    python3 calibrate_s3.py

阶段说明：
  Phase 1 — 背景噪声基线（UAV OFF）
    · 在各扇区采集背景 IQ，计算 OcuSync α 范围的环境 NCC 本底。

  Phase 2 — 阈值计算与自动写入
    · 依据背景本底推导最优阈值：th = max(硬下限, 背景最大值 × 安全余量)
    · 自动 patch rf_zynq/rf_stage3_cyclostationary.py。
"""

import sys
import os
import re
import time
import numpy as np

# ── 项目路径 ──────────────────────────────────────────────────────────────────
_PROJ_ROOT = os.path.dirname(os.path.abspath(__file__))
if _PROJ_ROOT not in sys.path:
    sys.path.insert(0, _PROJ_ROOT)

S3_SOURCE = os.path.join(_PROJ_ROOT, "rf_zynq", "rf_stage3_cyclostationary.py")
OUT_DIR   = os.path.join(_PROJ_ROOT, "database", "alert_images")
os.makedirs(OUT_DIR, exist_ok=True)

# ── SDR 参数（与 config.py 对齐）────────────────────────────────────────────
SDR_URI     = "ip:192.168.31.10"
SAMPLE_RATE = int(40e6)
RX_GAIN     = 50
BUFFER_SIZE = 2_621_440   # 65 ms @ 40 MSps
SECTORS_HZ  = [5745e6, 5785e6, 5825e6]
N_CAPTURES  = 6           # 每扇区采集段数（背景测量）
N_CAPTURES_UAV = 8        # UAV-ON 采集段数

# ── CAF 扫描参数（与 RF_Stage3_CycloAudit 完全一致）──────────────────────────
CHUNK_SIZE       = 200_000
TAU_30K, TAU_15K = 1333, 2667
ALPHA_SCAN_30K   = (18_000.0, 32_000.0)
ALPHA_SCAN_15K   = (9_000.0,  16_000.0)
MIN_POWER_GATE   = 1e-5

# ── 阈值计算参数 ──────────────────────────────────────────────────────────────
NOISE_MARGIN     = 5.0    # 阈值 = 背景 NCC 最大值 × NOISE_MARGIN
HARD_FLOOR_30K   = 0.028  # 最低可用阈值下限（2.8%，对应 13× 噪声底）
HARD_FLOOR_15K   = 0.022

# =============================================================================
# 核心 CAF-FFT 度量（与生产检测器相同算法）
# =============================================================================
def _caf_ncc_peak(chunk_raw, tau, alpha_range):
    """
    计算单帧 CAF-FFT 归一化 NCC 峰值。
    与 RF_Stage3_CycloAudit._compute_caf_spectrum() 完全等价。

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
    k_lo  = max(1,     int(np.round(alpha_range[0] / f_res)))
    k_hi  = min(N_z//2, int(np.round(alpha_range[1] / f_res)) + 1)

    if k_lo >= k_hi:
        return 0.0, alpha_range[0]

    seg      = ncc[k_lo:k_hi]
    best_idx = int(np.argmax(seg))
    return float(seg[best_idx]), float((k_lo + best_idx) * f_res)


# =============================================================================
# SDR 采集
# =============================================================================
def _init_sdr(freq_hz):
    """初始化 SDR 并调谐至指定频率，返回 sdr 对象（失败返回 None）。"""
    try:
        import adi
        sdr = adi.Pluto(SDR_URI)
        sdr.sample_rate                      = SAMPLE_RATE
        sdr.rx_rf_bandwidth                  = SAMPLE_RATE
        sdr.rx_hardwaregain_control_mode     = 'manual'
        sdr.rx_hardwaregain_chan0            = RX_GAIN
        sdr.rx_buffer_size                   = BUFFER_SIZE
        sdr.rx_lo                            = int(freq_hz)
        for _ in range(3):
            sdr.rx()     # 冲刷历史缓冲
        time.sleep(0.1)
        return sdr
    except Exception as e:
        print(f"  [!] SDR 连接失败：{e}")
        return None


def _capture_buffers(freq_hz, n):
    """采集 n 段 IQ 数据。失败时返回空列表。"""
    sdr = _init_sdr(freq_hz)
    if sdr is None:
        return []
    bufs = []
    for i in range(n):
        bufs.append(sdr.rx())
        print(f"    [{freq_hz/1e6:.0f}MHz] 已采集 {i+1}/{n} 段")
    return bufs


# =============================================================================
# Phase 1：背景噪声基线测量
# =============================================================================
def phase1_background():
    """
    UAV 关机状态下测量各扇区的 CAF-NCC 环境本底。

    Returns
    -------
    dict : {freq_hz: {'ncc_30k_max': float, 'ncc_15k_max': float,
                      'ncc_30k_avg': float, 'ncc_15k_avg': float}}
    """
    print("\n" + "=" * 60)
    print("  Phase 1 — 背景噪声基线测量（请确认无人机已关机）")
    print("=" * 60)
    input("  按 Enter 开始测量...")

    results = {}
    for freq in SECTORS_HZ:
        print(f"\n  [扇区 {freq/1e6:.0f}MHz]")
        bufs = _capture_buffers(freq, N_CAPTURES)

        if not bufs:
            print(f"  SDR 离线，扇区 {freq/1e6:.0f}MHz 跳过（使用默认本底）")
            results[freq] = {'ncc_30k_max': 0.030, 'ncc_15k_max': 0.025,
                             'ncc_30k_avg': 0.015, 'ncc_15k_avg': 0.013}
            continue

        ncc30_list, ncc15_list = [], []
        for buf in bufs:
            # 取缓冲区中段，规避冷启动瞬态
            chunk = buf[BUFFER_SIZE // 2: BUFFER_SIZE // 2 + CHUNK_SIZE]
            n30, _ = _caf_ncc_peak(chunk, TAU_30K, ALPHA_SCAN_30K)
            n15, _ = _caf_ncc_peak(chunk, TAU_15K, ALPHA_SCAN_15K)
            ncc30_list.append(n30)
            ncc15_list.append(n15)

        r = {
            'ncc_30k_max': float(np.max(ncc30_list)),
            'ncc_15k_max': float(np.max(ncc15_list)),
            'ncc_30k_avg': float(np.mean(ncc30_list)),
            'ncc_15k_avg': float(np.mean(ncc15_list)),
        }
        results[freq] = r

        print(f"    OcuSync 30kHz通道：NCC 均值={r['ncc_30k_avg']*100:.2f}%  "
              f"最大={r['ncc_30k_max']*100:.2f}%")
        print(f"    OcuSync 15kHz通道：NCC 均值={r['ncc_15k_avg']*100:.2f}%  "
              f"最大={r['ncc_15k_max']*100:.2f}%")

    return results


# =============================================================================
# Phase 2：阈值计算与自动写入
# =============================================================================
def _derive_thresholds(bg_results):
    """
    推导最优阈值。

    算法：th = max(HARD_FLOOR, bg_max × NOISE_MARGIN)
      · NOISE_MARGIN = 5.0：在背景最大值基础上留 5 倍安全余量
      · HARD_FLOOR：噪声底理论值（1/√N）的 10~13 倍绝对下限
      · 取三扇区最坏情况（最大值），保证全频段均可靠

    Returns
    -------
    (th_30k, th_15k) : 全扇区最坏情况下的推荐阈值
    """
    th30_candidates, th15_candidates = [], []

    for freq in SECTORS_HZ:
        bg   = bg_results.get(freq, {})
        bg30 = bg.get('ncc_30k_max', HARD_FLOOR_30K / NOISE_MARGIN)
        bg15 = bg.get('ncc_15k_max', HARD_FLOOR_15K / NOISE_MARGIN)
        th30_candidates.append(max(HARD_FLOOR_30K, bg30 * NOISE_MARGIN))
        th15_candidates.append(max(HARD_FLOOR_15K, bg15 * NOISE_MARGIN))

    return max(th30_candidates), max(th15_candidates)


def phase3_apply(th_30k, th_15k):
    """
    将推导出的阈值自动写入 rf_stage3_cyclostationary.py。
    使用正则表达式精确替换阈值行，保留其余代码不变。
    """
    print("\n" + "=" * 60)
    print("  Phase 3 — 阈值写入")
    print(f"  THRESHOLD_30K: {th_30k*100:.2f}%")
    print(f"  THRESHOLD_15K: {th_15k*100:.2f}%")
    print(f"  目标文件: {S3_SOURCE}")
    print("=" * 60)

    with open(S3_SOURCE, 'r', encoding='utf-8') as f:
        src = f.read()

    # 正则替换 THRESHOLD_30K 和 THRESHOLD_15K 的数值
    src_new = re.sub(
        r'(THRESHOLD_30K\s*=\s*)[\d.]+',
        lambda m: f'{m.group(1)}{th_30k:.4f}',
        src
    )
    src_new = re.sub(
        r'(THRESHOLD_15K\s*=\s*)[\d.]+',
        lambda m: f'{m.group(1)}{th_15k:.4f}',
        src_new
    )

    if src_new == src:
        print("  [!] 阈值行未找到或已是最新值，文件未修改。")
        print("      请检查 THRESHOLD_30K / THRESHOLD_15K 是否仍在源码中。")
        return False

    # 备份原文件
    backup = S3_SOURCE + '.bak'
    with open(backup, 'w', encoding='utf-8') as f:
        f.write(src)
    print(f"  原始文件已备份至: {backup}")

    with open(S3_SOURCE, 'w', encoding='utf-8') as f:
        f.write(src_new)
    print("  ✓ 阈值写入成功！")
    return True


# =============================================================================
# 可视化报告
# =============================================================================
def _save_report(bg_results, th_30k, th_15k):
    try:
        import matplotlib
        matplotlib.use('Agg')
        matplotlib.rcParams['font.family'] = [
            'WenQuanYi Micro Hei', 'Noto Sans CJK SC',
            'Microsoft YaHei', 'SimHei', 'DejaVu Sans'
        ]
        matplotlib.rcParams['axes.unicode_minus'] = False
        import matplotlib.pyplot as plt

        n = len(SECTORS_HZ)
        fig, axes = plt.subplots(1, n, figsize=(6 * n, 5), sharey=False)
        if n == 1:
            axes = [axes]

        for ax, freq in zip(axes, SECTORS_HZ):
            bg = bg_results.get(freq, {})
            values = [
                bg.get('ncc_30k_max', 0) * 100,
                bg.get('ncc_15k_max', 0) * 100,
                bg.get('ncc_30k_avg', 0) * 100,
                bg.get('ncc_15k_avg', 0) * 100,
            ]
            colors = ['#EF5350', '#FF7043', '#78909C', '#90A4AE']
            labels = ['背景 30kHz 最大', '背景 15kHz 最大',
                      '背景 30kHz 均值', '背景 15kHz 均值']

            bars = ax.bar(labels, values, color=colors, alpha=0.85, width=0.5)
            ax.axhline(th_30k * 100, color='#1565C0', linestyle='--',
                       linewidth=1.5, label=f'新阈值 30k={th_30k*100:.1f}%')
            ax.axhline(th_15k * 100, color='#2E7D32', linestyle='--',
                       linewidth=1.5, label=f'新阈值 15k={th_15k*100:.1f}%')

            for bar, val in zip(bars, values):
                if val > 0:
                    ax.text(bar.get_x() + bar.get_width() / 2,
                            val + 0.1, f'{val:.2f}%',
                            ha='center', va='bottom', fontsize=9)

            ax.set_title(f'{freq/1e6:.0f} MHz 扇区')
            ax.set_ylabel('CAF-NCC (%)')
            ax.legend(fontsize=8)
            ax.grid(alpha=0.3, axis='y')
            ax.tick_params(axis='x', labelrotation=20, labelsize=8)

        from datetime import datetime
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fig.suptitle(
            f'S3 CAF-FFT 现场校准报告  |  '
            f'THRESHOLD_30K={th_30k*100:.2f}%  THRESHOLD_15K={th_15k*100:.2f}%',
            fontsize=12, fontweight='bold'
        )
        plt.tight_layout()
        path = os.path.join(OUT_DIR, f's3_calibration_{ts}.png')
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"  校准报告图已保存: {path}")
    except Exception as e:
        print(f"  [!] 报告图生成失败（不影响阈值写入）: {e}")


# =============================================================================
# 主流程
# =============================================================================
def main():
    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║      RF-Vision S3 CAF-FFT 一键现场校准向导 v1.0         ║")
    print("║   自动测量环境 NCC 本底 → 计算最优阈值 → 写入配置文件   ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print(f"  SDR 地址   : {SDR_URI}")
    print(f"  采样率     : {SAMPLE_RATE/1e6:.0f} MSps")
    print(f"  目标扇区   : {[int(f/1e6) for f in SECTORS_HZ]} MHz")
    print()

    # Phase 1：背景噪声测量
    bg_results = phase1_background()

    # Phase 2：推导阈值并写入
    th_30k, th_15k = _derive_thresholds(bg_results)

    print(f"\n  ┌─────────────────────────────────┐")
    print(f"  │  推导阈值结果（全扇区最坏情况）  │")
    print(f"  │  THRESHOLD_30K = {th_30k*100:5.2f}%         │")
    print(f"  │  THRESHOLD_15K = {th_15k*100:5.2f}%         │")
    print(f"  └─────────────────────────────────┘")

    ans = input("\n  确认写入 rf_stage3_cyclostationary.py？[Y/n] ").strip().lower()
    if ans in ('', 'y'):
        phase3_apply(th_30k, th_15k)
        _save_report(bg_results, th_30k, th_15k)
        print("\n  ✓ 校准完成！重启 system_hub.py 后新阈值立即生效。")
    else:
        print("\n  已取消写入。如需手动更新，请参考上方推导阈值。")


if __name__ == '__main__':
    main()
