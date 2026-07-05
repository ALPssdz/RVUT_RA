# -*- coding: utf-8 -*-
"""
calibrate_s3.py -- S3 CAF-FFT Auto Background Calibration
==========================================================
Automatically measures ambient NCC noise floor across all sectors,
derives optimal detection thresholds, and writes them to s3_thresholds.json.
No user interaction required -- runs fully autonomously.

Phases:
  Phase 1 -- Background noise baseline (UAV must be OFF)
             Capture IQ across all sectors, compute CAF-NCC floor.
  Phase 2 -- Threshold derivation & JSON persistence
             th = max(HARD_FLOOR, bg_eff x NOISE_MARGIN)
             Runtime RF_Stage3_CycloAudit loads rf_zynq/s3_thresholds.json
"""

import sys
import os
import time
import numpy as np

# -- Project root (this file lives in rf_zynq/, root is two levels up) --------
_PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJ_ROOT not in sys.path:
    sys.path.insert(0, _PROJ_ROOT)

OUT_DIR = os.path.join(_PROJ_ROOT, "database", "alert_images")
os.makedirs(OUT_DIR, exist_ok=True)

# -- SDR parameters (read from config.py -- same values used at runtime) ------
# IMPORTANT: calibration gain MUST equal operational gain (config.SDR_GAIN_DB).
# Loading from config ensures a single source of truth.
from backend_rk3588.config import SDR_URI, SAMPLE_RATE, SDR_GAIN_DB
from rf_zynq.rf_stage3_cyclostationary import RF_Stage3_CycloAudit

RX_GAIN     = SDR_GAIN_DB
BUFFER_SIZE = 2_621_440   # 65 ms @ 40 MSps
SECTORS_HZ  = [5745e6, 5785e6, 5825e6]
N_CAPTURES  = 8           # IQ captures per sector (8 frames -> more stable bg estimate)

# -- CAF scan parameters (identical to RF_Stage3_CycloAudit) ------------------
# Keep these values sourced from the runtime class. Calibration thresholds are
# only valid when the CAF windowing and protocol parameters match detection.
CHUNK_SIZE       = RF_Stage3_CycloAudit.CHUNK_SIZE
OVERLAP          = RF_Stage3_CycloAudit.OVERLAP
TAU_30K          = RF_Stage3_CycloAudit.TAU_OCUSYNC_30K
TAU_15K          = RF_Stage3_CycloAudit.TAU_OCUSYNC_15K
ALPHA_SCAN_30K   = RF_Stage3_CycloAudit.ALPHA_SCAN_30K
ALPHA_SCAN_15K   = RF_Stage3_CycloAudit.ALPHA_SCAN_15K
MIN_POWER_GATE   = RF_Stage3_CycloAudit.MIN_POWER_GATE

# -- Threshold derivation parameters ------------------------------------------
# Formula:
#   bg_eff = BG_P95_WEIGHT * p95 + (1 - BG_P95_WEIGHT) * avg
#   th     = max(HARD_FLOOR, bg_eff * NOISE_MARGIN)
#
# P95 replaces raw max for outlier robustness:
#   raw max is dominated by single burst events (SMPS spikes, multipath) that
#   can be 3-4x above the typical NCC level.  P95 is robust to the top 5% of
#   burst events while still capturing sustained elevated backgrounds.
#   NOISE_MARGIN = 2.0 applies on this tighter estimate; PSR + CFS gates
#   provide the remaining false-alarm suppression at runtime.
NOISE_MARGIN   = 2.0
BG_P95_WEIGHT  = 0.4    # weight on bg_p95; (1-0.4)=0.6 weight on bg_avg
HARD_FLOOR_30K = RF_Stage3_CycloAudit.THRESHOLD_30K
HARD_FLOOR_15K = RF_Stage3_CycloAudit.THRESHOLD_15K

# =============================================================================
# Core CAF-FFT metric (identical algorithm to RF_Stage3_CycloAudit)
# =============================================================================
def _caf_ncc_peak(chunk_raw, tau, alpha_range):
    """
    Single-frame CAF-FFT normalized NCC peak.

    R_x^alpha(tau) via FFT of lag product z[n] = x[n] * conj(x[n-tau]).
    NCC[alpha] = |Z[k]| / (N_z * P_x)

    Returns (peak_ncc, best_alpha_hz)
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


def _wifi_ncc_at_250k(chunk_raw):
    """
    Single-frame WiFi CAF-NCC at cycling frequency 250 kHz.

    IEEE 802.11 OFDM symbol period T_sym = 4 us -> cyclic frequency = 250 kHz.
    This bin is used as a WiFi environment sensor:
      wifi_ncc >> 0  : active WiFi present in the band
      wifi_ncc ~ 0   : quiet RF environment

    Returns
    -------
    float : normalized CAF amplitude at the WiFi bin
    """
    ALPHA_WIFI = 250_000.0   # Hz
    TAU_WIFI   = 128         # samples @ 40MSPS

    x = chunk_raw.astype(np.complex64) / 32768.0
    x -= x.mean()
    power = float(np.mean(np.abs(x) ** 2))
    if power < MIN_POWER_GATE:
        return 0.0

    z   = x[TAU_WIFI:] * np.conj(x[:-TAU_WIFI])
    N_z = len(z)
    Z   = np.fft.fft(z)
    ncc = np.abs(Z) / (N_z * (power + 1e-12))

    f_res = SAMPLE_RATE / N_z
    k_wifi = max(1, min(N_z // 2, int(np.round(ALPHA_WIFI / f_res))))
    return float(ncc[k_wifi])


# =============================================================================
# SDR capture
# =============================================================================
def _init_sdr(freq_hz):
    """Initialize SDR at given frequency. Returns sdr object or None."""
    try:
        import adi
        sdr = adi.Pluto(SDR_URI)
        sdr.sample_rate                  = SAMPLE_RATE
        sdr.rx_rf_bandwidth              = SAMPLE_RATE
        sdr.rx_hardwaregain_control_mode = 'manual'
        sdr.rx_hardwaregain_chan0        = RX_GAIN
        sdr.rx_buffer_size               = BUFFER_SIZE
        sdr.rx_lo                        = int(freq_hz)
        for _ in range(3):
            sdr.rx()   # flush stale buffers
        time.sleep(0.1)
        return sdr
    except Exception as e:
        print(f"  [!] SDR init failed: {e}")
        return None


def _capture_buffers(freq_hz, n):
    """Capture n IQ buffers. Returns empty list on failure."""
    sdr = _init_sdr(freq_hz)
    if sdr is None:
        return []
    bufs = []
    for i in range(n):
        bufs.append(sdr.rx())
        print(f"    [{freq_hz/1e6:.0f}MHz] captured {i+1}/{n}")
    return bufs


# =============================================================================
# Phase 1: Background noise baseline
# =============================================================================
def phase1_background():
    """
    Measure CAF-NCC ambient floor across all sectors (UAV must be OFF).

    Upgrade over v1.0:
      Previously only one chunk (the center slice) was extracted per buffer.
      Now all overlapping chunks (using RF_Stage3_CycloAudit.OVERLAP)
      are analyzed per buffer, giving ~13x more frames per sector for a far
      more stable and representative background distribution.

    Returns
    -------
    dict : {freq_hz: {'ncc_30k_p95': float, 'ncc_15k_p95': float,
                      'ncc_30k_avg': float, 'ncc_15k_avg': float,
                      'ncc_30k_max': float, 'ncc_15k_max': float}}
    """
    print("\n" + "=" * 60)
    print("  Phase 1 -- Background noise baseline (UAV OFF)")
    print(f"  Overlap={OVERLAP*100:.0f}%  ChunkSize={CHUNK_SIZE}  Captures={N_CAPTURES}")
    print("=" * 60)

    step_size = int(CHUNK_SIZE * (1.0 - OVERLAP))

    results = {}
    for freq in SECTORS_HZ:
        print(f"\n  [Sector {freq/1e6:.0f}MHz]")
        bufs = _capture_buffers(freq, N_CAPTURES)

        if not bufs:
            print(f"  SDR offline -- sector {freq/1e6:.0f}MHz skipped (using defaults)")
            results[freq] = {
                'ncc_30k_p95': 0.030, 'ncc_15k_p95': 0.025,
                'ncc_30k_avg': 0.015, 'ncc_15k_avg': 0.013,
                'ncc_30k_max': 0.035, 'ncc_15k_max': 0.030,
            }
            continue

        ncc30_all, ncc15_all = [], []   # all chunk-level NCC values across all buffers
        wifi_all  = []                  # WiFi 250kHz CAF-NCC per chunk
        for buf in bufs:
            buf_arr = np.asarray(buf)
            total   = len(buf_arr)
            # Slide overlapping windows across the full buffer
            for i in range(0, total - CHUNK_SIZE, step_size):
                chunk = buf_arr[i: i + CHUNK_SIZE]
                n30, _ = _caf_ncc_peak(chunk, TAU_30K, ALPHA_SCAN_30K)
                n15, _ = _caf_ncc_peak(chunk, TAU_15K, ALPHA_SCAN_15K)
                w      = _wifi_ncc_at_250k(chunk)     # WiFi 环境探针
                ncc30_all.append(n30)
                ncc15_all.append(n15)
                wifi_all.append(w)

        arr30 = np.array(ncc30_all)
        arr15 = np.array(ncc15_all)
        arr_w = np.array(wifi_all)
        r = {
            'ncc_30k_p95': float(np.percentile(arr30, 95)),
            'ncc_15k_p95': float(np.percentile(arr15, 95)),
            'ncc_30k_avg': float(arr30.mean()),
            'ncc_15k_avg': float(arr15.mean()),
            'ncc_30k_max': float(arr30.max()),
            'ncc_15k_max': float(arr15.max()),
            'wifi_avg':    float(arr_w.mean()),    # WiFi 250kHz ambient 均值
            'wifi_p95':    float(np.percentile(arr_w, 95)),  # WiFi P95（PSR 触发线依据）
        }
        results[freq] = r

        n_chunks = len(ncc30_all)
        print(f"    Chunks analyzed: {n_chunks} "
              f"({N_CAPTURES} buffers x ~{n_chunks//N_CAPTURES} chunks/buf)")
        print(f"    OcuSync 30kHz: avg={r['ncc_30k_avg']*100:.2f}%  "
              f"p95={r['ncc_30k_p95']*100:.2f}%  max={r['ncc_30k_max']*100:.2f}%")
        print(f"    OcuSync 15kHz: avg={r['ncc_15k_avg']*100:.2f}%  "
              f"p95={r['ncc_15k_p95']*100:.2f}%  max={r['ncc_15k_max']*100:.2f}%")
        wifi_trigger = max(0.010, r['wifi_avg'] * 2.0)
        print(f"    WiFi@250kHz:   avg={r['wifi_avg']*100:.3f}%  "
              f"p95={r['wifi_p95']*100:.3f}%  "
              f"=> PSR trigger_th={wifi_trigger*100:.3f}%")

    return results


# =============================================================================
# Phase 2: Threshold derivation & auto-patch
# =============================================================================
def _derive_thresholds(bg_results):
    """
    Derive per-sector thresholds using a P95-based outlier-robust estimate.

    Formula:
      bg_eff = BG_P95_WEIGHT * bg_p95 + (1 - BG_P95_WEIGHT) * bg_avg
      th     = max(HARD_FLOOR, bg_eff * NOISE_MARGIN)

    Statistical rationale:
      P95 replaces raw max to suppress single-spike outliers (SMPS bursts)
      that can inflate the threshold by 3-4x versus the typical background.
      P95 still captures the 95th-percentile noise level (sustained bursts),
      while the remaining suppression is delegated to PSR + CFS runtime gates.
      NOISE_MARGIN=2.0 on a P95-based estimate is statistically equivalent to
      ~4x on the raw max for typical 5.8 GHz indoor environments.

    Returns
    -------
    dict: {freq_hz: {'th_30k': float, 'th_15k': float}}
    """
    per_sector = {}
    for freq in SECTORS_HZ:
        bg    = bg_results.get(freq, {})
        # P95 + avg (fallback to conservative defaults if absent)
        p95_30 = bg.get('ncc_30k_p95', HARD_FLOOR_30K / NOISE_MARGIN)
        avg30  = bg.get('ncc_30k_avg', p95_30 * 0.5)
        p95_15 = bg.get('ncc_15k_p95', HARD_FLOOR_15K / NOISE_MARGIN)
        avg15  = bg.get('ncc_15k_avg', p95_15 * 0.5)

        # Weighted P95-based effective background
        bg_eff_30 = BG_P95_WEIGHT * p95_30 + (1 - BG_P95_WEIGHT) * avg30
        bg_eff_15 = BG_P95_WEIGHT * p95_15 + (1 - BG_P95_WEIGHT) * avg15

        per_sector[freq] = {
            'th_30k': max(HARD_FLOOR_30K, bg_eff_30 * NOISE_MARGIN),
            'th_15k': max(HARD_FLOOR_15K, bg_eff_15 * NOISE_MARGIN),
        }
    return per_sector


THRESHOLD_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "s3_thresholds.json")


def phase2_apply(per_sector_th, bg_results=None):
    """
    Persist per-sector calibrated thresholds to s3_thresholds.json.

    JSON schema (v2):
      {
        "sectors": {
          "5745000000": {"th_30k": 0.050, "th_15k": 0.030},
          ...
        },
        "wifi_ambient": {
          "5745000000": 0.0045,  # mean WiFi NCC at 250kHz (for adaptive PSR threshold)
          ...
        },
        "calibrated_at": "..."
      }
    RF_Stage3_CycloAudit.__init__() reads both 'sectors' and 'wifi_ambient'.
    """
    import json
    from datetime import datetime
    print("\n" + "=" * 60)
    print("  Phase 2 -- Saving per-sector thresholds to JSON")
    for freq, th in per_sector_th.items():
        print(f"  {freq/1e6:.0f} MHz:  "
              f"TH_30k={th['th_30k']*100:.2f}%   TH_15k={th['th_15k']*100:.2f}%")
    print(f"  File : {THRESHOLD_JSON}")
    print("=" * 60)

    payload = {
        "sectors": {
            str(int(freq)): {
                "th_30k": round(th["th_30k"], 6),
                "th_15k": round(th["th_15k"], 6),
            }
            for freq, th in per_sector_th.items()
        },
        "calibrated_at": datetime.now().isoformat(timespec='seconds'),
    }

    # Attach WiFi ambient data (for adaptive PSR threshold in run_spectral_audit)
    if bg_results is not None:
        wifi_payload = {}
        for freq in SECTORS_HZ:
            bg = bg_results.get(freq, {})
            wifi_mean = bg.get('wifi_avg', 0.0)
            wifi_payload[str(int(freq))] = round(wifi_mean, 6)
        payload["wifi_ambient"] = wifi_payload
        print("  WiFi ambient saved:")
        for k, v in wifi_payload.items():
            trigger = max(0.010, v * 2.0)
            print(f"    {int(k)//1000000:.0f} MHz  "
                  f"wifi_mean={v*100:.3f}%  PSR_trigger_th={trigger*100:.3f}%")

    with open(THRESHOLD_JSON, 'w') as f:
        json.dump(payload, f, indent=2)
    print(f"  OK -- thresholds saved.")


# =============================================================================
# Calibration report plot
# =============================================================================
def _save_report(bg_results, per_sector_th):
    try:
        import matplotlib
        matplotlib.use('Agg')
        matplotlib.rcParams['font.family'] = ['DejaVu Sans']
        matplotlib.rcParams['axes.unicode_minus'] = False
        import matplotlib.pyplot as plt

        n    = len(SECTORS_HZ)
        fig, axes = plt.subplots(1, n, figsize=(6 * n, 5), sharey=False)
        if n == 1:
            axes = [axes]

        for ax, freq in zip(axes, SECTORS_HZ):
            bg = bg_results.get(freq, {})
            th = per_sector_th.get(freq, {'th_30k': HARD_FLOOR_30K, 'th_15k': HARD_FLOOR_15K})
            th_30k = th['th_30k']
            th_15k = th['th_15k']
            wifi_mean = bg.get('wifi_avg', 0.0)
            wifi_trigger = max(0.010, wifi_mean * 2.0)
            values = [
                bg.get('ncc_30k_p95', 0) * 100,
                bg.get('ncc_15k_p95', 0) * 100,
                bg.get('ncc_30k_avg', 0) * 100,
                bg.get('ncc_15k_avg', 0) * 100,
                bg.get('ncc_30k_max', 0) * 100,
                bg.get('ncc_15k_max', 0) * 100,
                bg.get('wifi_avg',    0) * 100,
                bg.get('wifi_p95',    0) * 100,
            ]
            colors = ['#EF5350', '#FF7043', '#78909C', '#90A4AE',
                      '#B71C1C', '#BF360C', '#AB47BC', '#7B1FA2']
            labels = ['30k P95', '15k P95',
                      '30k avg', '15k avg',
                      '30k max', '15k max',
                      'WiFi avg', 'WiFi P95']

            bars = ax.bar(labels, values, color=colors, alpha=0.85, width=0.6)
            ax.axhline(th_30k * 100, color='#1565C0', linestyle='--',
                       linewidth=1.5, label=f'TH_30k={th_30k*100:.1f}%')
            ax.axhline(th_15k * 100, color='#2E7D32', linestyle='--',
                       linewidth=1.5, label=f'TH_15k={th_15k*100:.1f}%')
            ax.axhline(wifi_trigger * 100, color='#FF6F00', linestyle=':',
                       linewidth=1.2, label=f'PSR_trig={wifi_trigger*100:.2f}%')

            for bar, val in zip(bars, values):
                if val > 0:
                    ax.text(bar.get_x() + bar.get_width() / 2,
                            val + 0.1, f'{val:.2f}%',
                            ha='center', va='bottom', fontsize=9)

            ax.set_title(f'{freq/1e6:.0f} MHz Sector')
            ax.set_ylabel('CAF-NCC (%)')
            ax.legend(fontsize=8)
            ax.grid(alpha=0.3, axis='y')
            ax.tick_params(axis='x', labelrotation=15, labelsize=8)

        from datetime import datetime
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fig.suptitle(
            f'S3 CAF-FFT Calibration Report  (NOISE_MARGIN={NOISE_MARGIN}x)',
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
# Main (fully automatic -- no user input required)
# =============================================================================
def main():
    print()
    print("=" * 62)
    print("  RF-Vision S3 CAF-FFT Auto Calibration v2.0")
    print("  Multi-chunk P95 + WiFi ambient sensing + Adaptive PSR")
    print("=" * 62)
    print(f"  SDR    : {SDR_URI}")
    print(f"  Fs     : {SAMPLE_RATE/1e6:.0f} MSps")
    print(f"  Sectors: {[int(f/1e6) for f in SECTORS_HZ]} MHz")
    print()

    # Phase 1: background measurement
    bg_results = phase1_background()

    # Phase 2: derive per-sector thresholds
    per_sector_th = _derive_thresholds(bg_results)

    print("\n  +" + "-" * 46 + "+")
    print(f"  | Per-sector derived thresholds  (NOISE_MARGIN = {NOISE_MARGIN}x)   |")
    for freq, th in per_sector_th.items():
        line = (f"  |  {freq/1e6:.0f} MHz:  "
                f"TH_30k={th['th_30k']*100:5.2f}%   "
                f"TH_15k={th['th_15k']*100:5.2f}%")
        print(line.ljust(48) + "|")
    print("  +" + "-" * 46 + "+")

    # Auto-write
    phase2_apply(per_sector_th, bg_results=bg_results)
    _save_report(bg_results, per_sector_th)

    # WiFi environment summary
    print("\n  +" + "-" * 50 + "+")
    print("  | WiFi Ambient Environment Summary               |")
    for freq in SECTORS_HZ:
        bg = bg_results.get(freq, {})
        w_avg = bg.get('wifi_avg', 0.0)
        w_trig = max(0.010, w_avg * 2.0)
        line = (f"  |  {freq/1e6:.0f} MHz  "
                f"WiFi_avg={w_avg*100:.3f}%  "
                f"PSR_trigger={w_trig*100:.3f}%")
        print(line.ljust(52) + "|")
    print("  +" + "-" * 50 + "+")

    print("\n  Calibration complete. New thresholds are active.\n")


if __name__ == '__main__':
    main()
