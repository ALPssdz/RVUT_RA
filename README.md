# RF-Vision-UAV-Tracker

**English** | [简体中文](README_zh.md)

## Table of Contents
- [1. Introduction](#1-introduction)
- [2. System Architecture](#2-system-architecture)
- [3. Three-Stage RF Detection Pipeline](#3-three-stage-rf-detection-pipeline)
- [4. Asymmetric Fusion Methodology](#4-asymmetric-fusion-methodology)
- [5. Software Stack & Module Organization](#5-software-stack--module-organization)
- [6. Deployment Instructions](#6-deployment-instructions)

## 1. Introduction
RF-Vision-UAV-Tracker is a distributed, multi-modal Unmanned Aerial Vehicle (UAV) detection and early-warning system. By integrating Software-Defined Radio (SDR) with edge-computing optical vision, this system addresses the inherent limitations of single-sensor detection methodologies (e.g., localized blind spots and vulnerability to radio silence). It employs an asymmetric Out-Of-Band (OOB) sensor fusion architecture to achieve robust target acquisition and evidentiary logging in complex electromagnetic environments.

The central controller runs on **Orange Pi 5 (RK3588)**, leveraging the onboard **NPU (Neural Processing Unit)** via RKNN-Toolkit2 to execute hardware-accelerated YOLOv8 inference on the RF spectrogram stream, delivering significantly better real-time performance than CPU-only inference.

## 2. System Architecture
The hardware topology is established upon a Gigabit Ethernet LAN, interconnecting three decoupled physical nodes:

*   **RF Sensing Node (ZYNQ-7020 + AD9364)**
    Primary omnidirectional detection array. Leverages the 56 MHz tuning bandwidth of the AD9364 transceiver with vertically polarized dual-band antennas to sweep the 5.8 GHz ISM band (DJI OcuSync channels). Streams IQ samples over TCP/IP to the central controller via `libiio` / `pyadi-iio`.

*   **Vision Sensing Node (Kendryte K230)**
    Zenith-compensation node equipped with a 1080P optical sensor and onboard KPU for hardware-accelerated YOLO inference. Offsets the RF antenna "Zenith Null" (overhead polarization blind spot). Sends video via RTSP and lightweight alert telemetry (bounding box + confidence) via a stateless UDP side-channel.

*   **Central Controller (Orange Pi 5 — RK3588)**
    Global event bus and aggregation hub. Executes the three-stage RF detection pipeline, runs YOLOv8 spectrogram inference on the RK3588 NPU via RKNN-Toolkit-Lite2, fuses multi-modal evidence, and serves a PyQt5 GUI with real-time visualization and SQLite3 alert persistence.

## 3. Three-Stage RF Detection Pipeline

```
IQ Samples (AD9364, 40 MSps, 2.62M samples/burst)
        │
        ▼
  Stage 1 — RSSI Pre-scan (S1)
    Fast power measurement across all 5.8 GHz sectors.
    Selects the dominant frequency band for deep inspection.
        │
        ▼
  Stage 2 — Spectrogram + YOLOv8 (S2)
    STFT waterfall image (640×640, HOT colormap).
    YOLOv8n inference on RK3588 NPU via RKNN (~30 ms).
        │
        ▼
  Stage 3 — Cyclic Frequency Discriminator (S3)  [v3.0]
    Cyclic Autocorrelation Function (CAF) with FFT-accelerated α-scan.
    Exploits the orthogonality of OFDM cycle frequencies to achieve
    protocol-level separation between OcuSync and Wi-Fi — even when
    Wi-Fi is 20 dB stronger than the UAV signal.

    Target cycle frequencies:
      OcuSync 2.0 (Δf=15 kHz, τ=2667): α_sym = Fs/(N_fft+N_cp) ≈ 12 kHz
      OcuSync 3.0/4.0 (Δf=30 kHz, τ=1333): α_sym ≈ 24 kHz
      Wi-Fi 802.11 (Δf=312.5 kHz, τ=128): α_sym = 250 kHz  ← orthogonal

    Wi-Fi leakage into OcuSync channel (N=200000, Fs=40 MHz):
      NCC_WiFi ≈ P_WiFi · sinc(1130) ≈ 0.028% of Wi-Fi power

    Four-stage decision funnel:
      L1 — Frame-level CAF peak extraction
      L2 — Combined statistic (peak×0.45 + avg×0.55) > adaptive threshold
      L3 — τ-domain PSR (Peak-to-Sidelobe Ratio) ≥ 2.5×
      L4 — α-domain CFS (Cyclic Frequency Sharpness) ≥ 1.8×
```

## 4. Asymmetric Fusion Methodology
Differing from conventional Boolean AND-logic fusion, this system implements independent, asynchronous trigger paths to maximize detection recall:

1.  **RF Trigger (Primary)** — S3 CAF-FFT discriminator confirms OcuSync protocol fingerprint and fires an alert with a cyclic spectrum snapshot.
2.  **Visual Trigger (Secondary)** — K230 UDP telemetry independently triggers an alert, compensating for UAVs under RF silence or traversing the antenna null.

Both trigger paths produce a fused composite evidence image (RF waterfall + optical frame) stored in the SQLite3 alert database.

## 5. Software Stack & Module Organization

```
RF-Vision-UAV-Tracker/
├── system_hub.py            # Entry point & central pipeline orchestrator
├── config.py                # Centralized hardware configuration
├── backend_rk3588/
│   └── main_rf_pipeline.py  # RFToolchain: S1→S2→S3 pipeline controller
├── rf_zynq/
│   ├── rf_stage1_rssi_scan.py       # S1: Fast RSSI frequency scan
│   ├── rf_stage2_waterfall_yolo.py  # S2: IQ → STFT waterfall tensor
│   ├── rf_stage3_cyclostationary.py # S3: CAF-FFT cyclic frequency discriminator
│   └── rknn_infer.py                # RKNN-Lite2 YOLOv8 NPU inference wrapper
├── vision_k230/
│   └── k230_client.py       # RTSP video + UDP telemetry network client
├── ui_qt/
│   └── gui_host.py          # PyQt5 presentation layer (View only)
├── database/
│   └── db_manager.py        # SQLite3 alert persistence & LRU management
├── tools/
│   └── convert_yolo_to_rknn.py  # YOLOv8 → RKNN INT8 offline converter
├── mock_transmitter/
│   ├── uav_tx_gui.py        # PlutoSDR UAV RF target simulator GUI
│   └── mock_k230.py         # PC-side K230 simulator (MJPEG + UDP)
├── calibrate_s3.py              # S3 one-click field calibration wizard
├── diag_s3_false_positive.py    # S3 background noise full-spectrum diagnostics (low-level)
├── diag_uav_on_calibration.py   # S3 live UAV signal strength measurement (low-level)
├── deploy_orangepi.sh           # One-shot Orange Pi 5 environment setup
└── start_rf_vision.sh           # One-click system launch script
```

## 6. Deployment Instructions

```bash
# Clone and run the automated environment setup script
git clone https://github.com/ALPssdz/RF-Vision-UAV-Tracker.git
cd RF-Vision-UAV-Tracker
bash deploy_orangepi.sh

# Convert YOLOv8 weights to RKNN INT8 (run on x86 Linux / WSL2)
python tools/convert_yolo_to_rknn.py

# Copy best.rknn to the target path, then launch
python3 system_hub.py
```

### S3 Threshold Calibration (Recommended After Deployment)

After deploying in a new RF environment, run the one-click calibration wizard
(completes in ~3 minutes):

```bash
python3 calibrate_s3.py
```

The interactive wizard guides you through three phases:

```
Phase 1 — Background Noise Baseline (UAV OFF)
  → Automatically measures CAF-NCC ambient floor across all sectors

Phase 2 — Live UAV Signal Measurement (UAV ON, optional)
  → Measures real OcuSync signal strength to validate SNR margin

Phase 3 — Threshold Derivation & Auto-Patch
  → Calculates optimal THRESHOLD_30K / THRESHOLD_15K
  → Automatically patches rf_zynq/rf_stage3_cyclostationary.py
  → Saves calibration report plot (database/alert_images/)
```

Restart the system after calibration — new thresholds take effect immediately:

```bash
python3 system_hub.py
```