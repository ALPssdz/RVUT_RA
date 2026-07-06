# RF-RA8P1-UAV-Tracker

**English** | [简体中文](README_zh.md)

A 5.8 GHz RF UAV early-warning system led by an **RA8P1 master controller** with an **RK3588 external RF compute unit** and HDMI PyQt visualization.

The project has moved away from the earlier RF + K230 vision-fusion design. K230 support has been removed. RA8P1 is now the system control and final-decision authority, while Orange Pi 5 / RK3588 is an external compute/display unit for RF detection, evidence logging, and the HDMI dashboard.

## Roles

```text
RA8P1 Master Controller
  - system state machine
  - scan/calibration control
  - RF report intake
  - final alert decision
  - LED/buzzer/competition I/O

Orange Pi 5 / RK3588
  - RF Agent
  - S1/S2/S3 RF detection pipeline
  - SQLite alert database
  - PyQt HDMI dashboard
  - JDBG virtual serial reports to RA8P1

ZYNQ-7020 + AD9364
  - 5.8 GHz IQ acquisition frontend
  - libiio / pyadi-iio network control
```

## RA8P1 Link

```text
Transport: CPKHMI-RA8P1 JDBG virtual COM, wired to RA8P1 SCI9 UART
Device: /dev/ttyACM0
Baudrate: 2000000, 8-N-1
Initial protocol: JSON Lines + checksum
```

RA8P1 commands:

```text
START_SCAN
STOP_SCAN
RUN_CALIBRATION
RESET_ALERT
GET_STATUS
```

RK3588 reports:

```text
HEARTBEAT
AGENT_READY
CALIBRATION_DONE
DETECTION_REPORT
FAULT_REPORT
```

RA8P1 decisions:

```text
MASTER_DECISION
```

Formal control constraints:

```text
RA8P1_REQUIRED=True
RF pipeline is locked when RA8P1 is offline
The local GUI start button cannot bypass RA8P1 master control
RA8P1 START_SCAN is the only formal scan authorization
RA8P1 STOP_SCAN / RESET_ALERT / RUN_CALIBRATION are executed as master commands
```

## RF Pipeline

```text
IQ Samples
  -> S1 Kurtosis-Weighted RSSI Pre-scan
  -> S2 Waterfall Display
  -> S3 Cyclostationary Audit
  -> RF Detection Report
  -> RA8P1 Master Decision
  -> SQLite + HDMI PyQt Console
```

## Layout

```text
RF-Vision-UAV-Tracker/
├── system_hub.py
├── backend_rk3588/
├── rf_zynq/
├── protocol/
├── mock/
├── ui_qt/
├── database/
├── tools/
└── deploy_orangepi.sh
```

## Run

```bash
python3 system_hub.py
```

Run S3 calibration directly:

```bash
python3 rf_zynq/calibrate_s3.py
```

Run the RA8P1 protocol mock:

```bash
python3 mock/mock_ra8p1.py
```

## False-Positive Diagnostics

Runtime diagnostics are recorded under:

```text
diagnostics/captures/session_YYYYmmdd_HHMMSS/
├── events.jsonl
├── runtime.log
├── frames/
└── iq/
```

Package the latest session on Orange Pi:

```bash
cd ~/RVUT_RA/RF-Vision-UAV-Tracker
python3 tools/package_diagnostics.py
```

Copy it back to the laptop:

```bash
scp orangepi@192.168.31.34:~/RVUT_RA/RF-Vision-UAV-Tracker/diagnostics/*.tar.gz .
```

Current false-positive suppression:

```text
S3 SDS threshold: 1.08
Normal detection requires PSR / CFS / AFS hard validation to all pass
Strong NCC bypass still requires PSR and at least one of CFS/AFS
Diagnostics are written asynchronously to avoid heartbeat stalls
RA8P1 heartbeat timeout: 10000 ms
```

## Migration Status

- K230 runtime path removed
- K230 display panel replaced by RA8P1 master-decision panel
- RA8P1 JDBG UART protocol and RK3588 link client added
- Real RA8P1 `MASTER_DECISION` messages are consumed by the hub
- Competition default is `RA8P1_REQUIRED=True`: the RF pipeline is locked until RA8P1 is online
- Developers may set `RA8P1_REQUIRED=False` only for offline RF fallback testing
