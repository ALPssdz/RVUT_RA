# RF-RA8P1-UAV-Tracker

**English** | [简体中文](README_zh.md)

A 5.8 GHz RF UAV early-warning system led by an **RA8P1 master controller** with **RK3588 RF algorithm co-processing** and HDMI PyQt visualization.

The project has moved away from the earlier RF + K230 vision-fusion design. K230 support has been removed. RA8P1 is now the system control and final-decision authority, while Orange Pi 5 / RK3588 runs RF detection, evidence logging, and the HDMI dashboard.

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
  - UART reports to RA8P1

ZYNQ-7020 + AD9364
  - 5.8 GHz IQ acquisition frontend
  - libiio / pyadi-iio network control
```

## RA8P1 Link

```text
Transport: UART
Baudrate: 921600
Format: 8N1
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

## RF Pipeline

```text
IQ Samples
  -> S1 Kurtosis-Weighted RSSI Pre-scan
  -> S2 Waterfall Generation / YOLO Assist
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

## Migration Status

- K230 runtime path removed
- K230 display panel replaced by RA8P1 master-decision panel
- RA8P1 UART protocol skeleton added
- RF local confirmation is still retained as a transition path
- Next step: consume real RA8P1 UART `MASTER_DECISION` messages for final alerting

