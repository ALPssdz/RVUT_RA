# RA8P1 Serial Link Protocol

Transport on CPKHMI-RA8P1: RA8P1 SCI9 UART through the on-board SEGGER J-Link
OB virtual COM port.

Default RK3588 device: `/dev/ttyACM0`.

Default baudrate: `2000000`, 8-N-1.

Initial frame format: one compact JSON object per line with a 16-bit additive
`checksum` over the sorted JSON payload without the checksum field.

## Commands from RA8P1

- `START_SCAN`
- `STOP_SCAN`
- `RUN_CALIBRATION`
- `RESET_ALERT`
- `GET_STATUS`

## Reports from RK3588

- `HEARTBEAT`
- `AGENT_READY`
- `CALIBRATION_DONE`
- `DETECTION_REPORT`
- `FAULT_REPORT`

## Decisions from RA8P1

- `MASTER_DECISION`

Decision values:

- `CLEAR`: no confirmed target, continue scanning.
- `CANDIDATE`: RF Agent reported a possible target, but RA8P1 has not promoted it to final alert.
- `ALERT`: RA8P1 master controller has confirmed the target and latched the alert.

Current RA8P1 audit logic:

- Strong path: one report can trigger `ALERT` when `rf_detected=true` and `sds >= 1.35` or `ncc >= 0.050`.
- Medium path: `ALERT` when at least 2 of the latest 3 reports pass `rf_detected=true`, `sds >= 1.00`, and frequency consistency.
- Weak path: `ALERT` when at least 3 of the latest 5 reports pass `rf_detected=true`, `sds >= 0.85`, and frequency consistency.
- Frequency consistency tolerance: `60 MHz`.
- Alert latch: after `ALERT`, RA8P1 keeps returning `ALERT_LATCHED` until the RF Agent sends 5 consecutive clear/weak reports or a reset command is received.
- Fault protection: if heartbeat is missing for more than 10000 ms, RA8P1 enters fault handling and returns `CLEAR` with `HEARTBEAT_TIMEOUT`.

Current RK3588 S3 false-positive suppression:

- SDS threshold is `1.08`.
- SDS cannot pass by NCC alone.
- Normal detection requires PSR, CFS, and AFS hard validation to all pass.
- Strong NCC bypass still requires PSR pass and at least one of CFS/AFS pass.

Example:

```json
{"type":"DETECTION_REPORT","seq":12,"freq_mhz":5785,"ncc":0.034,"sds":1.18,"rf_detected":true,"suggestion":"ALERT","checksum":12345}
```
