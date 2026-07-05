# RA8P1 UART Protocol

Transport: UART, 921600 baud, 8N1.

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

Example:

```json
{"type":"DETECTION_REPORT","seq":12,"freq_mhz":5785,"ncc":0.034,"sds":1.18,"rf_detected":true,"suggestion":"ALERT","checksum":12345}
```

