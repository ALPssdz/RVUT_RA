# RA8P1 JDBG UART Master Firmware

Current integration target: CPKHMI-RA8P1 board.

Verified physical link:

```text
RA8P1 SCI9 -> on-board SEGGER J-Link OB virtual COM
Linux device on Orange Pi: /dev/ttyACM0
Baudrate: 2000000
Format: 8-N-1
```

Use this source in e2 studio:

```text
firmware_ra8p1/hal_entry_uart9_master.c
```

RA8P1 responsibilities:

- Own the system state machine.
- Send `START_SCAN` to the RK3588 RF Agent.
- Receive `HEARTBEAT`, `AGENT_READY`, `DETECTION_REPORT`, and `FAULT_REPORT`.
- Audit RF reports and return `MASTER_DECISION`.
- Keep final authority over `CLEAR`, `CANDIDATE`, and `ALERT`.
- Own the final output hook for LED, buzzer, relay, or judge-interface pins.

Current audit logic:

```text
Strong path:
  rf_detected=true and (sds >= 1.35 or ncc >= 0.050)
  -> ALERT

Medium path:
  latest 3 reports contain at least 2 hits with:
    rf_detected=true
    sds >= 1.00
    frequency within 60 MHz
  -> ALERT

Weak path:
  latest 5 reports contain at least 3 hits with:
    rf_detected=true
    sds >= 0.85
    frequency within 60 MHz
  -> ALERT

Candidate:
  rf_detected=true but no alert path is satisfied
  -> CANDIDATE

Clear:
  no RF hit
  -> CLEAR
```

Alert latch behavior:

```text
Once ALERT is entered:
  RA8P1 keeps returning ALERT with reason ALERT_LATCHED.

Release condition:
  5 consecutive reports are clear or below weak threshold.

Manual reset path:
  A RESET_ALERT line can clear the latch during integration testing.
  In the final competition build, this should be connected to a real button,
  judge signal, or competition-defined reset input.
```

Fault handling:

```text
Heartbeat timeout: 10000 ms
On timeout:
  state = FAULT
  decision = CLEAR
  reason = HEARTBEAT_TIMEOUT
```

Competition output hook:

```text
control_outputs_apply(decision, state)
```

All final alarm outputs must be driven from this function after the real FSP
pin mapping is fixed. The RK3588/Orange Pi must not drive final alarm hardware
directly.

Next firmware items before final competition lock:

- Map real LED, buzzer, and competition output pins.
- Add physical reset input for alert latch clearing.
- Add visible fault output for heartbeat timeout and agent fault.
- Tune `STRONG_SDS_THRESHOLD`, `STRONG_NCC_THRESHOLD`, and window thresholds using field data.
