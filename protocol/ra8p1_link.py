# -*- coding: utf-8 -*-
"""
RA8P1 serial control link.

On CPKHMI-RA8P1, RA8P1 SCI9 is wired to the on-board SEGGER J-Link OB virtual
COM port. On RK3588 this normally appears as /dev/ttyACM0 at 2,000,000 baud.
The class is deliberately non-fatal: if the cable or firmware is absent, the RF
agent reports the link as offline. Competition mode can require this link before
the RF pipeline is allowed to run.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Any, Dict, Optional

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime on target
    serial = None

from protocol.link_protocol import DEFAULT_BAUDRATE, RA8P1Protocol, decode_message, encode_message


class RA8P1Link:
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.protocol = RA8P1Protocol()

        self._serial = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._rx_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()

        self.online = False
        self.last_error = ""
        self.last_rx_at = 0.0
        self.last_tx_at = 0.0
        self.rx_count = 0
        self.tx_count = 0
        self.last_command: Dict[str, Any] = {}
        self.last_decision: Dict[str, Any] = {}

    def start(self) -> bool:
        if self.online:
            return True

        if serial is None:
            self.last_error = "pyserial not installed"
            self.online = False
            return False

        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=0.2,
            )
        except Exception as exc:
            self.last_error = str(exc)
            self.online = False
            return False

        self._running = True
        self.online = True
        self.last_error = ""
        self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._rx_thread.start()
        self.send(self.protocol.agent_ready())
        return True

    def stop(self):
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=0.5)
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self.online = False
        self._serial = None

    def send(self, payload: Dict[str, Any]) -> bool:
        if not self.online or self._serial is None:
            return False
        try:
            frame = encode_message(payload)
            with self._lock:
                self._serial.write(frame)
                self._serial.flush()
            self.tx_count += 1
            self.last_tx_at = time.time()
            return True
        except Exception as exc:
            self.last_error = str(exc)
            self._mark_offline()
            return False

    def send_heartbeat(self) -> bool:
        return self.send(self.protocol.heartbeat())

    def send_detection_report(
        self,
        freq_mhz: float,
        ncc: float,
        sds: float,
        rf_detected: bool,
        suggestion: str,
    ) -> bool:
        return self.send(
            self.protocol.detection_report(
                freq_mhz=freq_mhz,
                ncc=ncc,
                sds=sds,
                rf_detected=rf_detected,
                suggestion=suggestion,
            )
        )

    def poll(self) -> list[Dict[str, Any]]:
        messages = []
        while True:
            try:
                messages.append(self._rx_queue.get_nowait())
            except queue.Empty:
                break
        return messages

    def summary(self) -> str:
        if self.online:
            age = time.time() - self.last_rx_at if self.last_rx_at else -1.0
            age_text = f"RX {age:.1f}s ago" if age >= 0 else "waiting RX"
            return f"RA8P1 JDBG UART: ONLINE {self.port} @ {self.baudrate} ({age_text})"
        return f"RA8P1 JDBG UART: OFFLINE {self.port} @ {self.baudrate} ({self.last_error or 'not connected'})"

    def _read_loop(self):
        while self._running and self._serial is not None:
            try:
                line = self._serial.readline()
            except Exception as exc:
                self.last_error = str(exc)
                self._mark_offline()
                break

            if not line:
                continue

            try:
                msg = decode_message(line)
            except Exception as exc:
                self.last_error = f"decode error: {exc}"
                continue

            self.rx_count += 1
            self.last_rx_at = time.time()
            msg_type = msg.get("type", "")
            if msg_type in {"START_SCAN", "STOP_SCAN", "RUN_CALIBRATION", "RESET_ALERT", "GET_STATUS"}:
                self.last_command = msg
            elif msg_type == "MASTER_DECISION":
                self.last_decision = msg
            self._rx_queue.put(msg)

    def _mark_offline(self):
        self.online = False
        self._running = False
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
