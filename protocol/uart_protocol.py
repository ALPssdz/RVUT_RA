# -*- coding: utf-8 -*-
"""
UART protocol helpers for the RA8P1 master link.

Initial transport format: one compact JSON object per line.
The checksum is intentionally simple for bring-up; replace with CRC16 before
field deployment if the line protocol remains in use.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict


DEFAULT_BAUDRATE = 921600
ROLE_AGENT = "RK3588_RF_AGENT"
ROLE_MASTER = "RA8P1_MASTER"


def checksum_payload(payload: Dict[str, Any]) -> int:
    body = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return sum(body.encode("utf-8")) & 0xFFFF


def encode_message(payload: Dict[str, Any]) -> bytes:
    frame = dict(payload)
    frame["checksum"] = checksum_payload(frame)
    return (json.dumps(frame, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


def decode_message(line: bytes | str) -> Dict[str, Any]:
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    frame = json.loads(line.strip())
    received = int(frame.pop("checksum"))
    expected = checksum_payload(frame)
    if received != expected:
        raise ValueError(f"checksum mismatch: got {received}, expected {expected}")
    return frame


@dataclass
class SequenceGenerator:
    value: int = 0

    def next(self) -> int:
        self.value = (self.value + 1) & 0xFFFF
        return self.value


@dataclass
class RA8P1Protocol:
    role: str = ROLE_AGENT
    seq: SequenceGenerator = field(default_factory=SequenceGenerator)

    def heartbeat(self) -> Dict[str, Any]:
        return {
            "type": "HEARTBEAT",
            "seq": self.seq.next(),
            "role": self.role,
            "timestamp_ms": int(time.time() * 1000),
        }

    def detection_report(
        self,
        freq_mhz: float,
        ncc: float,
        sds: float,
        rf_detected: bool,
        suggestion: str,
    ) -> Dict[str, Any]:
        return {
            "type": "DETECTION_REPORT",
            "seq": self.seq.next(),
            "freq_mhz": round(float(freq_mhz), 3),
            "ncc": round(float(ncc), 6),
            "sds": round(float(sds), 6),
            "rf_detected": bool(rf_detected),
            "suggestion": suggestion,
        }

    def master_decision(self, decision: str, reason: str) -> Dict[str, Any]:
        return {
            "type": "MASTER_DECISION",
            "seq": self.seq.next(),
            "decision": decision,
            "reason": reason,
        }

