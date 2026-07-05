#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal RA8P1 master mock for protocol bring-up.

It demonstrates the intended control direction: RA8P1 sends scan commands and
returns MASTER_DECISION messages after receiving DETECTION_REPORT payloads.
"""

import argparse
import sys

PROJ_ROOT = __import__("os").path.dirname(__import__("os").path.dirname(__file__))
if PROJ_ROOT not in sys.path:
    sys.path.insert(0, PROJ_ROOT)

from protocol.uart_protocol import RA8P1Protocol, ROLE_MASTER, decode_message, encode_message


def decide(report: dict) -> str:
    if report.get("rf_detected") and float(report.get("sds", 0.0)) >= 1.0:
        return "ALERT"
    if report.get("rf_detected"):
        return "CANDIDATE"
    return "CLEAR"


def main() -> int:
    parser = argparse.ArgumentParser(description="RA8P1 master protocol mock")
    parser.add_argument("--stdin", action="store_true", help="read JSON lines from stdin")
    args = parser.parse_args()

    proto = RA8P1Protocol(role=ROLE_MASTER)
    sys.stdout.buffer.write(encode_message({"type": "START_SCAN", "seq": proto.seq.next()}))
    sys.stdout.flush()

    if not args.stdin:
        return 0

    for line in sys.stdin.buffer:
        try:
            msg = decode_message(line)
        except Exception as exc:
            sys.stderr.write(f"decode error: {exc}\n")
            continue
        if msg.get("type") == "DETECTION_REPORT":
            decision = decide(msg)
            response = proto.master_decision(decision, f"MOCK_{decision}")
            sys.stdout.buffer.write(encode_message(response))
            sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
