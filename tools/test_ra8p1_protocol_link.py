#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    serial = None

PROJ_ROOT = os.path.dirname(os.path.dirname(__file__))
if PROJ_ROOT not in sys.path:
    sys.path.insert(0, PROJ_ROOT)

from protocol.link_protocol import DEFAULT_BAUDRATE, RA8P1Protocol, decode_message, encode_message


def print_rx(line: bytes) -> None:
    text = line.decode("utf-8", errors="replace").rstrip()
    try:
        msg = decode_message(text)
        print("RX JSON:", msg)
    except Exception:
        print("RX RAW :", text)


def main() -> int:
    parser = argparse.ArgumentParser(description="Test RA8P1 JSON line protocol over JDBG UART.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    if serial is None:
        print("pyserial is not installed. Install python3-serial or pyserial.", file=sys.stderr)
        return 2

    proto = RA8P1Protocol()

    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1.0) as ser:
        print(f"opened {args.port} @ {args.baud}")
        ser.write(encode_message(proto.agent_ready()))
        ser.flush()
        print("TX: AGENT_READY")

        next_tx = 0.0
        seq = 0
        while True:
            line = ser.readline()
            if line:
                print_rx(line)

            now = time.monotonic()
            if now >= next_tx:
                seq += 1
                ser.write(encode_message(proto.heartbeat()))
                report = proto.detection_report(
                    freq_mhz=5785.0,
                    ncc=0.034,
                    sds=1.18 if seq % 3 else 0.42,
                    rf_detected=(seq % 3 != 0),
                    suggestion="ALERT" if seq % 3 != 0 else "CLEAR",
                )
                ser.write(encode_message(report))
                ser.flush()
                print(f"TX: HEARTBEAT + DETECTION_REPORT seq={seq}")
                next_tx = now + args.interval


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
        raise SystemExit(0)
