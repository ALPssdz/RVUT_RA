#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None


def main() -> int:
    parser = argparse.ArgumentParser(description="Test RA8P1 JDBG UART9 echo firmware.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="JDBG VCOM device path, default: /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=2000000, help="UART baud setting, default: 2000000")
    parser.add_argument("--interval", type=float, default=2.0, help="TX interval seconds, default: 2.0")
    args = parser.parse_args()

    if serial is None:
        print("pyserial is not installed. Run: python3 -m pip install pyserial", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=1.0) as ser:
        print(f"opened {args.port}, press Ctrl+C to stop")
        next_tx = 0.0
        seq = 0

        while True:
            line = ser.readline()
            if line:
                print("RX:", line.decode("utf-8", errors="replace").rstrip())

            now = time.monotonic()
            if now >= next_tx:
                seq += 1
                payload = f"HOST_PING seq={seq} time={time.time():.3f}\n"
                ser.write(payload.encode("utf-8"))
                ser.flush()
                print("TX:", payload.rstrip())
                next_tx = now + args.interval


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
        raise SystemExit(0)
