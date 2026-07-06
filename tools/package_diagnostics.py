#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import os
import tarfile
from datetime import datetime


PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ROOT = os.path.join(PROJ_ROOT, "diagnostics", "captures")


def find_latest_session(root: str) -> str:
    if not os.path.isdir(root):
        raise FileNotFoundError(f"diagnostics root not found: {root}")
    sessions = [
        os.path.join(root, name)
        for name in os.listdir(root)
        if name.startswith("session_") and os.path.isdir(os.path.join(root, name))
    ]
    if not sessions:
        raise FileNotFoundError(f"no session_* directory under: {root}")
    return max(sessions, key=os.path.getmtime)


def main() -> int:
    parser = argparse.ArgumentParser(description="Package latest diagnostics session for SCP transfer.")
    parser.add_argument("--root", default=DEFAULT_ROOT, help="diagnostics capture root")
    parser.add_argument("--session", default="", help="specific session directory; default: latest")
    parser.add_argument("--out", default="", help="output .tar.gz path")
    args = parser.parse_args()

    session_dir = os.path.abspath(args.session) if args.session else find_latest_session(args.root)
    if not os.path.isdir(session_dir):
        raise FileNotFoundError(session_dir)

    if args.out:
        out_path = os.path.abspath(args.out)
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(PROJ_ROOT, "diagnostics", f"{os.path.basename(session_dir)}_{stamp}.tar.gz")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with tarfile.open(out_path, "w:gz") as tar:
        tar.add(session_dir, arcname=os.path.basename(session_dir))

    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
