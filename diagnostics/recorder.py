# -*- coding: utf-8 -*-
"""
Runtime diagnostics recorder.

The recorder writes a self-contained capture session that can be copied back
from the Orange Pi with scp for false-positive analysis.
"""

from __future__ import annotations

import json
import os
import queue
import shutil
import threading
import time
from datetime import datetime
from typing import Any, Optional

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - target normally has OpenCV
    cv2 = None


class DiagnosticsRecorder:
    def __init__(
        self,
        root_dir: str,
        enabled: bool = True,
        save_iq: bool = True,
        max_iq_samples: int = 2621440,
        max_event_records: int = 500,
        max_root_bytes: int = 0,
    ):
        self.enabled = bool(enabled)
        self.save_iq = bool(save_iq)
        self.max_iq_samples = int(max_iq_samples)
        self.max_event_records = int(max_event_records)
        self.max_root_bytes = int(max_root_bytes)
        self.root_dir = os.path.abspath(root_dir)
        self._lock = threading.Lock()
        self._event_count = 0
        self._storage_limit_warned = False
        self._queue: "queue.Queue[tuple]" = queue.Queue(maxsize=16)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.root_dir, f"session_{timestamp}")
        self.frames_dir = os.path.join(self.session_dir, "frames")
        self.iq_dir = os.path.join(self.session_dir, "iq")
        self.events_path = os.path.join(self.session_dir, "events.jsonl")
        self.log_path = os.path.join(self.session_dir, "runtime.log")

        if self.enabled:
            os.makedirs(self.root_dir, exist_ok=True)
            self._enforce_root_limit(protected_session=None)
            os.makedirs(self.frames_dir, exist_ok=True)
            if self.save_iq:
                os.makedirs(self.iq_dir, exist_ok=True)
            self._worker = threading.Thread(target=self._write_loop, daemon=True)
            self._worker.start()

    def log_text(self, text: str) -> None:
        if not self.enabled:
            return
        if not self._enforce_root_limit(protected_session=self.session_dir):
            return
        line = str(text).rstrip()
        if not line:
            return
        record = f"{datetime.now().isoformat(timespec='milliseconds')} {line}\n"
        with self._lock:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(record)

    def record_event(
        self,
        event_type: str,
        metadata: dict[str, Any],
        frame_bgr: Optional[np.ndarray] = None,
        iq_buffer: Optional[Any] = None,
    ) -> None:
        if not self.enabled:
            return
        try:
            self._queue.put_nowait((event_type, metadata, frame_bgr, iq_buffer))
        except queue.Full:
            self.log_text(f"[DiagnosticsRecorder] drop event because writer queue is full: {event_type}")

    def _write_loop(self) -> None:
        while True:
            event_type, metadata, frame_bgr, iq_buffer = self._queue.get()
            try:
                self._write_event(event_type, metadata, frame_bgr, iq_buffer)
            except Exception as exc:
                self.log_text(f"[DiagnosticsRecorder] write event failed: {exc}")
            finally:
                self._queue.task_done()

    def _write_event(
        self,
        event_type: str,
        metadata: dict[str, Any],
        frame_bgr: Optional[np.ndarray] = None,
        iq_buffer: Optional[Any] = None,
    ) -> None:
        if not self.enabled:
            return
        if not self._enforce_root_limit(protected_session=self.session_dir):
            if not self._storage_limit_warned:
                self._storage_limit_warned = True
                self.log_text(
                    "[DiagnosticsRecorder] diagnostics capture root is over size limit; "
                    "skip new diagnostic events until old sessions are removed"
                )
            return

        with self._lock:
            if self._event_count >= self.max_event_records:
                return
            self._event_count += 1
            event_id = self._event_count

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        ms = int((time.time() % 1.0) * 1000)
        prefix = f"{event_id:05d}_{stamp}_{ms:03d}_{self._safe_name(event_type)}"

        frame_path = None
        if frame_bgr is not None and cv2 is not None:
            frame_path = os.path.join(self.frames_dir, f"{prefix}.jpg")
            try:
                cv2.imwrite(frame_path, frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
            except Exception as exc:
                frame_path = f"<frame_write_failed:{exc}>"

        iq_path = None
        if self.save_iq and iq_buffer is not None:
            iq_path = os.path.join(self.iq_dir, f"{prefix}.npz")
            try:
                iq = np.asarray(iq_buffer)
                if self.max_iq_samples > 0 and iq.size > self.max_iq_samples:
                    iq = iq[: self.max_iq_samples]
                np.savez_compressed(iq_path, iq=iq)
            except Exception as exc:
                iq_path = f"<iq_write_failed:{exc}>"

        event = {
            "event_id": event_id,
            "event_type": event_type,
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "frame_path": frame_path,
            "iq_path": iq_path,
            "metadata": metadata,
        }

        with self._lock:
            with open(self.events_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(event, ensure_ascii=False, default=self._json_default) + "\n")
        self._enforce_root_limit(protected_session=self.session_dir)

    def _enforce_root_limit(self, protected_session: Optional[str]) -> bool:
        if self.max_root_bytes <= 0 or not os.path.isdir(self.root_dir):
            return True

        total = self._directory_size(self.root_dir)
        if total <= self.max_root_bytes:
            return True

        protected = os.path.abspath(protected_session) if protected_session else None
        for session in self._old_session_dirs(protected):
            try:
                shutil.rmtree(session)
            except OSError:
                continue
            total = self._directory_size(self.root_dir)
            if total <= self.max_root_bytes:
                return True

        return self._directory_size(self.root_dir) <= self.max_root_bytes

    def _old_session_dirs(self, protected_session: Optional[str]) -> list[str]:
        sessions = []
        try:
            names = os.listdir(self.root_dir)
        except OSError:
            return sessions

        for name in names:
            path = os.path.abspath(os.path.join(self.root_dir, name))
            if protected_session and path == protected_session:
                continue
            if not name.startswith("session_") or not os.path.isdir(path):
                continue
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                mtime = 0.0
            sessions.append((mtime, path))

        sessions.sort(key=lambda item: item[0])
        return [path for _, path in sessions]

    @staticmethod
    def _directory_size(path: str) -> int:
        total = 0
        for dirpath, _, filenames in os.walk(path):
            for filename in filenames:
                file_path = os.path.join(dirpath, filename)
                try:
                    total += os.path.getsize(file_path)
                except OSError:
                    continue
        return total

    @staticmethod
    def _safe_name(value: str) -> str:
        return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in value)[:80]

    @staticmethod
    def _json_default(value: Any) -> Any:
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
        return str(value)
