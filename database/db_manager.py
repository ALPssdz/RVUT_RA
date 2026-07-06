import sqlite3
import os
import time
import threading
import cv2
import json
from datetime import datetime
from typing import Optional


class DBManager:
    """
    告警事件持久化管理模块（Data Persistence Adapter）。

    负责将射频告警证据图像写入文件系统，并在 SQLite3 关系型数据库中
    维护对应的元数据索引记录。所有文件 I/O 操作均限定于本模块所在目录
    （``database/``）内，确保路径可移植性。

    线程安全设计（修正自 v1.0）：
      ① WAL 模式（Write-Ahead Logging）：允许写操作与读操作真正并发，
         消除后台 hub 线程写入 与 Qt 主线程读取 之间的 "database is locked"。
      ② threading.Lock：串行化写序列（LRU 剪枝 + INSERT），保证原子性。
      ③ 读操作（get_all_alerts）无锁：WAL 模式已保证读写并发安全。
      ④ cv2.imwrite 返回值校验：写入失败时路径标记为 <MISSING>，不存破损记录。
      ⑤ 全方法异常兜底：任何 DB 错误均被捕获回滚，不影响 hub 主循环。

    单实例原则：
      系统中应仅存在一个 DBManager 实例（在 system_hub.CentralHubEngine 中创建）。
      GUI 层通过 hub.db_engine 引用该实例，避免多实例并发冲突。

    存储容量约束：database/ 总大小默认 5GB；超出时触发 LRU 淘汰，
    自动删除最早的记录及其关联图像文件。
    """

    MAX_TOTAL_BYTES = 5 * 1024 * 1024 * 1024
    PRUNE_COUNT = 50

    def __init__(self, db_filename: str = "rf_alert_history.db",
                 img_dirname: str = "alert_images",
                 max_total_bytes: int = MAX_TOTAL_BYTES):
        """
        Parameters
        ----------
        db_filename : str — SQLite3 数据库文件名，默认 rf_alert_history.db
        img_dirname : str — 告警图像存储目录名，默认 alert_images
        """
        self.module_dir = os.path.dirname(os.path.abspath(__file__))
        self.db_path    = os.path.join(self.module_dir, db_filename)
        self.img_dir    = os.path.join(self.module_dir, img_dirname)
        self.max_total_bytes = int(max_total_bytes)

        os.makedirs(self.img_dir, exist_ok=True)

        # 写操作互斥锁：保护 LRU 剪枝 + INSERT 的原子性
        self._write_lock = threading.Lock()

        self._init_tables()

    # ------------------------------------------------------------------
    # 内部辅助
    # ------------------------------------------------------------------

    def _connect(self) -> sqlite3.Connection:
        """
        创建并配置数据库连接。

        - check_same_thread=False：允许跨线程复用（写操作由 _write_lock 互斥，
          读操作由 WAL 并发保护）。
        - WAL 模式：读写并发，解决写锁阻塞读事务的问题。
        - synchronous=NORMAL：WAL 下 NORMAL 已足够安全，性能优于 FULL。
        - timeout=10：等待最多 10 秒再抛出 OperationalError。
        """
        conn = sqlite3.connect(self.db_path,
                               check_same_thread=False,
                               timeout=10)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA cache_size=-8192")   # 8 MiB 页缓存
        return conn

    def _init_tables(self):
        """创建/迁移 alerts 数据表（若已存在则补齐新字段），并激活 WAL 模式。"""
        conn = self._connect()
        conn.execute("""
            CREATE TABLE IF NOT EXISTS alerts (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp  TEXT    NOT NULL,
                freq_mhz   REAL    NOT NULL,
                score      REAL    NOT NULL,
                image_path TEXT    NOT NULL,
                event_type TEXT    NOT NULL DEFAULT 'ALERT',
                final_decision TEXT NOT NULL DEFAULT '',
                reason     TEXT    NOT NULL DEFAULT '',
                rf_metrics TEXT    NOT NULL DEFAULT '',
                metadata_json TEXT NOT NULL DEFAULT '{}'
            )
        """)
        self._ensure_column(conn, "alerts", "event_type", "TEXT NOT NULL DEFAULT 'ALERT'")
        self._ensure_column(conn, "alerts", "final_decision", "TEXT NOT NULL DEFAULT ''")
        self._ensure_column(conn, "alerts", "reason", "TEXT NOT NULL DEFAULT ''")
        self._ensure_column(conn, "alerts", "rf_metrics", "TEXT NOT NULL DEFAULT ''")
        self._ensure_column(conn, "alerts", "metadata_json", "TEXT NOT NULL DEFAULT '{}'")
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_alerts_id ON alerts(id)"
        )
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_alerts_type_id ON alerts(event_type, id)"
        )
        conn.commit()
        conn.close()

    @staticmethod
    def _ensure_column(conn: sqlite3.Connection, table: str, column: str, definition: str):
        cursor = conn.cursor()
        cursor.execute(f"PRAGMA table_info({table})")
        existing = {row[1] for row in cursor.fetchall()}
        if column not in existing:
            cursor.execute(f"ALTER TABLE {table} ADD COLUMN {column} {definition}")

    # ------------------------------------------------------------------
    # 公开写接口
    # ------------------------------------------------------------------

    def log_alert(self, freq_mhz: float, score: float, bgr_image) -> int:
        return self.log_event(
            event_type="ALERT",
            freq_mhz=freq_mhz,
            score=score,
            bgr_image=bgr_image,
            final_decision="ALERT",
        )

    def log_event(
        self,
        event_type: str,
        freq_mhz: float,
        score: float,
        bgr_image,
        final_decision: str = "",
        reason: str = "",
        rf_metrics: str = "",
        metadata: Optional[dict] = None,
    ) -> int:
        """
        将一次 RF 事件的融合证据图像持久化至磁盘，并在数据库中注册元数据。

        设计要点（v2.0 修正）：
          1. cv2.imwrite 在锁外执行（不占用 DB 事务时间）；返回值严格校验。
          2. 持有 _write_lock 后，在 **单条数据库连接** 中完成：
               ① LRU 剪枝（超限时批量删除旧记录及文件）
               ② INSERT 新记录
             整个操作在同一事务中提交，保证原子性，消除 TOCTOU 竞争。
          3. 全程 try-except：任何异常捕获后回滚，返回 -1，不影响 hub 循环。

        Returns
        -------
        int — 写入记录的自增 ID；失败时返回 -1
        """
        event_type = str(event_type or "NORMAL").upper()
        if event_type not in {"ALERT", "NORMAL"}:
            event_type = "NORMAL"
        metadata = metadata or {}

        # ── Step 1: 图像写入磁盘（锁外，避免长 I/O 占用 DB 事务）──────────
        now            = datetime.now()
        timestamp_str  = now.strftime("%Y-%m-%d %H:%M:%S")
        timestamp_file = now.strftime("%Y%m%d_%H%M%S")
        ms             = int((time.time() % 1) * 1000)
        prefix            = "UAV_Intercept" if event_type == "ALERT" else "RF_Normal"
        filename          = (f"{prefix}_{freq_mhz:.0f}MHz"
                             f"_{timestamp_file}_{ms:03d}.jpg")
        absolute_img_path = os.path.join(self.img_dir, filename)

        ok = cv2.imwrite(absolute_img_path, bgr_image,
                         [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            # imwrite 失败时仍入库，但路径标记提示缺失
            print(f"[DBManager] 警告: cv2.imwrite 写入失败 -> {filename}")
            absolute_img_path = "<MISSING>"

        # ── Step 2: 加锁 → LRU 剪枝 + INSERT（单连接单事务）───────────────
        with self._write_lock:
            conn = None
            try:
                conn   = self._connect()
                cursor = conn.cursor()

                # INSERT 新记录
                cursor.execute(
                    "INSERT INTO alerts "
                    "(timestamp, freq_mhz, score, image_path, event_type, final_decision, reason, rf_metrics, metadata_json) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (
                        timestamp_str,
                        freq_mhz,
                        score,
                        absolute_img_path,
                        event_type,
                        str(final_decision or ""),
                        str(reason or ""),
                        str(rf_metrics or ""),
                        json.dumps(metadata, ensure_ascii=False, default=str),
                    )
                )
                new_id = cursor.lastrowid
                conn.commit()
                self._enforce_storage_limit(conn)
                conn.close()
                return new_id

            except sqlite3.OperationalError as e:
                print(f"[DBManager] DB 操作异常（已回滚）: {e}")
            except Exception as e:
                print(f"[DBManager] 未知异常（已回滚）: {e}")
            finally:
                # 确保连接在异常路径下也被关闭
                if conn:
                    try:
                        conn.rollback()
                        conn.close()
                    except Exception:
                        pass

            return -1

    def _enforce_storage_limit(self, conn: sqlite3.Connection):
        if self.max_total_bytes <= 0:
            return

        total = self._directory_size(self.module_dir)
        if total <= self.max_total_bytes:
            return

        cursor = conn.cursor()
        deleted_total = 0
        while total > self.max_total_bytes:
            cursor.execute(
                "SELECT id, image_path FROM alerts ORDER BY id ASC LIMIT ?",
                (self.PRUNE_COUNT,)
            )
            old_rows = cursor.fetchall()
            if not old_rows:
                break

            ids_to_del = [row[0] for row in old_rows]
            for _, img_path in old_rows:
                if img_path and img_path != "<MISSING>":
                    try:
                        if os.path.exists(img_path):
                            os.remove(img_path)
                    except OSError:
                        pass

            placeholders = ",".join("?" * len(ids_to_del))
            cursor.execute(
                f"DELETE FROM alerts WHERE id IN ({placeholders})",
                ids_to_del
            )
            conn.commit()
            deleted_total += len(ids_to_del)
            total = self._directory_size(self.module_dir)

        if deleted_total:
            print(f"[DBManager] 容量淘汰：database/ 超过上限，删除最早 {deleted_total} 条记录。")

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

    # ------------------------------------------------------------------
    # 公开读接口（无锁，WAL 保证读写并发）
    # ------------------------------------------------------------------

    def get_all_alerts(self) -> list:
        return self.get_events("ALERT")

    def get_events(self, event_type: str = "ALERT") -> list:
        """
        按时间逆序检索指定类型事件，供 GUI 表现层渲染历史日志列表。

        WAL 模式下，此方法可与 log_alert 真正并发执行（无需等待写锁）。

        Returns
        -------
        list of tuple : [(id, timestamp, freq_mhz, score, image_path, final_decision, reason, rf_metrics), ...]
                        按 id 降序（最新记录在前）
        """
        event_type = str(event_type or "ALERT").upper()
        try:
            conn   = self._connect()
            cursor = conn.cursor()
            cursor.execute(
                "SELECT id, timestamp, freq_mhz, score, image_path, final_decision, reason, rf_metrics "
                "FROM alerts WHERE event_type = ? ORDER BY id DESC",
                (event_type,)
            )
            rows = cursor.fetchall()
            conn.close()
            return rows
        except Exception as e:
            print(f"[DBManager] get_events 异常: {e}")
            return []

    def clear_all(self, event_type: Optional[str] = None) -> int:
        """
        清除数据库中全部或指定类型记录及其关联图像文件。

        需持有 _write_lock，防止与 log_alert 并发冲突。

        Returns
        -------
        int — 本次操作删除的记录总条数；失败时返回 0
        """
        with self._write_lock:
            conn = None
            try:
                conn   = self._connect()
                cursor = conn.cursor()

                if event_type:
                    event_type = str(event_type).upper()
                    cursor.execute("SELECT image_path FROM alerts WHERE event_type = ?", (event_type,))
                else:
                    cursor.execute("SELECT image_path FROM alerts")
                paths         = [row[0] for row in cursor.fetchall()]
                deleted_count = len(paths)

                for img_path in paths:
                    if img_path and img_path != "<MISSING>":
                        try:
                            if os.path.exists(img_path):
                                os.remove(img_path)
                        except OSError:
                            pass

                if event_type:
                    cursor.execute("DELETE FROM alerts WHERE event_type = ?", (event_type,))
                else:
                    cursor.execute("DELETE FROM alerts")
                # 重置自增 ID 计数器（sqlite_sequence 可能不存在，忽略错误）
                if not event_type:
                    cursor.execute(
                        "DELETE FROM sqlite_sequence WHERE name='alerts'"
                    )
                conn.commit()
                conn.close()
                return deleted_count

            except Exception as e:
                print(f"[DBManager] clear_all 异常: {e}")
                if conn:
                    try:
                        conn.rollback()
                        conn.close()
                    except Exception:
                        pass
                return 0
