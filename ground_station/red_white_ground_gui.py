#!/usr/bin/env python3
"""Ground-station GUI for Orin mission-target geolocation."""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np
import socketio as sio_mod

os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.environ.get(
    "QT_PLUGIN_DIR", "/usr/lib/aarch64-linux-gnu/qt5/plugins"
)
os.environ.pop("QT_PLUGIN_PATH", None)

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


ORIN_IP = "10.0.0.8"
ORIN_PORT = 5001
LOG_FILE = Path(__file__).parent / "mission_targets.jsonl"


def wgs84_to_twd97(lat_deg: float, lon_deg: float) -> tuple[float, float]:
    """Convert WGS84 geodetic coordinates to TWD97 TM2 projected easting/northing (metres).

    Uses the GRS80 ellipsoid (TWD97 datum), central meridian 121°E,
    scale factor k0 = 0.9999, and false easting 250 000 m.
    Returns (easting_m, northing_m).
    """
    import math
    a = 6_378_137.0
    f = 1.0 / 298.257222101
    e2 = 2 * f - f * f
    ep2 = e2 / (1 - e2)
    k0 = 0.9999
    lon0 = math.radians(121.0)
    FE = 250_000.0

    lat = math.radians(lat_deg)
    dlon = math.radians(lon_deg) - lon0
    sl = math.sin(lat)

    N = a / math.sqrt(1 - e2 * sl * sl)
    T = math.tan(lat) ** 2
    C = ep2 * math.cos(lat) ** 2
    A = math.cos(lat) * dlon

    e2_2 = e2 * e2
    e2_3 = e2_2 * e2
    M = a * (
        (1 - e2 / 4 - 3 * e2_2 / 64 - 5 * e2_3 / 256) * lat
        - (3 * e2 / 8 + 3 * e2_2 / 32 + 45 * e2_3 / 1024) * math.sin(2 * lat)
        + (15 * e2_2 / 256 + 45 * e2_3 / 1024) * math.sin(4 * lat)
        - (35 * e2_3 / 3072) * math.sin(6 * lat)
    )
    easting = k0 * N * (
        A + (1 - T + C) * A ** 3 / 6
        + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * A ** 5 / 120
    ) + FE
    northing = k0 * (
        M + N * math.tan(lat) * (
            A * A / 2
            + (5 - T + 9 * C + 4 * C * C) * A ** 4 / 24
            + (61 - 58 * T + T * T + 600 * C - 330 * ep2) * A ** 6 / 720
        )
    )
    return easting, northing


def value(data: Dict[str, Any], key: str, default: Any = None) -> Any:
    result = data.get(key, default)
    return default if result is None else result


def number(data: Dict[str, Any], key: str) -> Optional[float]:
    result = data.get(key)
    try:
        return None if result is None else float(result)
    except (TypeError, ValueError):
        return None


def fmt_num(result: Optional[float], digits: int = 1, suffix: str = "") -> str:
    return "—" if result is None else f"{result:.{digits}f}{suffix}"


def fix_name(fix_type: Any) -> str:
    try:
        fix_type = int(fix_type)
    except (TypeError, ValueError):
        fix_type = 0
    return {
        0: "NO FIX",
        1: "NO FIX",
        2: "2D",
        3: "3D",
        4: "DGPS",
        5: "RTK FLOAT",
        6: "RTK FIXED",
    }.get(fix_type, f"FIX {fix_type}")


def location_reason(reason: Any) -> str:
    return {
        "gps_fix_below_gate": "GPS 品質未達精定位門檻",
        "no_aircraft_gps": "無無人機定位資料",
        "invalid_depth_and_no_height_agl": "深度與離地高度不足",
        "camera_ray_does_not_intersect_ground": "相機射線未交會地面",
        "ground_intersection_behind_camera": "地面交會點位於相機後方",
    }.get(str(reason), str(reason or "定位資料不足"))


_log_lock = threading.Lock()


def log_event(event: Dict[str, Any]) -> None:
    with _log_lock:
        with LOG_FILE.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event, ensure_ascii=False) + "\n")


class MJPEGWorker(QObject):
    frame_ready = pyqtSignal(np.ndarray)

    def __init__(self, url: str):
        super().__init__()
        self._url = url
        self._running = threading.Event()

    def start(self) -> None:
        self._running.set()
        threading.Thread(target=self._loop, daemon=True, name="mjpeg").start()

    def stop(self) -> None:
        self._running.clear()

    def _loop(self) -> None:
        while self._running.is_set():
            try:
                with urllib.request.urlopen(self._url, timeout=10) as response:
                    buf = b""
                    while self._running.is_set():
                        chunk = response.read(8192)
                        if not chunk:
                            break
                        buf += chunk
                        while True:
                            start = buf.find(b"\xff\xd8")
                            end = buf.find(b"\xff\xd9", start + 2)
                            if start < 0 or end < 0:
                                break
                            jpg, buf = buf[start:end + 2], buf[end + 2:]
                            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                            if frame is not None:
                                self.frame_ready.emit(frame)
            except Exception:
                time.sleep(2)


class EventWorker(QObject):
    event_received = pyqtSignal(dict)

    def __init__(self, url: str):
        super().__init__()
        self._url = url
        self._running = threading.Event()
        self._sio = sio_mod.Client(
            reconnection=True,
            reconnection_delay=2,
            logger=False,
            engineio_logger=False,
        )

        @self._sio.on("target", namespace="/events")
        def on_target(data):
            self.event_received.emit(data)
            log_event(data)

        @self._sio.on("detection", namespace="/events")
        def on_detection(data):
            self.event_received.emit(data)
            if data.get("type") in {"target_detection", "red_white_target"}:
                log_event(data)

        @self._sio.on("gps", namespace="/events")
        def on_gps(data):
            self.event_received.emit({**data, "type": "gps_update"})

        @self._sio.on("mission_status", namespace="/events")
        def on_mission_status(data):
            self.event_received.emit({**data, "type": "mission_status"})

        @self._sio.event(namespace="/events")
        def connect():
            self.event_received.emit({"type": "__connected__"})

        @self._sio.event(namespace="/events")
        def disconnect():
            self.event_received.emit({"type": "__disconnected__"})

    def start(self) -> None:
        self._running.set()
        threading.Thread(target=self._loop, daemon=True, name="socketio").start()

    def stop(self) -> None:
        self._running.clear()
        try:
            self._sio.disconnect()
        except Exception:
            pass

    def _loop(self) -> None:
        while self._running.is_set():
            try:
                self._sio.connect(
                    self._url,
                    namespaces=["/events"],
                    transports=["polling"],
                    wait_timeout=5,
                )
                self._sio.wait()
            except Exception:
                if self._running.is_set():
                    time.sleep(3)


class StatusWorker(QObject):
    status_received = pyqtSignal(dict)

    def __init__(self, url: str):
        super().__init__()
        self._url = url
        self._running = threading.Event()

    def start(self) -> None:
        self._running.set()
        threading.Thread(target=self._loop, daemon=True, name="status").start()

    def stop(self) -> None:
        self._running.clear()

    def _loop(self) -> None:
        while self._running.is_set():
            try:
                with urllib.request.urlopen(f"{self._url}/status", timeout=2) as response:
                    self.status_received.emit(json.loads(response.read()))
            except Exception:
                pass
            deadline = time.monotonic() + 1.0
            while self._running.is_set() and time.monotonic() < deadline:
                time.sleep(0.1)


class ValueCard(QGroupBox):
    def __init__(self, title: str):
        super().__init__(title)
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(12, 22, 12, 10)
        self._grid.setHorizontalSpacing(14)
        self._grid.setVerticalSpacing(6)
        self._rows: Dict[str, QLabel] = {}

    def add_value(self, key: str, title: str, color: str = "#e7edf8") -> None:
        row = len(self._rows)
        name = QLabel(title)
        name.setObjectName("fieldName")
        label = QLabel("—")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        label.setStyleSheet(f"color:{color}; font-weight:700;")
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._grid.addWidget(name, row, 0)
        self._grid.addWidget(label, row, 1)
        self._rows[key] = label

    def set_value(self, key: str, text: str, color: Optional[str] = None) -> None:
        label = self._rows[key]
        label.setText(text)
        if color:
            label.setStyleSheet(f"color:{color}; font-weight:800;")


class TargetTable(QTableWidget):
    HEADERS = ["目標 ID", "時間", "WGS84 座標", "TWD97 座標 (m)", "水平誤差", "偵測信心"]

    def __init__(self):
        super().__init__(0, len(self.HEADERS))
        self.setHorizontalHeaderLabels(self.HEADERS)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.horizontalHeader().setStretchLastSection(True)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setAlternatingRowColors(True)
        self._ids: set[str] = set()

    def add_target(self, event: Dict[str, Any]) -> None:
        target_id = str(event.get("target_id", "—"))
        if target_id in self._ids:
            return
        self._ids.add(target_id)
        target = event.get("target_location") or {}
        detections = event.get("detections") or []
        det = detections[0] if detections else {}
        lat, lon = number(target, "latitude"), number(target, "longitude")
        wgs84 = "—" if lat is None or lon is None else f"{lat:.7f}, {lon:.7f}"
        tx = number(target, "twd97_x")
        ty = number(target, "twd97_y")
        if tx is None and lat is not None and lon is not None:
            tx, ty = wgs84_to_twd97(lat, lon)
        twd97 = "—" if tx is None else f"E {tx:.2f}, N {ty:.2f}"
        timestamp = str(event.get("timestamp", ""))
        timestamp = timestamp[11:19] if len(timestamp) >= 19 else datetime.now().strftime("%H:%M:%S")
        texts = [
            target_id,
            timestamp,
            wgs84,
            twd97,
            "±" + fmt_num(number(target, "estimated_horizontal_sigma_m"), 2, " m"),
            fmt_num(number(det, "confidence"), 2),
        ]
        self.insertRow(0)
        for col, text in enumerate(texts):
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter)
            if col == 0:
                item.setForeground(QColor("#51e6a8"))
            self.setItem(0, col, item)


class MainWindow(QMainWindow):
    def __init__(self, orin_ip: str, port: int, fullscreen: bool):
        super().__init__()
        self._orin_ip = orin_ip
        self._port = port
        self._url = f"http://{orin_ip}:{port}"
        self._http_connected = False
        self._event_connected = False
        self._fps_count = 0
        self._fps_started = time.monotonic()
        self._last_frame_at = 0.0
        self._build_ui()

        self._mjpeg = MJPEGWorker(f"{self._url}/video")
        self._mjpeg.frame_ready.connect(self._on_frame)
        self._mjpeg.start()

        self._events = EventWorker(self._url)
        self._events.event_received.connect(self._on_event)
        self._events.start()

        self._status = StatusWorker(self._url)
        self._status.status_received.connect(self._on_status)
        self._status.start()

        clock = QTimer(self)
        clock.timeout.connect(self._tick)
        clock.start(1000)
        QTimer.singleShot(0, self.showFullScreen if fullscreen else self.show)

    def _build_ui(self) -> None:
        self.setWindowTitle("任務目標即時定位地面站")
        self.setMinimumSize(1280, 720)
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        left = QVBoxLayout()
        title_row = QHBoxLayout()
        title = QLabel("即時目標識別影像")
        title.setObjectName("mainTitle")
        self._video_state = QLabel("等待影像")
        self._video_state.setObjectName("pill")
        title_row.addWidget(title)
        title_row.addStretch()
        title_row.addWidget(self._video_state)
        left.addLayout(title_row)

        self._video = QLabel("連線至 Orin 10.0.0.8:5001…")
        self._video.setObjectName("video")
        self._video.setAlignment(Qt.AlignCenter)
        self._video.setMinimumSize(760, 430)
        self._video.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left.addWidget(self._video, stretch=1)
        self._note = QLabel("任務目標：搭載視覺基準標記之目標車輛；多幀時序驗證確認後顯示目標 ID、WGS84 及 TWD97 座標")
        self._note.setObjectName("note")
        left.addWidget(self._note)
        root.addLayout(left, stretch=7)

        right = QVBoxLayout()
        header = QHBoxLayout()
        right_title = QLabel("飛控與任務目標資訊")
        right_title.setObjectName("mainTitle")
        self._connection = QLabel("未連線")
        self._connection.setObjectName("connectionBad")
        header.addWidget(right_title)
        header.addStretch()
        header.addWidget(self._connection)
        right.addLayout(header)

        self._flight = ValueCard("無人機飛控")
        for key, label, color in [
            ("position", "無人機 WGS84", "#e7edf8"),
            ("height", "飛行高度 AGL (REL)", "#52dcff"),
            ("rangefinder", "測距儀 / 地形高度", "#e7edf8"),
            ("gps", "GPS 品質", "#ffd166"),
            ("motion", "航向 / 地速", "#e7edf8"),
            ("battery", "電池", "#e7edf8"),
        ]:
            self._flight.add_value(key, label, color)
        right.addWidget(self._flight)

        self._target = ValueCard("最新偵測目標")
        for key, label, color in [
            ("id", "目標 ID", "#51e6a8"),
            ("wgs84", "WGS84 座標", "#ffd166"),
            ("twd97", "TWD97 座標 (m)", "#ffd166"),
            ("accuracy", "估計水平誤差", "#ffd166"),
            ("range", "斜距 / 高度來源", "#e7edf8"),
            ("confidence", "偵測信心值", "#51e6a8"),
            ("vehicle_confidence", "車子模型分數", "#51e6a8"),
            ("white_vehicle_ratio", "車體白色比例", "#51e6a8"),
            ("roof_red_ratio", "車頂紅色比例", "#ff6b6b"),
            ("rtl", "自動返航狀態", "#ffd166"),
            ("status", "事件發送狀態", "#51e6a8"),
        ]:
            self._target.add_value(key, label, color)
        right.addWidget(self._target)

        history = QGroupBox("任務目標紀錄")
        history_layout = QVBoxLayout(history)
        self._history = TargetTable()
        history_layout.addWidget(self._history)
        right.addWidget(history, stretch=1)
        root.addLayout(right, stretch=4)

        self._status_text = QLabel(f"Orin {self._orin_ip}:{self._port}")
        self._fps_text = QLabel("FPS —")
        self._clock = QLabel()
        self.statusBar().addWidget(self._status_text)
        self.statusBar().addPermanentWidget(self._fps_text)
        self.statusBar().addPermanentWidget(self._clock)
        self._apply_theme()

    def _apply_theme(self) -> None:
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background:#07101d; color:#dbe7f7;
                font-family:"Noto Sans CJK TC","Microsoft JhengHei",sans-serif;
                font-size:16px;
            }
            QLabel#mainTitle { font-size:26px; font-weight:800; color:#f4f8ff; }
            QLabel#video {
                background:#02050a; border:1px solid #293a58; border-radius:8px;
                color:#91a4c4; font-size:22px;
            }
            QLabel#pill, QLabel#connectionBad, QLabel#connectionWarn, QLabel#connectionGood {
                padding:5px 11px; border-radius:10px; font-weight:800;
            }
            QLabel#pill { background:#17243a; color:#8fa8ca; }
            QLabel#connectionBad { background:#3a1720; color:#ff8598; }
            QLabel#connectionWarn { background:#3b3216; color:#ffd166; }
            QLabel#connectionGood { background:#123629; color:#51e6a8; }
            QLabel#note, QLabel#fieldName { color:#8295b3; }
            QGroupBox {
                background:#0d1829; border:1px solid #253653; border-radius:7px;
                margin-top:8px; font-weight:800; color:#91a9cc;
            }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; }
            QTableWidget {
                background:#091321; alternate-background-color:#0d1a2d;
                color:#d7e2f1; gridline-color:#263955; border:0;
            }
            QHeaderView::section {
                background:#14233a; color:#9eb4d3; border:0;
                border-right:1px solid #263955; padding:6px; font-weight:800;
            }
            QStatusBar { background:#09111f; color:#91a4c4; border-top:1px solid #253653; }
        """)

    def _on_frame(self, frame: np.ndarray) -> None:
        self._last_frame_at = time.monotonic()
        self._fps_count += 1
        elapsed = time.monotonic() - self._fps_started
        if elapsed >= 1.0:
            self._fps_text.setText(f"FPS {self._fps_count / elapsed:.1f}")
            self._fps_count = 0
            self._fps_started = time.monotonic()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image).scaled(
            max(self._video.width() - 2, 10),
            max(self._video.height() - 2, 10),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self._video.setPixmap(pixmap)
        self._video_state.setText("影像正常")

    def _on_event(self, event: Dict[str, Any]) -> None:
        event_type = event.get("type", "")
        if event_type == "__connected__":
            self._event_connected = True
            self._refresh_connection()
        elif event_type == "__disconnected__":
            self._event_connected = False
            self._refresh_connection()
        elif event_type == "gps_update":
            self._update_flight(event)
        elif event_type == "mission_status":
            self._update_mission(event)
        elif event_type in {"target_detection", "red_white_target", "target"} or event.get("target_id"):
            self._update_target(event)

    def _on_status(self, status: Dict[str, Any]) -> None:
        self._http_connected = True
        self._refresh_connection()
        gps = status.get("gps")
        if gps:
            self._update_flight(gps)
        target = status.get("last_detection")
        if target:
            self._update_target(target)
        mission = status.get("mission")
        if mission:
            self._update_mission(mission)

    def _refresh_connection(self) -> None:
        if self._http_connected and self._event_connected:
            text, style, detail = "Orin 完整連線", "connectionGood", "影像／事件正常"
        elif self._http_connected:
            text, style, detail = "僅影像連線", "connectionWarn", "事件通道重連中"
        else:
            text, style, detail = "Orin 未連線", "connectionBad", "重新連線中"
        self._connection.setText(text)
        self._connection.setObjectName(style)
        self._connection.style().unpolish(self._connection)
        self._connection.style().polish(self._connection)
        self._status_text.setText(f"Orin {self._orin_ip}:{self._port}　{detail}")

    def _update_flight(self, flight: Dict[str, Any]) -> None:
        lat, lon = number(flight, "latitude"), number(flight, "longitude")
        position = "—" if lat is None or lon is None else f"{lat:.7f}, {lon:.7f}"
        self._flight.set_value("position", position)
        relative_altitude = number(flight, "relative_altitude")
        self._flight.set_value("height", fmt_num(relative_altitude, 1, " m"))
        rangefinder = number(flight, "rangefinder_m")
        bottom_clearance = number(flight, "bottom_clearance")
        range_text = (
            f"{fmt_num(rangefinder, 1, ' m')} / {fmt_num(bottom_clearance, 1, ' m')}"
        )
        self._flight.set_value("rangefinder", range_text)
        gps_text = f"{fix_name(flight.get('fix_type'))} / {int(value(flight, 'satellites', 0))} 星 / HDOP {fmt_num(number(flight, 'hdop'), 1)}"
        gps_color = "#51e6a8" if int(value(flight, "fix_type", 0)) >= 5 else "#ffd166"
        self._flight.set_value("gps", gps_text, gps_color)
        self._flight.set_value("motion", f"{fmt_num(number(flight, 'heading'), 0, '°')} / {fmt_num(number(flight, 'groundspeed'), 1, ' m/s')}")
        self._flight.set_value("battery", f"{fmt_num(number(flight, 'battery_remaining_pct'), 0, '%')} / {fmt_num(number(flight, 'voltage_v'), 1, ' V')}")

    def _rtl_text(self, auto_rtl: Dict[str, Any]) -> tuple[str, str]:
        if not auto_rtl:
            return "等待任務目標", "#ffd166"
        if auto_rtl.get("mode_confirmed"):
            return "已偵測目標，飛控已回報 RTL", "#51e6a8"
        if auto_rtl.get("ack_accepted"):
            return "已偵測目標，飛控已接受返航指令", "#51e6a8"
        if auto_rtl.get("sent"):
            reason = str(auto_rtl.get("reason") or "")
            if reason == "already_sent":
                return "已偵測目標，自動返航已送出", "#51e6a8"
            if reason == "sent_waiting_confirmation":
                return "已偵測目標，等待飛控確認返航", "#ffd166"
            return "已偵測目標，正在送出自動返航", "#ffd166"
        if auto_rtl.get("enabled") is False:
            return "自動返航未啟用", "#ff8598"
        return str(auto_rtl.get("reason") or "等待自動返航"), "#ffd166"

    def _update_mission(self, mission: Dict[str, Any]) -> None:
        auto_rtl = mission.get("auto_rtl") or {}
        text, color = self._rtl_text(auto_rtl)
        message = mission.get("message")
        if message:
            text = str(message)
        self._target.set_value("rtl", text, color)
        if mission.get("locked") and mission.get("target_id"):
            self._target.set_value("status", f"任務已鎖定：{mission.get('target_id')}，後續誤偵不再新增 ID", "#51e6a8")

    def _update_target(self, event: Dict[str, Any]) -> None:
        target_id = str(event.get("target_id", "—"))
        target = event.get("target_location") or {}
        detections = event.get("detections") or []
        det = detections[0] if detections else {}
        self._target.set_value("id", target_id)
        lat, lon = number(target, "latitude"), number(target, "longitude")
        has_position = lat is not None and lon is not None
        if target.get("valid") or has_position:
            prefix = "" if target.get("valid") else "暫估 "
            self._target.set_value(
                "wgs84",
                f"{prefix}{lat:.7f}, {lon:.7f}" if has_position else "—",
            )
            tx = number(target, "twd97_x")
            ty = number(target, "twd97_y")
            if tx is None and lat is not None and lon is not None:
                tx, ty = wgs84_to_twd97(lat, lon)
            self._target.set_value(
                "twd97",
                f"{prefix}E {tx:.2f},  N {ty:.2f}" if tx is not None else "—",
            )
            reason = str(value(target, "reason", ""))
            accuracy = "±" + fmt_num(number(target, "estimated_horizontal_sigma_m"), 2, " m")
            if not target.get("valid") and reason:
                accuracy = f"{accuracy}（{location_reason(reason)}）"
            self._target.set_value("accuracy", accuracy)
        else:
            self._target.set_value("wgs84", "定位失效")
            self._target.set_value("twd97", "—")
            self._target.set_value("accuracy", location_reason(target.get("reason")))
        self._target.set_value("range", f"{fmt_num(number(target, 'slant_range_m'), 1, ' m')} / {value(target, 'height_source', '—')}")
        vehicle_detector = det.get("vehicle_detector") or {}
        self._target.set_value("confidence", fmt_num(number(det, "confidence"), 2))
        self._target.set_value("vehicle_confidence", fmt_num(number(vehicle_detector, "confidence"), 2))
        self._target.set_value("white_vehicle_ratio", fmt_num(number(det, "white_vehicle_ratio"), 2))
        self._target.set_value("roof_red_ratio", fmt_num(number(det, "red_ratio"), 3))
        text, color = self._rtl_text(event.get("auto_rtl") or {})
        self._target.set_value("rtl", text, color)
        self._target.set_value("status", "任務目標已發送；系統鎖定第一個任務目標")
        self._history.add_target(event)

    def _tick(self) -> None:
        self._clock.setText(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        if self._last_frame_at and time.monotonic() - self._last_frame_at > 5:
            self._video_state.setText("影像逾時")

    def closeEvent(self, event) -> None:
        self._mjpeg.stop()
        self._events.stop()
        self._status.stop()
        event.accept()


def main() -> None:
    parser = argparse.ArgumentParser(description="Mission-target ground station")
    parser.add_argument("--orin-ip", default=ORIN_IP)
    parser.add_argument("--port", type=int, default=ORIN_PORT)
    parser.add_argument("--fullscreen", action="store_true")
    args = parser.parse_args()
    app = QApplication(sys.argv)
    app.setApplicationName("任務目標即時定位地面站")
    window = MainWindow(args.orin_ip, args.port, args.fullscreen)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
