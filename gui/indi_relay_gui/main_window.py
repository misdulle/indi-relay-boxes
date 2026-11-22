"""PyQt5 user interface for the observatory relay / roof manager."""

import time

import math

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import (QBrush, QColor, QImage, QKeySequence, QPainter, QPen,
                         QPixmap, QPolygonF)
from PyQt5.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QShortcut,
    QSplitter, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from .attitude import (alt_az, angle_between, format_dec, format_ra,
                       optical_axis, quat_angle)
from . import mount3d
from .controller import Controller, Mode
from .indi import IndiClient
from .model import IndiModel

IPSTATE_COLORS = {
    "IPS_OK": "#2e8b57",
    "IPS_BUSY": "#d0a020",
    "IPS_ALERT": "#c0392b",
    "IPS_IDLE": "#7f8c8d",
}
LIGHT_COLORS = {
    "IPS_OK": "#2e8b57",
    "IPS_BUSY": "#d0a020",
    "IPS_ALERT": "#c0392b",
    "IPS_IDLE": "#7f8c8d",
}
SAFETY_LABELS = (
    ("MOUNT_PARKED", "Mount parked"),
    ("IMU_SAFE", "Scope safe (IMU)"),
    ("MOTOR_POWER", "Motor power"),
    ("MOVE_ALLOWED", "Move allowed"),
)
MODE_COLORS = {
    Mode.INACTIVE: "#7f8c8d",
    Mode.STATIC: "#2980b9",
    Mode.STARTING: "#d0a020",
    Mode.RUNNING: "#2e8b57",
    Mode.STOPPING: "#d35400",
}
ROOF_COLORS = {"open": "#2e8b57", "closed": "#2980b9", "moving": "#d0a020", "error": "#c0392b"}


def _button_stylesheet(background, extra=""):
    return "QPushButton { background-color: %s; color: white; font-weight: bold; %s }" % (
        background, extra)


class _ClientProxy:
    """Command sender for the controller; the socket lives on the window."""

    def __init__(self, window):
        self.window = window

    def _client(self):
        client = self.window.client
        if client is None:
            raise ConnectionError("not connected to the indiserver")
        return client

    def new_switch(self, device, name, members):
        self._client().new_switch(device, name, members)

    def new_text(self, device, name, members):
        self._client().new_text(device, name, members)

    def new_number(self, device, name, members):
        self._client().new_number(device, name, members)


class _Bridge:
    """INDI reader thread -> Qt signals."""

    def __init__(self, window):
        self.window = window

    def on_vector(self, vector):
        self.window.vector_received.emit(vector)

    def on_del_property(self, device, name):
        self.window.property_deleted.emit(device, name)

    def on_message(self, device, message):
        self.window.message_received.emit(device, message)

    def on_disconnected(self, reason):
        self.window.disconnected_from_server.emit(reason)


class RelayPanel(QGroupBox):
    port_toggled = pyqtSignal(str, str, bool)

    def __init__(self, alias, parent=None, columns=1):
        super().__init__(alias, parent)
        self.alias = alias
        self._buttons = {}
        self._columns = max(1, int(columns))
        self.setAlignment(Qt.AlignHCenter)
        self._grid = QGridLayout(self)
        self._grid.setSpacing(4)

    def refresh(self, model, device, interactive, exclude_ports=None):
        ports = model.ports(device, exclude_ports=exclude_ports)
        vector = model.get(device, "PORTS")
        alert = vector is not None and vector["state"] == "IPS_ALERT"
        if vector is None:
            self.setTitle("%s (offline)" % device)
        else:
            self.setTitle("%s %s" % (device, " - communication error" if alert else ""))
        wanted = {port["name"]: port for port in ports}
        for name in list(self._buttons):
            if name not in wanted:
                self._buttons[name].deleteLater()
                del self._buttons[name]
        for index, port in enumerate(ports):
            button = self._buttons.get(port["name"])
            if button is None:
                button = QPushButton(port["label"])
                button.setCheckable(True)
                button.setMinimumHeight(34)
                button.setToolTip(port["name"])
                button.clicked.connect(
                    lambda checked, member=port["name"]:
                    self.port_toggled.emit(device, member, checked))
                self._grid.addWidget(button, index // self._columns, index % self._columns)
                self._buttons[port["name"]] = button
            button.setText(port["label"])
            button.setChecked(port["on"])
            fill = "#555555" if not port["on"] else "#2e8b57"
            if alert:
                fill = "#c0392b"
            button.setStyleSheet(_button_stylesheet(fill))
            button.setEnabled(interactive and not alert)


class HeartbeatsPanel(QGroupBox):
    """Clock plus liveness indicators (indiserver, SCOPE_SAFETY, pi5, IMU)."""

    def __init__(self, parent=None):
        super().__init__("Heartbeats", parent)
        self.setAlignment(Qt.AlignHCenter)
        layout = QVBoxLayout(self)
        self.clock_label = QLabel("--:--:--")
        self.clock_label.setAlignment(Qt.AlignCenter)
        self.clock_label.setStyleSheet("font-size: 16pt; font-family: monospace; font-weight: bold;")
        layout.addWidget(self.clock_label)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Source", "State", "Detail"])
        self.tree.setRootIsDecorated(False)
        self.tree.setEditTriggers(QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.tree)

    def refresh(self, now, entries):
        self.clock_label.setText(time.strftime("%Y-%m-%d %H:%M:%S"))
        wanted = {}
        for name, entry in entries.items():
            message = entry.get("message", "")
            age = entry.get("age")
            if age is not None:
                message = "%s (last update %.0f s ago)" % (message, min(999, age))
            wanted[name] = (entry.get("ok", False), entry.get("known", False), message)
        rows = {self.tree.topLevelItem(i).text(0): self.tree.topLevelItem(i)
                for i in range(self.tree.topLevelItemCount())}
        for name, (ok, known, message) in wanted.items():
            item = rows.pop(name, None)
            if item is None:
                item = QTreeWidgetItem([name])
                self.tree.addTopLevelItem(item)
            if ok:
                state, color = "ALIVE", "#2e8b57"
            elif known:
                state, color = "STALE", "#c0392b"
            else:
                state, color = "WAITING", "#7f8c8d"
            item.setText(1, state)
            item.setBackground(1, QColor(color))
            item.setForeground(1, QColor("white"))
            item.setText(2, message)
            item.setToolTip(2, message)
        for item in rows.values():
            self.tree.takeTopLevelItem(self.tree.indexOfTopLevelItem(item))


class ScopeCanvas(QWidget):
    """Top view (azimuth, map convention) + side view (altitude) drawing.

    The scope tube is drawn tangent to the pier circle; the tangency side is
    the mount pier side (TELESCOPE_PIER_SIDE), pointing south and horizontal
    when parked.  Front lens = filled arrow, camera = thick stub at the tail.
    Right-drag orbits the 3D mount view (view_orbit: daz, dalt in degrees).
    """

    view_orbit = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(190)
        self.data = {"mount": {}, "quat": None, "quat_age": None,
                     "measured": None, "predicted": None, "discrepancy": None,
                     "reference": None, "rotation": 200.0}
        self._orbit_pos = None
        self.setMouseTracking(False)

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self._orbit_pos = event.pos()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._orbit_pos is None or not (event.buttons() & Qt.RightButton):
            super().mouseMoveEvent(event)
            return
        pos = event.pos()
        daz = -(pos.x() - self._orbit_pos.x()) * 0.5
        dalt = (pos.y() - self._orbit_pos.y()) * 0.3
        self._orbit_pos = pos
        self.view_orbit.emit(daz, dalt)
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self._orbit_pos = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        width = float(self.width())
        height = float(self.height())
        self._draw_top(painter, QRectF(0.0, 0.0, width / 2.0, height), self.data)
        self._draw_side(painter, QRectF(width / 2.0, 0.0, width / 2.0, height), self.data)

    def _sky_point(self, center, radius, az_deg):
        angle = math.radians(az_deg)
        return QPointF(center.x() + radius * math.sin(angle),
                       center.y() - radius * math.cos(angle))

    def _elev_point(self, anchor, dist, alt_deg):
        alt = max(-90.0, min(90.0, alt_deg))
        angle = math.radians(alt)
        return QPointF(anchor.x() + dist * math.cos(angle),
                       anchor.y() - dist * math.sin(angle))

    def _tangent_az(self, az_deg, pier):
        """Azimuth of the tube tangency point on the pier circle."""
        if pier == "west":
            return (az_deg + 90.0) % 360.0
        if pier == "east":
            return (az_deg - 90.0) % 360.0
        return None

    def _top_tube(self, painter, center, radius, length, az_deg, pier, color,
                  dashed, width, marked, rotation=0.0):
        painter.save()
        pen = QPen(QColor(color))
        pen.setWidth(width)
        pen.setStyle(Qt.DashLine if dashed else Qt.SolidLine)
        painter.setPen(pen)
        screen_az = (az_deg + rotation) % 360.0
        offset = self._tangent_az(az_deg, pier)
        offset = offset if offset is None else (offset + rotation) % 360.0
        tangent = center if offset is None else self._sky_point(center, radius, offset)
        tail = self._sky_point(tangent, -length, screen_az)
        tip = self._sky_point(tangent, length, screen_az)
        painter.drawLine(tail, tip)
        if marked:
            self._lens(painter, tip, screen_az, color)
            self._camera(painter, tail, screen_az, color)
        painter.restore()

    def _lens(self, painter, tip, az_deg, color):
        painter.save()
        painter.setPen(QPen(QColor(color), 1))
        painter.setBrush(QBrush(QColor(color)))
        painter.drawPolygon(QPolygonF([
            self._sky_point(tip, 13.0, az_deg),
            self._sky_point(tip, 7.0, (az_deg + 145.0) % 360.0),
            self._sky_point(tip, 7.0, (az_deg - 145.0) % 360.0)]))
        painter.restore()

    def _camera(self, painter, tail, az_deg, color):
        painter.save()
        pen = QPen(QColor(color))
        pen.setWidth(9)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(tail, self._sky_point(tail, 7.0, az_deg))
        painter.restore()

    def _draw_top(self, painter, rect, data):
        painter.save()
        painter.setPen(QPen(QColor("#777777")))
        painter.drawText(rect.adjusted(8, 6, -8, -6), Qt.AlignTop | Qt.AlignLeft, "top view (az N/E/S/W)")
        painter.drawText(rect.adjusted(8, -6, -8, 6), Qt.AlignBottom | Qt.AlignLeft,
                         "arrow = front lens, stub = camera")
        center = QPointF(rect.center().x(), rect.center().y() + 8.0)
        radius = min(rect.width(), rect.height()) * 0.20
        rotation = float(data.get("rotation") or 0.0)
        painter.setPen(QPen(QColor("#555555"), 1))
        painter.drawEllipse(center, radius, radius)
        painter.setPen(QPen(QColor("#777777")))
        for az, label in ((0.0, "N"), (90.0, "E"), (180.0, "S"), (270.0, "W")):
            painter.drawText(self._sky_point(center, radius + 12.0, (az + rotation) % 360.0), label)
        painter.setBrush(QBrush(QColor("#444444")))
        painter.drawEllipse(center, 3.5, 3.5)
        pier = (data.get("mount") or {}).get("pier")
        length = radius * 0.25 + 26.0
        if data.get("measured") is not None:
            self._top_tube(painter, center, radius, length, data["measured"][0],
                           pier, "#2e8b57", dashed=False, width=4, marked=True,
                           rotation=rotation)
        elif data.get("reference") is not None:
            self._top_tube(painter, center, radius, length, data["reference"][0],
                           pier, "#888888", dashed=False, width=3, marked=True,
                           rotation=rotation)
        if data.get("predicted") is not None:
            self._top_tube(painter, center, radius, length, data["predicted"][0],
                           pier, "#3498db", dashed=True, width=2, marked=False,
                           rotation=rotation)
        painter.restore()

    def _draw_side(self, painter, rect, data):
        painter.save()
        painter.setPen(QPen(QColor("#777777")))
        painter.drawText(rect.adjusted(8, 6, -8, -6), Qt.AlignTop | Qt.AlignLeft, "side view (altitude)")
        base_y = rect.bottom() - 18.0
        anchor = QPointF(rect.left() + 18.0, base_y)
        painter.setPen(QPen(QColor("#555555"), 2))
        painter.drawLine(QPointF(rect.left() + 6.0, base_y),
                         QPointF(rect.right() - 6.0, base_y))
        length = min(rect.width() - 40.0, (rect.height() - 40.0))
        if data.get("measured") is not None:
            self._elevation_line(painter, anchor, length, data["measured"][1],
                                 "#2e8b57", dashed=False, marked=True)
        elif data.get("reference") is not None:
            self._elevation_line(painter, anchor, length, data["reference"][1],
                                 "#888888", dashed=False, marked=True)
        if data.get("predicted") is not None:
            self._elevation_line(painter, anchor, length, data["predicted"][1],
                                 "#3498db", dashed=True, marked=False)
        painter.restore()

    def _elevation_line(self, painter, anchor, length, alt_deg, color, dashed, marked=False):
        painter.save()
        pen = QPen(QColor(color))
        pen.setWidth(4)
        pen.setStyle(Qt.DashLine if dashed else Qt.SolidLine)
        painter.setPen(pen)
        tail = self._elev_point(anchor, -length * 0.30, alt_deg)
        tip = self._elev_point(anchor, length * 0.70, alt_deg)
        painter.drawLine(tail, tip)
        if marked:
            screen_az = 90.0 - max(-90.0, min(90.0, alt_deg))
            self._lens(painter, tip, screen_az, color)
            self._camera(painter, tail, screen_az, color)
        painter.restore()


class ScopePanel(QGroupBox):
    """IMU + mount visualization: azimuth / altitude pointers and readouts."""

    orbit_requested = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__("Scope / mount position", parent)
        layout = QVBoxLayout(self)
        self.readout = QLabel("")
        self.readout.setStyleSheet("font-family: monospace; font-size: 10pt;")
        self.readout.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        layout.addWidget(self.readout)
        self.canvas = ScopeCanvas()
        self.canvas.view_orbit.connect(self.orbit_requested.emit)
        layout.addWidget(self.canvas, 1)

    def refresh(self, data):
        self.canvas.data = dict(data)
        self.canvas.update()
        mount = data.get("mount") or {}
        lines = ["mount  RA %s  DEC %s  pier %s%s" % (
            format_ra(mount.get("ra")), format_dec(mount.get("dec")),
            mount.get("pier") or "?",
            "  PARKED" if mount.get("parked") else "")]
        if data.get("reference") is not None:
            pier = mount.get("pier")
            lines.append("parked ref: az 180 alt 0 (south, horizontal%s)"
                         % (", scope on %s side of pier" % pier if pier
                            else ", pier side unknown"))
        quat = data.get("quat")
        if quat is not None:
            lines.append("IMU    w %+.4f  x %+.4f  y %+.4f  z %+.4f  (%.0f s ago)"
                         % (quat[0], quat[1], quat[2], quat[3],
                            data.get("quat_age") or 0.0))
        else:
            lines.append("IMU    no quaternion (add a type: quaternion check)")
        parts = []
        if data.get("park_angle") is not None:
            parts.append("IMU vs park pose %.1f" % data["park_angle"])
        if data.get("measured") is not None:
            parts.append("IMU az %.0f alt %.0f" % data["measured"])
        if data.get("predicted") is not None:
            parts.append("mount az %.0f alt %.0f" % data["predicted"])
        if data.get("discrepancy") is not None:
            parts.append("difference %.1f" % data["discrepancy"])
        lines.append("point  " + ("; ".join(parts) if parts else "waiting"))
        self.readout.setText("\n".join(lines))


class Mount3DCanvas(QWidget):
    """Simple orthographic 3D stick figure of the german equatorial mount."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(240)
        self.figure = {"segments": [], "labels": [], "markers": [],
                       "view": (200.0, 25.0)}

    def set_figure(self, figure):
        if figure is not None:
            self.figure = figure
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(0.0, 0.0, float(self.width()), float(self.height()))
        view_az, view_alt = self.figure.get("view", (200.0, 25.0))
        center = (rect.center().x(), rect.center().y() + rect.height() * 0.18)
        scale = min(rect.width(), rect.height()) * 0.25
        painter.setPen(QPen(QColor("#777777")))
        for point, text in self.figure.get("labels", []):
            u, v, _ = mount3d.project(point, view_az, view_alt, scale, center)
            painter.drawText(QPointF(u, v), text)
        for segment in mount3d.sort_for_drawing(self.figure):
            ua, va, _ = mount3d.project(segment["a"], view_az, view_alt, scale, center)
            ub, vb, _ = mount3d.project(segment["b"], view_az, view_alt, scale, center)
            pen = QPen(QColor(segment["color"]))
            pen.setWidthF(max(1.5, segment["width"] * scale / 260.0))
            pen.setCapStyle(Qt.RoundCap)
            painter.setPen(pen)
            painter.drawLine(QPointF(ua, va), QPointF(ub, vb))
        for marker in self.figure.get("markers", []):
            self._draw_marker(painter, marker, view_az, view_alt, scale, center)

    def _draw_marker(self, painter, marker, view_az, view_alt, scale, center):
        u, v, _ = mount3d.project(marker["point"], view_az, view_alt,
                                  scale, center)
        if marker["kind"] == "lens":
            radius = max(3.0, scale * 0.055)
            painter.setPen(QPen(QColor("#eef7f2"), 1))
            painter.setBrush(QBrush(QColor(marker["color"])))
            painter.drawEllipse(QPointF(u, v), radius, radius)
        else:
            side = max(5.0, scale * 0.09)
            painter.setPen(QPen(QColor("#eef7f2"), 1))
            painter.setBrush(QBrush(QColor(marker["color"])))
            painter.drawRect(QRectF(u - side / 2.0, v - side / 2.0, side, side))


class CamPanel(QGroupBox):
    """RTSP view of the camera inside the shed (low fps, low CPU)."""

    frame_received = pyqtSignal(object)
    status_changed = pyqtSignal(bool, str)

    def __init__(self, config, log, parent=None):
        super().__init__("Shed camera", parent)
        self.setAlignment(Qt.AlignHCenter)
        self._log = log
        self._reader = None
        self._config = config
        self.view = QLabel("camera disabled")
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setMinimumHeight(150)
        self.view.setStyleSheet(
            "background: #202020; color: #909090; font-size: 9pt;")
        layout = QVBoxLayout(self)
        self.toggle = QCheckBox("stream on")
        self.toggle.toggled.connect(self._on_toggle)
        layout.addWidget(self.toggle)
        layout.addWidget(self.view, 1)
        self.frame_received.connect(self._on_frame)
        self.status_changed.connect(self._on_status)
        self.restart(config)

    def restart(self, config):
        self._config = config
        self.stop()
        settings = config.get("shed_camera") or {}
        enabled = bool(settings.get("enabled")) and bool(settings.get("url"))
        self.toggle.blockSignals(True)
        self.toggle.setChecked(enabled)
        self.toggle.blockSignals(False)
        if not enabled:
            self.view.setText("camera disabled")
            return
        self._start_reader(settings)

    def _on_toggle(self, checked):
        self.stop()
        settings = self._config.get("shed_camera") or {}
        if checked and settings.get("url"):
            self._start_reader(settings)
        else:
            self.toggle.blockSignals(True)
            self.toggle.setChecked(False)
            self.toggle.blockSignals(False)
            self.view.setPixmap(QPixmap())
            self.view.setText("no url" if checked
                              else "camera disabled")

    def _start_reader(self, settings):
        from .camera import RtspReader, strip_url_credentials
        self._log("camera: connecting to %s"
                  % strip_url_credentials(settings["url"]))
        self._reader = RtspReader(
            settings["url"], settings.get("fps", 8),
            on_frame=self.frame_received.emit,
            on_status=self.status_changed.emit)
        self._reader.start()

    def stop(self):
        if self._reader is not None:
            self._reader.stop()
            self._reader = None

    def _on_frame(self, frame):
        if frame is None or frame.ndim != 3:
            return
        height, width = frame.shape[:2]
        image = QImage(frame.data, width, height, int(frame.strides[0]),
                       QImage.Format_BGR888).copy()
        self.view.setPixmap(QPixmap.fromImage(image).scaled(
            self.view.size(), Qt.KeepAspectRatio, Qt.FastTransformation))

    def _on_status(self, ok, message):
        if not ok:
            self.view.setPixmap(QPixmap())
            self.view.setText(message)


class RoofPanel(QGroupBox):
    roof_open = pyqtSignal()
    roof_close = pyqtSignal()
    roof_stop = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__("Roof", parent)
        self.setAlignment(Qt.AlignHCenter)
        layout = QVBoxLayout(self)
        self.state_label = QLabel("offline")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size: 18pt; font-weight: bold;")
        layout.addWidget(self.state_label)
        buttons = QHBoxLayout()
        self.open_button = QPushButton("OPEN")
        self.close_button = QPushButton("CLOSE")
        self.stop_button = QPushButton("STOP")
        self.open_button.setStyleSheet(_button_stylesheet("#2e8b57", "font-size: 12pt;"))
        self.close_button.setStyleSheet(_button_stylesheet("#d0a020", "font-size: 12pt;"))
        self.stop_button.setStyleSheet(_button_stylesheet("#c0392b", "font-size: 12pt;"))
        for button in (self.open_button, self.close_button, self.stop_button):
            button.setMinimumHeight(46)
            buttons.addWidget(button)
        self.open_button.clicked.connect(self.roof_open.emit)
        self.close_button.clicked.connect(self.roof_close.emit)
        self.stop_button.clicked.connect(self.roof_stop.emit)
        layout.addLayout(buttons)
        self.safety_tree = QTreeWidget()
        self.safety_tree.setHeaderLabels(["Safety", "State"])
        self.safety_tree.setRootIsDecorated(False)
        self.safety_tree.setMaximumHeight(130)
        self.safety_tree.setEditTriggers(QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.safety_tree)

    def refresh(self, model, device, movable, stop_allowed, scope_safe=False,
                block_reason=""):
        self.open_button.setEnabled(movable)
        self.close_button.setEnabled(movable)
        self.open_button.setToolTip(block_reason)
        self.close_button.setToolTip(block_reason)
        self.stop_button.setEnabled(stop_allowed)
        if device is None or model.get(device, "DOME_SHUTTER") is None:
            self.state_label.setText("offline")
            self.state_label.setStyleSheet("color: %s; font-size: 18pt; font-weight: bold;"
                                           % IPSTATE_COLORS["IPS_IDLE"])
            self.safety_tree.clear()
            return
        state = model.roof_state(device)
        color = ROOF_COLORS.get(state["state"], IPSTATE_COLORS["IPS_IDLE"])
        self.state_label.setText(state["text"])
        self.state_label.setStyleSheet("color: %s; font-size: 18pt; font-weight: bold;" % color)
        lights = model.roof_safety(device)
        self.safety_tree.clear()
        safety = "ok" if scope_safe else "not safe"
        scope_item = QTreeWidgetItem(["Scope safety", safety])
        scope_item.setBackground(1, QColor("#2e8b57" if scope_safe else "#c0392b"))
        scope_item.setForeground(1, QColor("white"))
        self.safety_tree.addTopLevelItem(scope_item)
        for member, label in SAFETY_LABELS:
            if member not in lights:
                continue
            item = QTreeWidgetItem([label, lights[member].replace("IPS_", "")])
            item.setBackground(1, QColor(LIGHT_COLORS.get(lights[member], "#555555")))
            item.setForeground(1, QColor("white"))
            self.safety_tree.addTopLevelItem(item)


class ChecksPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Health checks (running mode)", parent)
        layout = QVBoxLayout(self)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Check", "Status", "Detail"])
        self.tree.setRootIsDecorated(False)
        self.tree.setEditTriggers(QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.tree)

    def refresh(self, statuses):
        rows = {self.tree.topLevelItem(i).text(0): self.tree.topLevelItem(i)
                for i in range(self.tree.topLevelItemCount())}
        for name, entry in statuses.items():
            item = rows.pop(name, None)
            if item is None:
                item = QTreeWidgetItem([name])
                self.tree.addTopLevelItem(item)
            ok = entry.get("last_ok")
            detail = entry.get("last_message") or ""
            if entry.get("recovering"):
                state, color = "RECOVERING", "#d0a020"
            elif ok is None:
                state, color = "WAITING", "#7f8c8d"
            elif ok:
                state, color = "OK", "#2e8b57"
            else:
                state, color = "FAIL (%d)" % entry.get("failures", 0), "#c0392b"
            item.setText(1, state)
            item.setBackground(1, QColor(color))
            item.setForeground(1, QColor("white"))
            item.setText(2, detail)
            item.setToolTip(2, detail)
        for item in rows.values():
            self.tree.takeTopLevelItem(self.tree.indexOfTopLevelItem(item))


class ModePanel(QGroupBox):
    mode_selected = pyqtSignal(str)
    abort_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__("Operating mode", parent)
        layout = QGridLayout(self)
        self.big_label = QLabel(Mode.INACTIVE)
        self.big_label.setAlignment(Qt.AlignCenter)
        self.big_label.setStyleSheet("font-size: 22pt; font-weight: bold; color: %s;"
                                     % MODE_COLORS[Mode.INACTIVE])
        layout.addWidget(self.big_label, 0, 0, 1, 2)
        self.combo = QComboBox()
        self.apply_button = QPushButton("Apply")
        self.apply_button.clicked.connect(
            lambda: self.mode_selected.emit(self.combo.currentText()))
        layout.addWidget(self.combo, 1, 0)
        layout.addWidget(self.apply_button, 1, 1)
        self.abort_button = QPushButton("ABORT sequence")
        self.abort_button.setStyleSheet(_button_stylesheet("#c0392b"))
        self.abort_button.clicked.connect(self.abort_requested.emit)
        self.abort_button.hide()
        layout.addWidget(self.abort_button, 2, 0, 1, 2)

    def refresh(self, mode, allowed, busy):
        self.big_label.setText(mode)
        self.big_label.setStyleSheet("font-size: 22pt; font-weight: bold; color: %s;"
                                     % MODE_COLORS.get(mode, "#7f8c8d"))
        current = self.combo.currentText()
        self.combo.blockSignals(True)
        self.combo.clear()
        for option in dict.fromkeys(list(allowed) + [mode]):
            self.combo.addItem(option)
        if current in allowed or current == mode:
            self.combo.setCurrentText(current)
        else:
            self.combo.setCurrentText(allowed[0] if allowed else mode)
        self.combo.setEnabled(bool(allowed))
        self.apply_button.setEnabled(bool(allowed))
        self.combo.blockSignals(False)
        self.abort_button.setVisible(busy)


class ConfigPanel(QGroupBox):
    def __init__(self, config, parent=None):
        super().__init__("Configuration", parent)
        self.setAlignment(Qt.AlignHCenter)
        layout = QFormLayout(self)
        for key, value in config["indiserver"].items():
            label = QLabel(str(value))
            label.setWordWrap(True)
            layout.addRow(QLabel("%s:" % key), label)
        devices = ", ".join("%s=%s" % (k, v) for k, v in sorted(config["devices"].items()))
        devices_label = QLabel(devices)
        devices_label.setWordWrap(True)
        layout.addRow(QLabel("Devices:"), devices_label)
        hidden = ", ".join(config["relay_exclude_names"]) or "-"
        hidden_label = QLabel(hidden)
        hidden_label.setWordWrap(True)
        layout.addRow(QLabel("Hidden relays:"), hidden_label)


class LogPanel(QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(4000)

    def append(self, text):
        self.appendPlainText(text)


class MainWindow(QMainWindow):
    reconnect_event = pyqtSignal()
    log_event = pyqtSignal(str)
    vector_received = pyqtSignal(object)
    property_deleted = pyqtSignal(str, str)
    message_received = pyqtSignal(str, str)
    disconnected_from_server = pyqtSignal(str)
    mode_event = pyqtSignal(str, object)

    def __init__(self, config, config_path):
        super().__init__()
        self.config = config
        self.config_path = config_path
        self.model = IndiModel()
        self.client = None
        self.controller = Controller(self.model, _ClientProxy(self), config, self.log)
        self.controller.add_listener(self._on_controller_event)
        self._last_server_rx = time.time()
        self._safety_rx = None
        self._safety_ok = False
        self._park_anchor = None
        self.setWindowTitle("indi-relay-boxes control")
        self._build_menu()
        self._build_ui()
        self._wire()
        self.log("config: %s" % config_path)
        self._connect()

    def _build_menu(self):
        menu = self.menuBar().addMenu("&File")
        reload_action = menu.addAction("Reload &config")
        reload_action.triggered.connect(self._reload_config)
        quit_action = menu.addAction("Quit")
        quit_action.setShortcut(QKeySequence("Ctrl+Q"))
        quit_action.triggered.connect(self.close)

    def toggle_maximized(self):
        if self.isMaximized():
            self.showNormal()
        else:
            self.showMaximized()

    def _build_ui(self):
        central = QWidget()
        layout = QHBoxLayout(central)
        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.relay_panels = []
        relay_columns = QHBoxLayout()
        relay_columns.setSpacing(6)
        for alias in ("kmtronic8", "relay16"):
            panel = RelayPanel(alias)
            panel.port_toggled.connect(self._on_port_toggled)
            self.relay_panels.append(panel)
            relay_columns.addWidget(panel)
        left_layout.addLayout(relay_columns)
        left_layout.addWidget(ConfigPanel(self.config))
        left_layout.addStretch(1)
        left.setMaximumWidth(400)
        splitter.addWidget(left)

        middle = QWidget()
        middle_layout = QVBoxLayout(middle)
        middle_layout.setContentsMargins(0, 0, 0, 0)
        self.roof_panel = RoofPanel()
        self.mode_panel = ModePanel()
        self.scope_panel = ScopePanel()
        self.cam_panel = CamPanel(self.config, self.log)
        middle_layout.addWidget(self.roof_panel)
        middle_layout.addWidget(self.mode_panel)
        middle_layout.addWidget(self.scope_panel, 1)
        middle_layout.addWidget(self.cam_panel, 1)
        splitter.addWidget(middle)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.mount3d_panel = Mount3DCanvas()
        self.heartbeats_panel = HeartbeatsPanel()
        self.checks_panel = ChecksPanel()
        self.log_panel = LogPanel()
        right_layout.addWidget(self.heartbeats_panel)
        right_layout.addWidget(self.checks_panel)
        right_layout.addWidget(self.mount3d_panel, 1)
        right_layout.addWidget(self.log_panel, 1)
        splitter.addWidget(right)

        self._ticker = QTimer(self)
        self._ticker.timeout.connect(self._tick)
        self._ticker.start(500)

        QShortcut(QKeySequence("Ctrl+F"), self, self.toggle_maximized)

        splitter.setSizes([320, 360, 420])
        self.setCentralWidget(central)

        self.connection_label = QLabel("disconnected")
        self.connection_label.setStyleSheet("color: #c0392b; font-weight: bold;")
        self.reconnect_button = QPushButton("Reconnect")
        self.reconnect_button.clicked.connect(self._connect)
        self.statusBar().addWidget(self.connection_label)
        self.statusBar().addWidget(self.reconnect_button)

    def _wire(self):
        self.log_event.connect(self._on_log)
        self.reconnect_event.connect(self._on_reconnect_requested)
        self.vector_received.connect(self._on_vector)
        self.property_deleted.connect(self._on_property_deleted)
        self.message_received.connect(lambda dev, msg: self.log("[%s] %s" % (dev, msg)))
        self.disconnected_from_server.connect(self._on_disconnected)
        self.mode_event.connect(self._on_mode_event)
        self.scope_panel.orbit_requested.connect(self._orbit_3d)
        self.mode_panel.mode_selected.connect(lambda mode: self.controller.set_mode(mode))
        self.mode_panel.abort_requested.connect(self.controller.abort_sequence)
        self.roof_panel.roof_open.connect(self._on_roof_open)
        self.roof_panel.roof_close.connect(self._on_roof_close)
        self.roof_panel.roof_stop.connect(self.controller.roof_stop)
        self._refresh_all()

    def log(self, message):
        self.log_event.emit(str(message))

    def _on_log(self, message):
        self.log_panel.append(message)

    def _connect(self):
        if self.client is not None:
            old, self.client = self.client, None
            old.close()
        host = self.config["indiserver"]["host"]
        port = self.config["indiserver"]["port"]
        bridge = _Bridge(self)
        client = IndiClient(host, port,
                            on_vector=bridge.on_vector,
                            on_del_property=bridge.on_del_property,
                            on_message=bridge.on_message,
                            on_disconnected=bridge.on_disconnected)
        try:
            client.connect()
        except OSError as error:
            self.log("cannot connect to %s:%d: %s" % (host, port, error))
            self._apply_connection_state(False, "connect failed")
            return
        self.client = client
        self._apply_connection_state(True, "%s:%d" % (host, port))
        self.log("connected to %s:%d, waiting for properties" % (host, port))

    def _apply_connection_state(self, connected, text):
        self.connection_label.setText(text)
        color = "#2e8b57" if connected else "#c0392b"
        self.connection_label.setStyleSheet("color: %s; font-weight: bold;" % color)

    def _on_vector(self, vector):
        self._last_server_rx = time.time()
        self.controller.ensure_device_connected(vector.get("device", ""))
        if (vector.get("name") == "CONNECTION"
                and vector.get("type") == "SwitchVector"):
            self.controller.note_connection_state(
                vector.get("device", ""),
                vector["members"].get("CONNECT", {}).get("on", False))
        if vector.get("name") == "SCOPE_SAFETY":
            self._safety_rx = time.time()
            self._safety_ok = (vector["members"].get("SAFE", {}).get("light") == "IPS_OK")
        self.model.apply_vector(vector)
        self._refresh_all()

    def _on_property_deleted(self, device, name):
        self.model.remove(device, name)
        self.controller.forget_device(device, name)
        self._refresh_all()

    def _on_disconnected(self, reason):
        self.model = IndiModel()
        self.controller.reset_connections()
        self._safety_rx = None
        self._safety_ok = False
        self._park_anchor = None
        self.log("indiserver connection lost: %s" % reason)
        self._apply_connection_state(False, "disconnected")
        self._refresh_all()

    def _on_controller_event(self, event, payload):
        if event == "reconnect_requested":
            self.reconnect_event.emit()
            return
        self.mode_event.emit(event, payload)

    def _on_reconnect_requested(self):
        self.log("pi5_wake: indiserver should be back - reconnecting")
        self._connect()

    def _on_mode_event(self, event, payload):
        if event != "mode_changed":
            self.log("mode: %s" % payload)
        self._refresh_all()

    def _on_roof_open(self):
        try:
            self.controller.send_roof_motion("SHUTTER_OPEN")
        except Exception as error:
            self.log("roof command failed: %s" % error)

    def _on_roof_close(self):
        try:
            self.controller.send_roof_motion("SHUTTER_CLOSE")
        except Exception as error:
            self.log("roof command failed: %s" % error)

    def _on_port_toggled(self, device, member, on):
        alias = next((k for k, v in self.config["devices"].items() if v == device), None)
        self.controller.set_port(alias or device, member, on)
        self._refresh_all()

    def _tick(self):
        now = time.time()
        entries = {}
        connected = self.client is not None
        if connected:
            self.controller.poll_device_connections(now)
        for device in sorted(set(self.config["devices"].values())):
            state = self.controller.device_connection_status(device)
            entries[device] = {
                "ok": state == "connected",
                "known": state != "waiting",
                "age": None,
                "message": "CONNECTION %s" % state}
        server_age = now - self._last_server_rx
        entries["indiserver"] = {
            "ok": connected and server_age < 30,
            "known": connected,
            "age": server_age if connected else None,
            "message": "INDI server messages"}
        if self._safety_rx is not None:
            entries["SCOPE_SAFETY"] = {
                "ok": self._safety_ok and (now - self._safety_rx) < 5,
                "known": True,
                "age": now - self._safety_rx,
                "message": "scope safety (EulFocuser)"}
        else:
            entries["SCOPE_SAFETY"] = {"ok": False, "known": False, "age": None,
                                       "message": "no SCOPE_SAFETY update yet"}
        for name, monitor in self.controller.monitors.items():
            entries[name] = monitor.status(now)
        self.heartbeats_panel.refresh(now, entries)
        self.scope_panel.refresh(self._scope_data(now))

    def _scope_data(self, now):
        mount_device = self.config["devices"].get("mount")
        data = {"mount": self.model.mount_state(mount_device) or {},
                "quat": None, "quat_age": None, "measured": None,
                "predicted": None, "discrepancy": None, "reference": None,
                "park_angle": None, "figure": None, "rotation":
                float(self.config.get("scope_view_rotation", 200.0))}
        monitor = None
        for candidate in self.controller.monitors.values():
            if getattr(candidate, "kind", "") == "quaternion":
                monitor = candidate
                break
        if monitor is not None and monitor.quaternion and monitor.quaternion_at:
            age = now - monitor.quaternion_at
            data["quat"] = monitor.quaternion
            data["quat_age"] = age
            # While the mount reports PARK the freshest quaternion IS the park
            # reference: this re-anchors constantly, so sensor yaw drift cannot
            # make the parked scope wander away from plain south.  After the
            # scope moves, pointings are inferred from this last parked reading.
            if data["mount"].get("parked") and age < 10:
                self._park_anchor = data["quat"]
            park_ref = self._park_anchor or self.config.get("imu_park_quaternion")
            if park_ref and age < 10:
                data["measured"] = optical_axis(
                    data["quat"], park_ref, self.config.get("imu_optical_axis", "+x"))
                data["park_angle"] = quat_angle(park_ref, data["quat"])
        if data["mount"].get("parked"):
            data["reference"] = (180.0, 0.0)   # parked: horizontal, pointing south
        mount = data["mount"]
        site = self.model.site_coords(mount_device)
        view3d = self.config.get("mount3d_view") or {"az": 200.0, "alt": 25.0}
        data["figure"] = mount3d.build_figure(
            mount, site,
            float(view3d.get("az", 200.0)), float(view3d.get("alt", 25.0)))
        if (mount.get("ra") is not None and mount.get("dec") is not None
                and site and (site["lat"] or site["lon"])):
            data["predicted"] = alt_az(mount["ra"], mount["dec"],
                                       site["lat"], site["lon"])
        if data["measured"] is not None and data["predicted"] is not None:
            data["discrepancy"] = angle_between(data["measured"], data["predicted"])
        return data

    def _orbit_3d(self, daz, dalt):
        view = self.config.setdefault(
            "mount3d_view", {"az": 200.0, "alt": 25.0})
        view["az"] = (float(view.get("az", 200.0)) + daz) % 360.0
        view["alt"] = max(-80.0, min(80.0, float(view.get("alt", 25.0)) + dalt))
        self.scope_panel.refresh(self._scope_data(time.time()))

    def _refresh_all(self):
        interactive = self.controller.relay_actions_allowed()
        roof_relays = self.model.roof_control_ports(self.config["devices"].get("roof"))
        seen = set()
        for panel in self.relay_panels:
            device = self.config["devices"].get(panel.alias, panel.alias)
            seen.add(device)
            exclude = roof_relays if panel.alias == "relay16" else None
            panel.refresh(self.model, device, interactive, exclude_ports=exclude)
        self.roof_panel.refresh(self.model, self.config["devices"].get("roof"),
                                movable=self.controller.roof_movable(),
                                stop_allowed=self.controller.roof_stop_allowed(),
                                scope_safe=self.controller.scope_safe(),
                                block_reason=self.controller.roof_block_reason())
        self.mode_panel.refresh(self.controller.mode, self.controller.allowed_mode_changes(),
                                self.controller.is_busy())
        checks = self.controller.checks
        self.checks_panel.refresh(checks.status_snapshot() if checks else {})

    def _reload_config(self):
        from . import config as config_module
        try:
            self.config = config_module.load(self.config_path)
        except Exception as error:
            self.log("config reload failed: %s" % error)
            return
        self.controller.config = self.config
        self.cam_panel.restart(self.config)
        self.log("config reloaded from %s" % self.config_path)
        self._refresh_all()

    def closeEvent(self, event):
        if self.controller.mode != Mode.INACTIVE:
            answer = QMessageBox.question(
                self, "Still active",
                "The operating mode is %r.\nLeave everything as it is and quit?"
                % self.controller.mode)
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        self.cam_panel.stop()
        self.controller.shutdown()
        if self.client is not None:
            self.client.close()
        event.accept()
