'''
MIT License

Copyright (c) 2024 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import time
import math
import shutil
import threading
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from PyQt5.QtCore import QObject, pyqtSignal, QProcess, QThread, QDateTime, QTimer, Qt, QPointF, QRectF
from PyQt5.QtGui import QBrush, QColor, QFont, QFontMetrics, QImage, QPainter, QPen
from PyQt5.QtWidgets import QAbstractItemView, QMessageBox, QTableWidgetItem, QWidget
import Common
from geometry_msgs.msg import Point, PoseStamped

try:
    import os
    os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')
    import pyqtgraph as pg
    _HAS_PYQTGRAPH = True
except ImportError:
    _HAS_PYQTGRAPH = False

POSITION_PLOT_HISTORY_S = 10.0  # seconds of history shown in the live plots
STEP_RESPONSE_DEFAULT_WINDOW_S = 30
STEP_RESPONSE_MAX_WINDOW_S = 60
# Cap on the flight-log widget. Every entry is edge-triggered (arm/disarm, mode change,
# yaw-align flip, operator actions), so this is not a spam guard -- it bounds growth over
# a long session, where an unbounded QListWidget makes each scrollToBottom() slower.
LOG_MAX_LINES = 500
# Onboard RealSense + YOLO node (VLM tab): vision_localization on Orin0, whose namespace
# comes from its `--ros-ns` flag. It ran as /vision for the 2026-09-24 bench bag and as
# /uav_0 from 2026-09-25; if the flag changes, change these two and nothing else.
VISION_IMAGE_TOPIC = '/uav_0/color/compressed'
VISION_DETECTIONS_TOPIC = '/uav_0/detections'
# No frame (or no detections) for this long -> the view says so instead of showing a
# frozen frame as if it were live. Receive time, not header stamp: the Jetson's clock
# is not synchronised with the ground station's.
VISION_STALE_S = 1.0
# OptiTrack liveness indicator (optitrack_status). The RAW VRPN stream, on purpose: see
# the subscription in SingleDroneRosNode for why /uav_0/mocap cannot show a dropout.
# The rigid body must be named `uav_0` in Motive for this topic to exist.
OPTITRACK_TOPIC = '/vrpn_mocap/uav_0/pose'
# No frame for this long -> "No OptiTrack". Tune here. The stream runs at ~120 Hz, so
# 0.4 s is ~48 missed frames: well clear of normal WiFi jitter (largest gap in the
# 2026-09-24 flight bag: 23 ms), still quick enough to act on.
OPTITRACK_TIMEOUT_S = 0.4
# DDS discovery takes a moment after launch; no "No OptiTrack" verdict before this.
OPTITRACK_STARTUP_GRACE_S = 2.0
# Audio alarms (AudioAnnouncer): spoken text through speech-dispatcher, preceded by a
# siren for a loss. Both optional: without them the indicators still work, silently.
SPEECH_COMMAND = shutil.which('spd-say')
# The siren is generated in memory (_siren_pcm) and piped to a raw-PCM player, so no
# sound file ships with the repo. pacat (PulseAudio/PipeWire) first, plain ALSA second.
SIREN_RATE_HZ = 22050
if shutil.which('pacat'):
    SIREN_PLAYER = ['pacat', '--raw', '--format=s16le', f'--rate={SIREN_RATE_HZ}', '--channels=1']
elif shutil.which('aplay'):
    SIREN_PLAYER = ['aplay', '-q', '-t', 'raw', '-f', 'S16_LE', '-r', str(SIREN_RATE_HZ), '-c', '1']
else:
    SIREN_PLAYER = None
# Orin health report (cpu_status / wifi_status): fsc_system_monitor on Orin0, JSON in a
# std_msgs/String at 1 Hz. Its `stamp` is the Orin's clock, so freshness is judged by
# receive time here, like the camera view.
SYSTEM_STATUS_TOPIC = '/uav_0/system_monitor/status'
# Three missed 1 Hz reports -> "No WiFi". The Orin is WiFi-only, so from the ground
# station a silent monitor and a dead link look the same; the text says which it saw.
SYSTEM_STATUS_STALE_S = 3.0
# "Fair" (yellow) thresholds, from the system_monitor author's notes (2026-09-26).
WIFI_WEAK_SIGNAL_DBM = -75.0
WIFI_SLOW_PING_MS = 50.0
WIFI_BACKLOG_REPORTS = 3          # consecutive reports with packets queued for WiFi
WIFI_UDP_TXQ_WARN_BYTES = 100_000  # the kernel limit is 212992
# Background colours for the status labels (same green as the rotor pies).
STATUS_STYLE = {
    'good': 'background-color: #24A148; color: white;',
    'fair': 'background-color: #F1C21B; color: black;',
    'bad': 'background-color: #DA1E28; color: white;',
}
# cpu_status and wifi_status are two-line labels 41 px tall.
TWO_LINE_FONT = ' font-size: 9pt;'
# from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandLong, SetMode
from px4_msgs.msg import ActuatorMotors, VehicleStatus,VehicleAttitudeSetpoint,VehicleAttitude, VehicleGlobalPosition, BatteryStatus,VehicleRatesSetpoint, EstimatorStatusFlags
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference, PositionControllerState, VehicleInfo
from fsc_autopilot_ros2_msgs.srv import ActivateController, ListControllers

# from mavros_msgs.msg import State, AttitudeTarget
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import json
# from fsc_autopilot_msgs.msg import TrackingReference
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def wifi_summary(wifi, backlog_reports):
    """(level, text) for wifi_status from one report's `wifi` block.

    Every field may be null (not measured yet), so each one is checked on its own.
    `backlog_reports` is how many consecutive reports have shown a WiFi send backlog.
    """
    wifi = wifi or {}
    connected = wifi.get('connected')
    if connected is False:
        return 'bad', 'No WiFi\nOrin reports no access point'
    if connected is None:
        # Not measured yet. The report itself got here, so the link is not down.
        return 'fair', 'WiFi: link unknown\nmonitor has no WiFi data yet'
    signal = wifi.get('signal_dbm')
    ping = wifi.get('ping_ms')
    freq = wifi.get('freq_mhz')
    txq = wifi.get('udp_txq_max_bytes')

    problems = []
    if signal is not None and signal < WIFI_WEAK_SIGNAL_DBM:
        problems.append('weak signal')
    if ping is None:
        problems.append('ping lost')
    elif ping > WIFI_SLOW_PING_MS:
        problems.append('slow ping')
    if backlog_reports >= WIFI_BACKLOG_REPORTS:
        problems.append('TX backlog')
    if txq is not None and txq > WIFI_UDP_TXQ_WARN_BYTES:
        problems.append('send queue high')

    details = []
    if signal is not None:
        details.append(f'{signal:.0f} dBm')
    if ping is not None:
        details.append(f'{ping:.1f} ms' if ping < 10 else f'{ping:.0f} ms')
    if freq:
        details.append('2.4 GHz' if freq < 3000 else ('5 GHz' if freq < 5925 else '6 GHz'))
    details = ' · '.join(details)
    if problems:
        # Two at most, so the first line fits the label.
        return 'fair', f"WiFi: {', '.join(problems[:2])}\n{details}"
    return 'good', f'WiFi good\n{details}'


def cpu_summary(cpu):
    """Two-line load text for cpu_status, or None if the report carries no loads."""
    loads = (cpu or {}).get('load_pct')
    if not loads:
        return None

    def pct(i):
        value = loads[i] if i < len(loads) else None
        return '-' if value is None else f'{value:.0f}%'

    # CPUs 0-3 are the housekeeping group; 4 and 5 are isolated for the uXRCE-DDS
    # Agent and the control node (system_monitor's field notes).
    housekeeping = '   '.join(f'cpu{i} {pct(i)}' for i in range(4))
    return f'{housekeeping}\ncpu4 (Agent) {pct(4)}   cpu5 (control) {pct(5)}'


class QuadrotorThrottleWidget(QWidget):
    """Top-down X-frame view with one normalized-throttle pie per motor."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._commands = [0.0, 0.0, 0.0, 0.0]

    def set_commands(self, commands):
        self._commands = [max(0.0, min(1.0, float(value))) for value in commands[:4]]
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()
        center = QPointF(width / 2.0, height / 2.0 - 4.0)
        rotor_centers = [
            QPointF(width * 0.77, height * 0.23),  # motor 1: front-right
            QPointF(width * 0.23, height * 0.68),  # motor 2: rear-left
            QPointF(width * 0.23, height * 0.23),  # motor 3: front-left
            QPointF(width * 0.77, height * 0.68),  # motor 4: rear-right
        ]

        painter.setPen(QPen(QColor("#555555"), 5, Qt.SolidLine, Qt.RoundCap))
        for rotor_center in rotor_centers:
            painter.drawLine(center, rotor_center)
        painter.setBrush(QBrush(QColor("#555555")))
        painter.drawEllipse(center, 7, 7)

        radius = max(18.0, min(width, height) * 0.105)
        painter.setFont(QFont("Sans Serif", 8))
        for index, (rotor_center, command) in enumerate(zip(rotor_centers, self._commands)):
            rotor_rect = QRectF(
                rotor_center.x() - radius,
                rotor_center.y() - radius,
                radius * 2.0,
                radius * 2.0,
            )
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor("#E6E6E6")))
            painter.drawEllipse(rotor_rect)
            painter.setBrush(QBrush(QColor("#24A148")))
            painter.drawPie(rotor_rect, 90 * 16, -round(command * 360 * 16))
            painter.setPen(QPen(QColor("#333333"), 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(rotor_rect)

            text_rect = QRectF(
                rotor_center.x() - 39,
                rotor_center.y() + radius + 2,
                78,
                18,
            )
            painter.setPen(QColor("#222222"))
            painter.drawText(
                text_rect,
                Qt.AlignCenter,
                f"M{index + 1}: {command * 100:.1f}%",
            )

        painter.setPen(QColor("#555555"))
        painter.drawText(QRectF(center.x() - 25, 2, 50, 16), Qt.AlignCenter, "FRONT")


class CameraDetectionView(QWidget):
    """Camera frame, letterboxed to the widget, with the detector's boxes on top.

    Boxes arrive in pixels of the published image, so they go through the same
    scale/offset as the image rather than being drawn in widget coordinates.
    """

    BOX_COLOR = QColor(50, 255, 30)  # same green as the bench bag's vision_viz.py
    ALERT_COLOR = QColor("#FF5050")

    def __init__(self, parent=None):
        super().__init__(parent)
        self._image = None
        # (label, confidence, x1, y1, x2, y2, z or None) per box
        self._detections = ()
        self._status = "Waiting for camera..."
        self._live = False
        self._alert = True
        self._font = QFont("Sans Serif", 8)
        self.setAttribute(Qt.WA_OpaquePaintEvent)

    def set_frame(self, image, detections, status, live, alert):
        # live: the frame is current (a stale one is dimmed).
        # alert: something needs the operator's attention (status line turns red).
        self._image = image
        self._detections = detections
        self._status = status
        self._live = live
        self._alert = alert
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.black)
        painter.setFont(self._font)
        metrics = QFontMetrics(self._font)

        if self._image is not None:
            image_w, image_h = self._image.width(), self._image.height()
            scale = min(self.width() / image_w, self.height() / image_h)
            offset_x = (self.width() - image_w * scale) / 2.0
            offset_y = (self.height() - image_h * scale) / 2.0
            target = QRectF(offset_x, offset_y, image_w * scale, image_h * scale)
            painter.setRenderHint(QPainter.SmoothPixmapTransform)
            painter.drawImage(target, self._image)

            for label, confidence, x1, y1, x2, y2, z in self._detections:
                box = QRectF(
                    offset_x + x1 * scale,
                    offset_y + y1 * scale,
                    (x2 - x1) * scale,
                    (y2 - y1) * scale,
                )
                painter.setPen(QPen(self.BOX_COLOR, 2))
                painter.setBrush(Qt.NoBrush)
                painter.drawRect(box)

                text = f"{label} {confidence:.2f}"
                if z is not None:
                    text += f"  z={z:.2f} m"
                text_w = metrics.horizontalAdvance(text) + 6
                text_h = metrics.height() + 2
                # Above the box, or just inside it when the box touches the top edge.
                text_y = box.top() - text_h if box.top() - text_h >= target.top() else box.top()
                text_rect = QRectF(box.left(), text_y, text_w, text_h)
                painter.fillRect(text_rect, QColor(0, 0, 0, 160))
                painter.setPen(self.BOX_COLOR)
                painter.drawText(text_rect, Qt.AlignCenter, text)

            if not self._live:
                # Dim a frozen frame so it cannot be mistaken for live video.
                painter.fillRect(target, QColor(0, 0, 0, 140))

        status_h = metrics.height() + 4
        status_rect = QRectF(0, self.height() - status_h, self.width(), status_h)
        painter.fillRect(status_rect, QColor(0, 0, 0, 170))
        painter.setPen(self.ALERT_COLOR if self._alert else Qt.white)
        painter.drawText(status_rect.adjusted(6, 0, -6, 0), Qt.AlignVCenter | Qt.AlignLeft, self._status)


def _siren_pcm(sweeps=3, sweep_s=0.4, low_hz=700.0, high_hz=1400.0, level=0.6):
    """A rising-falling siren (1.2 s by default) as mono s16le PCM at SIREN_RATE_HZ."""
    t = np.arange(int(SIREN_RATE_HZ * sweep_s)) / SIREN_RATE_HZ
    # Frequency rises then falls within each sweep; phase is its running integral, so
    # the tone glides without clicks.
    freq = np.tile(low_hz + (high_hz - low_hz) * (1.0 - np.abs(2.0 * t / sweep_s - 1.0)), sweeps)
    wave = np.sin(2.0 * np.pi * np.cumsum(freq) / SIREN_RATE_HZ)
    fade = int(0.01 * SIREN_RATE_HZ)  # 10 ms ramps: no pop at start/end
    envelope = np.ones_like(wave)
    envelope[:fade] = np.linspace(0.0, 1.0, fade)
    envelope[-fade:] = np.linspace(1.0, 0.0, fade)
    return (wave * envelope * level * 32767).astype('<i2').tobytes()


class AudioAnnouncer(QObject):
    """Plays announcements (optional siren, then speech) one at a time, off the GUI thread.

    Every step is a QProcess, so nothing here blocks the GUI. Each announcement belongs
    to a source ("optitrack", "wifi"):
      * a newer announcement from the SAME source replaces that source's queued one and
        cuts off its playing one. Otherwise a quick loss-then-recovery would play
        "OptiTrack normal" during the siren and then the stale "No OptiTrack" after it.
      * announcements from DIFFERENT sources queue, so a WiFi alarm never cuts off an
        OptiTrack alarm (they tend to fire together when the link drops).
    """

    # A hung player or speech-dispatcher must not stall the queue. A step that overruns
    # is killed and the announcement carries on (siren -> speech -> next), so a broken
    # siren still lets the words through.
    STEP_TIMEOUT_MS = 10000

    def __init__(self, parent=None):
        super().__init__(parent)
        self._queue = deque()     # (source, text, siren) waiting to play
        self._current = None      # (source, text, siren) playing now
        self._process = None      # QProcess of the current step
        self._on_done = None      # what follows the current step
        self._speaking = False
        # Bumped whenever a step is abandoned, so its late `finished` is ignored.
        self._generation = 0
        self._siren = _siren_pcm() if SIREN_PLAYER is not None else None
        self._watchdog = QTimer(self)
        self._watchdog.setSingleShot(True)
        self._watchdog.timeout.connect(self._step_timed_out)

    def announce(self, source, text, siren=False):
        self._queue = deque(item for item in self._queue if item[0] != source)
        if self._current is not None and self._current[0] == source:
            self._stop_current()
        self._queue.append((source, text, siren))
        if self._current is None:
            self._start_next()

    def _start_next(self):
        if not self._queue:
            return
        self._current = self._queue.popleft()
        if self._current[2] and self._siren is not None:
            self._run(SIREN_PLAYER, self._siren, self._speak_current)
        else:
            self._speak_current()

    def _speak_current(self):
        if SPEECH_COMMAND is None:
            self._finish_current()
            return
        self._speaking = True
        # -w: exit when the text has been spoken, which is what sequences the queue.
        self._run([SPEECH_COMMAND, '-w', self._current[1]], None, self._finish_current)

    def _finish_current(self):
        self._current = None
        self._speaking = False
        self._start_next()

    def _run(self, command, stdin_data, on_done):
        self._generation += 1
        generation = self._generation
        process = QProcess(self)

        def done(*_):
            process.deleteLater()
            if generation == self._generation:
                self._watchdog.stop()
                self._process = None
                on_done()

        def failed(error):
            # A program that cannot start never emits `finished`.
            if error == QProcess.FailedToStart:
                done()

        process.finished.connect(done)
        process.errorOccurred.connect(failed)
        self._process = process
        self._on_done = on_done
        self._watchdog.start(self.STEP_TIMEOUT_MS)
        process.start(command[0], command[1:])
        if stdin_data is not None:
            process.write(stdin_data)
            process.closeWriteChannel()

    def _kill_step(self):
        self._generation += 1
        self._watchdog.stop()
        if self._process is not None:
            self._process.kill()  # its `finished` still fires, and just deletes it
            self._process = None
        if self._speaking:
            # The text is already with speech-dispatcher; killing the client does not
            # reliably stop it, so cancel explicitly.
            QProcess.startDetached(SPEECH_COMMAND, ['-C'])

    def _stop_current(self):
        self._kill_step()
        self._current = None
        self._speaking = False

    def _step_timed_out(self):
        on_done = self._on_done
        self._kill_step()
        on_done()


class SingleDroneRosNode(Node, QObject):
    ## define signals
    update_data = pyqtSignal(int)
    direct_mode_result = pyqtSignal(bool, str)
    # Controller discovery/activation. `controllers_listed` carries
    # (success, message, [(name, description, active, selectable, reason), ...]);
    # plain tuples rather than ROS messages so nothing ROS-typed crosses into Qt slots.
    controllers_listed = pyqtSignal(bool, str, list)
    controller_activated = pyqtSignal(bool, str, str)

    def __init__(self):
        Node.__init__(self, 'single_drone_gui_node')
        QObject.__init__(self)
        self.data_struct = Common.CommonData()
        
        # Define QoS profile for PX4 topics (best effort reliability)
        self.px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Define QoS profile for PX4 input topics (commands to PX4)
        self.px4_input_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.controller_type_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Onboard vision stream. The image publisher is BEST_EFFORT, so a RELIABLE reader
        # would not match it at all; depth 1 because only the newest frame is ever shown
        # and a queue of older ones over WiFi is just latency. Detections are published
        # RELIABLE -- a BEST_EFFORT reader still matches, and a dropped set only costs
        # one frame its exact boxes (see CommonData.match_vision_detections).
        self.vision_image_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vision_detections_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # vrpn_mocap publishes best-effort/volatile; only arrival matters, so depth 1.
        self.optitrack_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # system_monitor publishes RELIABLE. A best-effort reader still matches, and it
        # means the Orin never retransmits stale reports to us over a link that is
        # already stalling -- the case the WiFi indicator exists to show.
        self.system_status_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Define subscribers
        self.imu_sub = self.create_subscription(VehicleAttitude, '/uav_0/fmu/out/vehicle_attitude', self.imu_callback, self.px4_qos_profile)
        self.pos_global_sub = self.create_subscription(VehicleGlobalPosition, '/uav_0/fmu/out/vehicle_global_position', self.pos_global_callback, self.px4_qos_profile)
        # One subscription, not two. Pose and twist arrive in the same Odometry message,
        # so subscribing twice deserialised every message twice for nothing -- 118 of the
        # 670 callbacks/s measured in DIRECT on 2026-08-03. See docs/gui_responsiveness.md.
        self.pos_local_adjusted_sub = self.create_subscription(Odometry, '/uav_0/state_estimator/local_position/odom', self.odom_callback, 10)
        self.bat_sub = self.create_subscription(BatteryStatus, '/uav_0/fmu/out/battery_status', self.bat_callback, self.px4_qos_profile)
        self.status_sub = self.create_subscription(VehicleStatus, '/uav_0/fmu/out/vehicle_status_v1', self.status_callback, self.px4_qos_profile)
        self.commanded_attitude_sub = self.create_subscription(VehicleAttitudeSetpoint, '/uav_0/fsc_autopilot_ros2/attitude_setpoint_debug', self.commanded_attitude_callback, self.px4_input_qos_profile)
        self.commanded_bodyrate_callback = self.create_subscription(VehicleRatesSetpoint, '/uav_0/fsc_autopilot_ros2/rate_setpoint_debug', self.commanded_bodyrate_callback, self.px4_input_qos_profile)
        self.estimator_type_sub = self.create_subscription(Bool, '/estimator_type', self.estimator_type_callback, 10)
        self.vehicle_info_sub = self.create_subscription(VehicleInfo, '/uav_0/fsc_autopilot_ros2/vehicle_info', self.vehicle_info_callback, 10)
        self.estimator_status_flags_sub = self.create_subscription(EstimatorStatusFlags, '/uav_0/fmu/out/estimator_status_flags', self.estimator_status_flags_callback, self.px4_qos_profile)
        self.controller_type_sub = self.create_subscription(
            String,
            '/uav_0/fsc_autopilot_ros2/controller_type',
            self.controller_type_callback,
            self.controller_type_qos_profile
        )
        # Per-rotor commands, for the "N/T for each rotor" widget. Each
        # direct-actuation node variant publishes this under its OWN service
        # namespace, so subscribe to every known one -- only one control node
        # ever runs at a time, so the extra subscriptions simply stay silent.
        # There are exactly FOUR such namespaces across the seven forks, and
        # forks SHARE them in pairs (AM + bare-drone), so this list is shorter
        # than the fork count:
        #   direct_actuation             classic AM + classic drone
        #   geometric_direct_actuation   geometric AM + geometric drone
        #   geometric_l1_direct_actuation   geometric+L1 AM + geometric+L1 drone
        #   whole_body_direct_actuation  whole-body AM
        # A fork missing from this list shows up as N/T pies pinned at 0.0%
        # for a whole DIRECT flight with nothing else wrong -- the other two
        # gates below (the "Direct Actuation" substring test on
        # controller_type, and `connected`) pass fine, so there is no error
        # anywhere, just a silent zero. Diagnose with `ros2 topic info` on the
        # fork's motors_debug: "Publisher count: 1, Subscription count: 0".
        # Happened twice -- whole_body added 2026-08-23, geometric_l1 added
        # 2026-08-24. When a NEW fork is added, add its namespace here.
        self.motor_commands_subs = [
            self.create_subscription(
                ActuatorMotors,
                topic,
                self.motor_commands_callback,
                10
            )
            for topic in (
                '/uav_0/fsc_autopilot_ros2/direct_actuation/motors_debug',
                '/uav_0/fsc_autopilot_ros2/geometric_direct_actuation/motors_debug',
                '/uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/motors_debug',
                '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/motors_debug',
            )
        ]
        # Back-compat alias: some code/logging referred to the single handle.
        self.motor_commands_sub = self.motor_commands_subs[0]
        self.position_error_sub = self.create_subscription(
            PositionControllerState,
            '/uav_0/fsc_autopilot_ros2/position_controller/state',
            self.position_error_callback,
            10
        )
        self.vision_image_sub = self.create_subscription(
            CompressedImage,
            VISION_IMAGE_TOPIC,
            self.vision_image_callback,
            self.vision_image_qos_profile
        )
        self.vision_detections_sub = self.create_subscription(
            String,
            VISION_DETECTIONS_TOPIC,
            self.vision_detections_callback,
            self.vision_detections_qos_profile
        )
        # OptiTrack liveness. NOT /uav_0/mocap: during a VRPN dropout
        # fsc_optitrack_processor_ros2 keeps republishing the held pose there at its
        # timer rate, and the estimator's /uav_0/mocap_status watches /uav_0/mocap, so
        # both keep reading "normal" with OptiTrack gone. The raw VRPN stream is the one
        # that actually stops -- and it is what the processor's own dropout detector uses.
        # raw=True: only the arrival time is used, so skip deserialising ~120 frames/s.
        self.optitrack_sub = self.create_subscription(
            PoseStamped,
            OPTITRACK_TOPIC,
            self.optitrack_callback,
            self.optitrack_qos_profile,
            raw=True
        )
        self.system_status_sub = self.create_subscription(
            String,
            SYSTEM_STATUS_TOPIC,
            self.system_status_callback,
            self.system_status_qos_profile
        )

        # Define publishers / services
        # self.coords_pub = self.create_publisher(TrackingReference, 'position_controller/target', 10)
        self.geofence_pub = self.create_publisher(Marker, 'tracking_controller/geofence', 10)
        self.position_com_pub = self.create_publisher(
            PositionControllerReference, '/uav_0/fsc_autopilot_ros2/position_controller/reference', 10)
        self.direct_mode_client = self.create_client(
            SetBool,
            '/uav_0/fsc_autopilot_ros2/direct_actuation/set_direct_mode'
        )
        # Generic controller discovery/activation. Advertised by whichever autopilot
        # node is running; currently only the direct-actuation node implements it, so
        # both clients simply stay not-ready under the baseline stack and the tab
        # reports "unavailable" instead of failing.
        self.list_controllers_client = self.create_client(
            ListControllers,
            '/uav_0/fsc_autopilot_ros2/list_controllers'
        )
        self.activate_controller_client = self.create_client(
            ActivateController,
            '/uav_0/fsc_autopilot_ros2/activate_controller'
        )

        # self.set_home_service = self.create_client(CommandHome, 'mavros/cmd/set_home')

        # self.arming_service = self.create_client(CommandLong, 'mavros/cmd/command')
        # self.land_service = self.create_client(CommandLong, 'mavros/cmd/command')
        # self.set_mode_service = self.create_client(SetMode, 'mavros/set_mode')

        # Timer for main loop (will be started when thread runs)
        self.timer = None

        # Requests handed over from the Qt GUI thread. rclpy client objects are NOT
        # thread-safe: calling call_async()/service_is_ready() from the GUI thread while
        # executor.spin() drives the same client in this thread blocks the GUI on the
        # client's internal lock for up to ~1 s. Everything rclpy-touching therefore
        # happens in timer_callback(), which runs in the ROS thread.
        self._pending_requests = deque()
        self._request_lock = threading.Lock()
        # Cached service availability, refreshed here so the GUI never queries the ROS
        # graph from its own thread (also up to ~1 s under contention).
        self._services_ready_cache = False
        self._services_ready_checked = 0.0

        # In-flight controller service calls and their deadlines (see
        # _check_controller_deadlines).
        self._list_future = None
        self._list_deadline = None
        self._activate_future = None
        self._activate_deadline = None

        # read geofence from json file
        with open('src/ROS_Node/geofence.json') as f:
            geofence = json.load(f)
            self.config = [0, 0, 0]
            self.config[0] = geofence['x']
            self.config[1] = geofence['y']
            self.config[2] = geofence['z']
        
    ### define signal connections to / from gui ###
    def connect_update_gui(self, callback):
        self.update_data.connect(callback)

    ### define callback functions from ros topics ###
    def imu_callback(self, msg): 
        # get orientation and convert to euler angles
        # note that uses PX4 [w, x, y, z] 
        self.data_struct.update_imu(msg.q[1], msg.q[2], msg.q[3], msg.q[0]) 
        
    def pos_global_callback(self, msg):
        self.data_struct.update_global_pos(msg.lat, msg.lon, msg.alt)
    
    def odom_callback(self, msg):
        # Pose and twist come from the one message; splitting this across two
        # subscriptions doubled the deserialisation cost for identical data.
        self.data_struct.update_local_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.data_struct.update_vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)

    def bat_callback(self, msg):
        self.data_struct.update_bat(msg.remaining, msg.voltage_v)

    def status_callback(self, msg):
        self.data_struct.update_state(msg.pre_flight_checks_pass,msg.arming_state, msg.nav_state, (msg.timestamp-msg.armed_time))

    def commanded_attitude_callback(self, msg):
        # the attitude setpoint received from px4 
        self.data_struct.update_attitude_target(msg.q_d[1], msg.q_d[2], msg.q_d[3], msg.q_d[0], msg.thrust_body[2])

    def commanded_bodyrate_callback(self, msg):
        self.data_struct.update_body_rate_target(msg.roll, msg.pitch, msg.yaw, msg.thrust_body[2])

    def estimator_type_callback(self, msg):
        self.data_struct.update_estimator_type(msg.data)

    def vehicle_info_callback(self, msg):
        # info[0] holds the vehicle name (see fsc_autopilot_ros2_msgs/msg/VehicleInfo)
        if msg.info:
            self.data_struct.update_vehicle_name(msg.info[0])

    def estimator_status_flags_callback(self, msg):
        self.data_struct.update_yaw_align(msg.cs_yaw_align)

    def controller_type_callback(self, msg):
        self.data_struct.update_controller_type(msg.data)

    def motor_commands_callback(self, msg):
        commands = [
            float(value) if math.isfinite(value) else 0.0
            for value in msg.control[:4]
        ]
        self.data_struct.update_motor_commands(commands)

    def position_error_callback(self, msg):
        self.data_struct.update_position_error(
            msg.position_error.x,
            msg.position_error.y,
            msg.position_error.z
        )

    def optitrack_callback(self, _serialized_msg):
        self.data_struct.update_optitrack()

    def system_status_callback(self, msg):
        # JSON from fsc_system_monitor: {"stamp", "wifi": {...}, "cpu": {...},
        # "mocap": {...}, "odom": {...}}; any field may be null. Parsed here so the GUI
        # thread only ever sees a dict.
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                raise ValueError('not a JSON object')
        except ValueError as e:
            self.get_logger().warn(
                f'Unparseable {SYSTEM_STATUS_TOPIC} message: {e}',
                throttle_duration_sec=5.0)
            return
        self.data_struct.update_system_status(status)

    def vision_image_callback(self, msg):
        # Stored as received; decoding is left to the GUI, which only does it while the
        # camera view is on screen (_update_camera_view).
        stamp = msg.header.stamp
        self.data_struct.update_vision_image(
            bytes(msg.data), stamp.sec * 1_000_000_000 + stamp.nanosec)

    def vision_detections_callback(self, msg):
        # std_msgs/String carrying JSON from the onboard detector:
        #   {"stamp": {"sec", "nanosec"}, "frame_id", "count",
        #    "detections": [{"label", "confidence", "bbox": [x1, y1, x2, y2],
        #                    "position": [x, y, z]}]}
        # bbox is in pixels of the published color image; position is metres in
        # camera_color_optical_frame (x right, y down, z forward), so z is range.
        # "stamp" is the stamp of the color frame the boxes were computed on.
        try:
            data = json.loads(msg.data)
            stamp = data['stamp']
            stamp_ns = int(stamp['sec']) * 1_000_000_000 + int(stamp['nanosec'])
            detections = []
            for det in data.get('detections', []):
                bbox = det.get('bbox')
                if not bbox or len(bbox) != 4:
                    continue
                x1, y1, x2, y2 = (float(v) for v in bbox)
                position = det.get('position')
                z = float(position[2]) if position and len(position) == 3 else None
                detections.append(
                    (str(det.get('label', '?')), float(det.get('confidence', 0.0)),
                     x1, y1, x2, y2, z))
        except (ValueError, KeyError, TypeError, AttributeError) as e:
            self.get_logger().warn(
                f'Unparseable {VISION_DETECTIONS_TOPIC} message: {e}',
                throttle_duration_sec=5.0)
            return
        self.data_struct.update_vision_detections(stamp_ns, tuple(detections))

    def publish_coordinates(self, x, y, z, yaw):
        msg = PositionControllerReference()
        now = self.get_clock().now().to_msg()  # builtin_interfaces/Time
        msg.header.stamp = now
        msg.header.frame_id = "ground"
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw
        msg.yaw_unit = PositionControllerReference.DEGREES
        self.position_com_pub.publish(msg)
        self.get_logger().info(f"Publishing coordinates: {x}, {y}, {z}, {yaw}")
        # self.coords_pub.publish(point)

    def direct_mode_service_ready(self):
        return self.direct_mode_client.service_is_ready()

    def request_direct_mode(self, direct):
        if not self.direct_mode_service_ready():
            self.direct_mode_result.emit(
                False,
                "Direct-actuation mode service is unavailable"
            )
            return

        request = SetBool.Request()
        request.data = direct
        future = self.direct_mode_client.call_async(request)
        future.add_done_callback(self._direct_mode_response_callback)

    def _direct_mode_response_callback(self, future):
        try:
            response = future.result()
            self.direct_mode_result.emit(response.success, response.message)
        except Exception as exc:
            self.direct_mode_result.emit(False, f"Mode switch service failed: {exc}")

    # ------------------------------------------------------------------
    # Controller discovery / activation
    #
    # Both calls are asynchronous with a finite deadline, enforced in
    # _check_controller_deadlines() off the existing 30 Hz timer. rclpy's
    # call_async has no timeout of its own, so without this an autopilot node that
    # accepts the connection and then never answers would leave the Controller tab
    # stuck on "Requesting..." forever.
    # ------------------------------------------------------------------
    CONTROLLER_SERVICE_TIMEOUT_S = 3.0

    def controller_services_ready(self):
        """Cached — safe to call from the Qt thread at GUI rate."""
        return self._services_ready_cache

    def _refresh_services_ready(self):
        """ROS-thread only. Graph queries are slow and lock-contended."""
        now = time.monotonic()
        if now - self._services_ready_checked < 0.5:
            return
        self._services_ready_checked = now
        self._services_ready_cache = (
            self.list_controllers_client.service_is_ready()
            and self.activate_controller_client.service_is_ready())

    # -- called FROM THE QT THREAD: enqueue only, never touch rclpy ---------
    def queue_controller_list(self):
        with self._request_lock:
            self._pending_requests.append(("list", None))

    def queue_controller_activation(self, name):
        with self._request_lock:
            self._pending_requests.append(("activate", name))

    def queue_coordinates(self, x, y, z, yaw):
        # Same reason as the two above: publish_coordinates() touches the clock, a
        # publisher and the logger, and doing that from the Qt thread while the executor
        # spins means contending with it for the rclpy locks. The controller buttons were
        # moved onto this queue when the switch button stalled ~1 s; this one was missed.
        with self._request_lock:
            self._pending_requests.append(("coords", (x, y, z, yaw)))

    def _drain_requests(self):
        """ROS-thread only: issue whatever the GUI queued."""
        while True:
            with self._request_lock:
                if not self._pending_requests:
                    return
                kind, arg = self._pending_requests.popleft()
            if kind == "list":
                self.request_controller_list()
            elif kind == "activate":
                self.request_controller_activation(arg)
            elif kind == "coords":
                self.publish_coordinates(*arg)

    def request_controller_list(self):
        if not self.list_controllers_client.service_is_ready():
            self.controllers_listed.emit(
                False, "Controller discovery service is unavailable", [])
            return
        future = self.list_controllers_client.call_async(ListControllers.Request())
        self._list_future = future
        self._list_deadline = time.monotonic() + self.CONTROLLER_SERVICE_TIMEOUT_S
        future.add_done_callback(self._controller_list_response)

    def _controller_list_response(self, future):
        self._list_future = None
        self._list_deadline = None
        try:
            response = future.result()
        except Exception as exc:
            self.controllers_listed.emit(False, f"Discovery failed: {exc}", [])
            return
        rows = [
            (c.name, c.description, c.active, c.selectable, c.reason)
            for c in response.controllers
        ]
        self.controllers_listed.emit(response.success, response.message, rows)

    def request_controller_activation(self, name):
        if not self.activate_controller_client.service_is_ready():
            self.controller_activated.emit(
                False, "Controller activation service is unavailable", "")
            return
        request = ActivateController.Request()
        request.name = name
        future = self.activate_controller_client.call_async(request)
        self._activate_future = future
        self._activate_deadline = time.monotonic() + self.CONTROLLER_SERVICE_TIMEOUT_S
        future.add_done_callback(self._controller_activate_response)

    def _controller_activate_response(self, future):
        self._activate_future = None
        self._activate_deadline = None
        try:
            response = future.result()
        except Exception as exc:
            self.controller_activated.emit(False, f"Activation failed: {exc}", "")
            return
        self.controller_activated.emit(
            response.success, response.message, response.active)

    def _check_controller_deadlines(self):
        now = time.monotonic()
        if self._list_deadline is not None and now > self._list_deadline:
            self._list_deadline = None
            if self._list_future is not None:
                self._list_future.cancel()
                self._list_future = None
            self.controllers_listed.emit(
                False,
                f"Discovery timed out after {self.CONTROLLER_SERVICE_TIMEOUT_S:.0f}s",
                [])
        if self._activate_deadline is not None and now > self._activate_deadline:
            self._activate_deadline = None
            if self._activate_future is not None:
                self._activate_future.cancel()
                self._activate_future = None
            # Deliberately does NOT claim the switch failed -- the request may have
            # been received and acted on. The GUI re-reads live status instead.
            self.controller_activated.emit(
                False,
                f"No response after {self.CONTROLLER_SERVICE_TIMEOUT_S:.0f}s; "
                "confirm the active controller from status",
                "")

    def publish_geofence(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "geofence"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        corners = [(x, y, 0), (x, -y, 0),  (-x, -y, 0),  (-x, y, 0), (x, y, z), (x, -y, z),(-x, -y, z), (-x, y, z)]
        edges = [
            (corners[0], corners[1]), (corners[1], corners[2]), (corners[2], corners[3]), (corners[3], corners[0]),  # Bottom face
            (corners[4], corners[5]), (corners[5], corners[6]), (corners[6], corners[7]), (corners[7], corners[4]),  # Top face
            (corners[0], corners[4]), (corners[1], corners[5]), (corners[2], corners[6]), (corners[3], corners[7])   # Vertical edges
        ]

        # create points for each edge
        for edge in edges:
            start, end = edge   # define edge
            point_start = Point()
            point_start.x, point_start.y, point_start.z = start # start points
            point_end = Point()
            point_end.x, point_end.y, point_end.z = end         # end points
            marker.points.append(point_start)
            marker.points.append(point_end)

        print("Geofence published")
        self.geofence_pub.publish(marker)

    # Timer callback for main loop
    def timer_callback(self):
        # Runs in the ROS thread (created in run()), so this is the only safe place to
        # touch rclpy clients.
        self._refresh_services_ready()
        self._drain_requests()
        self._check_controller_deadlines()
        self.update_data.emit(0)
    
    # main loop of ros node (for compatibility with thread)
    def run(self):
        # Start the timer when the thread begins
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 Hz
        
        # Use executor to spin in this thread
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        
        try:
            executor.spin()
        except Exception as e:
            print(f"ROS executor error: {e}")
        finally:
            executor.shutdown()
            self.destroy_node()

class SingleDroneRosThread(QObject):
    def __init__(self, ui):
        QObject.__init__(self)
        self.ros_object = SingleDroneRosNode()
        self.thread = QThread()

        # setup signals
        self.ui = ui
        self.set_ros_callbacks()

        # last-seen state used to detect/log transitions in update_gui_data;
        # None means "not observed yet" so the first sample never logs a
        # spurious transition from the CommonData defaults.
        self._prev_armed = None
        self._prev_mode = None
        self._prev_yaw_align = None
        self._prev_controller_type = None
        self._yaw_align = False
        self._controller_type = ""
        self._mode_request_pending = False

        # Controller tab state. `_controllers` mirrors the last successful
        # list_controllers response as (name, description, active, selectable,
        # reason) tuples; empty until a discovery call succeeds. A failed or
        # timed-out refresh deliberately leaves the previous list on screen
        # (AGENTS.md) rather than blanking the table.
        self._controllers = []
        self._controller_request_pending = False
        # Last observed service availability, so _update_controller_switch can tell a
        # real change from the 30 Hz no-op case.
        self._controller_services_ready = False
        # Name whose activation is in flight, so the result can be reported against
        # what was actually asked for rather than against the current selection.
        self._pending_activation = None

        # Plot redraw decimation: data is appended every GUI tick (30 Hz) so nothing is
        # lost, but the curves are re-drawn only every Nth tick.
        #
        # This is NOT what fixed the lag -- measured 2026-08-04, decimating 30 -> 3 Hz
        # changed the dropped-frame rate not at all. The fix was giving the plots a fixed
        # relative-time x-axis (see _setup_position_plot_impl). Decimation is kept only
        # because it cuts plot CPU ~3x for free and 10 Hz is well past what an operator
        # can read. Do not cite it as the fix.
        self.PLOT_REDRAW_EVERY = 3
        self._plot_tick = 0

        # inertial-position and body-angle plot data buffers
        self._plot_t0_pos = None
        self._plot_t_pos = deque()
        self._plot_x = deque()
        self._plot_y = deque()
        self._plot_z = deque()
        self._plot_roll = deque()
        self._plot_pitch = deque()
        self._plot_yaw = deque()
        self._plot_yaw_cmd = deque()
        self._plot_err_x = deque()
        self._plot_err_y = deque()
        self._plot_err_z = deque()
        self._plot_x_cmd = deque()
        self._plot_y_cmd = deque()
        self._plot_z_cmd = deque()
        # last setpoint actually sent via send_coordinates(); held constant
        # between sends so the dashed command lines stay flat until updated
        self._last_x_cmd = 0.0
        self._last_y_cmd = 0.0
        self._last_z_cmd = 0.0
        self._last_yaw_cmd = 0.0
        self._pref_waiting = True
        self._pref_recording = False
        self._pref_armed = False
        self._pref_t0 = None
        self._pref_window_s = STEP_RESPONSE_DEFAULT_WINDOW_S
        self._pref_command = (0.0, 0.0, 0.0)
        self._pref_t = deque()
        self._pref_x = deque()
        self._pref_y = deque()
        self._pref_z = deque()
        self._pref_x_cmd = deque()
        self._pref_y_cmd = deque()
        self._pref_z_cmd = deque()
        self._setup_position_plot()
        self._setup_body_angle_plot()
        self._setup_step_response_plot()
        self._setup_step_response_controls()
        self._setup_motor_display()
        self._setup_camera_view()
        self._setup_optitrack_status()
        self._setup_system_status()
        self._setup_controller_switch()

        # Move ROS node to thread and start
        self.ros_object.moveToThread(self.thread)
        self.lock = self.ros_object.data_struct.lock
        self.thread.started.connect(self.ros_object.run)

        # set geofence
        self.ui.Geofence_X.display(self.ros_object.config[0])
        self.ui.Geofence_Y.display(self.ros_object.config[1])
        self.ui.Geofence_Z.display(self.ros_object.config[2])
        # while self.ros_object.geofence_pub.get_num_connections() < 1:
        #     rate = rclpy.Rate(1)
        #     print("Waiting for Rviz to connect to geofence publisher")
        #     rate.sleep()
        # self.ros_object.publish_geofence(int(self.ros_object.config[0]), int(self.ros_object.config[1]), int(self.ros_object.config[2]))


    def start(self):
        self.thread.start()
    
    def log_message(self, message):
        """Add timestamped message to GUI logging widget"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        formatted_message = f"[{timestamp}] {message}"
        widget = self.ui.list_cmd_log
        widget.addItem(formatted_message)
        # Drop the oldest entries past the cap. takeItem() detaches the item; deleting
        # the reference is what actually frees it, since Qt no longer owns it.
        while widget.count() > LOG_MAX_LINES:
            del_item = widget.takeItem(0)
            del del_item
        widget.scrollToBottom()

    def _setup_motor_display(self):
        container = self.ui.display_quad_rotor_NT
        self._motor_display = QuadrotorThrottleWidget(container)
        self._motor_display.setGeometry(container.rect())
        self._motor_display.set_commands((0.0, 0.0, 0.0, 0.0))
        self._motor_display.show()

    def _setup_camera_view(self):
        container = self.ui.cam_vision_0
        self._camera_view = CameraDetectionView(container)
        self._camera_view.setGeometry(container.rect())
        self._camera_view.show()
        # Decoded frame and the seq it came from, so each JPEG is decoded once.
        self._camera_image = None
        self._camera_image_seq = 0
        self._camera_decode_failed = False
        # What the view currently shows; repaint only when this changes.
        self._camera_shown = None

    def _update_camera_view(self):
        # Hidden tab: skip everything. Frames keep landing in CommonData as raw bytes,
        # so the view is current again on the first tick after it is shown.
        if not self._camera_view.isVisible():
            return
        if not self.lock.tryLock():
            return
        data = self.ros_object.data_struct
        seq = data.vision_image_seq
        image_bytes = data.current_vision_image
        detection_key, detections = data.match_vision_detections(data.current_vision_stamp_ns)
        image_time = data.last_vision_image_time
        detections_time = data.last_vision_detections_time
        self.lock.unlock()

        if seq != self._camera_image_seq and image_bytes is not None:
            self._camera_image_seq = seq
            # Format sniffed from the data (JPEG, or PNG if the publisher changes).
            image = QImage.fromData(image_bytes)
            self._camera_decode_failed = image.isNull()
            if not self._camera_decode_failed:
                self._camera_image = image

        now = time.monotonic()
        image_age = now - image_time
        live = image_time > 0.0 and image_age < VISION_STALE_S and not self._camera_decode_failed
        alert = True
        if image_time == 0.0:
            status = f"Waiting for {VISION_IMAGE_TOPIC}"
        elif self._camera_decode_failed:
            # The boxes belong to the frame that failed, not the one still on screen.
            status = f"Cannot decode frames on {VISION_IMAGE_TOPIC}"
            detection_key, detections = None, ()
        elif not live:
            status = f"No video for {int(image_age)} s"
        elif detections_time == 0.0 or now - detections_time >= VISION_STALE_S:
            status = f"Detector silent ({VISION_DETECTIONS_TOPIC})"
        else:
            status = f"{len(detections)} object(s)"
            alert = False

        shown = (self._camera_image_seq, detection_key, status)
        if shown == self._camera_shown:
            return
        self._camera_shown = shown
        self._camera_view.set_frame(self._camera_image, detections, status, live, alert)

    def _set_status_label(self, label, text, style):
        # Only touch the widget on a change, and restyle only when the style changed:
        # setStyleSheet re-polishes the widget, which is not something to do at the
        # 30 Hz tick rate (or every second while a "for N s" counter runs).
        shown_text, shown_style = self._status_label_shown.get(label.objectName(), (None, None))
        if text != shown_text:
            label.setText(text)
        if style != shown_style:
            label.setStyleSheet(style)
        self._status_label_shown[label.objectName()] = (text, style)

    def _setup_optitrack_status(self):
        self._status_label_shown = {}
        # None until the first verdict, so startup announces the state once either way.
        self._optitrack_ok = None
        self._optitrack_started = time.monotonic()
        # No frames yet, so red from the start; only the announcement waits for the
        # startup grace period.
        self._set_status_label(self.ui.optitrack_status, "No OptiTrack", STATUS_STYLE['bad'])
        self._announcer = AudioAnnouncer(self)
        if SPEECH_COMMAND is None:
            print("[AUDIO] spd-say not found; spoken announcements disabled")
        if SIREN_PLAYER is None:
            print("[AUDIO] neither pacat nor aplay found; sirens disabled")

    def _update_optitrack_status(self):
        ok = self.ros_object.data_struct.optitrack_fresh(OPTITRACK_TIMEOUT_S)
        if (not ok and self._optitrack_ok is None
                and time.monotonic() - self._optitrack_started < OPTITRACK_STARTUP_GRACE_S):
            return
        # Edge-triggered, so each announcement is spoken once per transition.
        if ok == self._optitrack_ok:
            return
        was_ok = self._optitrack_ok
        self._optitrack_ok = ok
        if ok:
            self._set_status_label(
                self.ui.optitrack_status, "OptiTrack normal", STATUS_STYLE['good'])
            self._announcer.announce('optitrack', "OptiTrack normal")
            self.log_message("OptiTrack normal")
        else:
            self._set_status_label(
                self.ui.optitrack_status, "No OptiTrack", STATUS_STYLE['bad'])
            # Siren only for a real loss. The first verdict at launch (e.g. at the desk
            # with no mocap) is spoken but not sirened: an alarm that sounds on every
            # launch teaches people to ignore it.
            self._announcer.announce('optitrack', "No OptiTrack", siren=was_ok is True)
            self.log_message(
                f"No OptiTrack: nothing on {OPTITRACK_TOPIC} for {OPTITRACK_TIMEOUT_S:.1f} s")

    def _setup_system_status(self):
        self._system_status_seq = 0
        # Consecutive reports showing packets queued for WiFi (see wifi_summary).
        self._wifi_backlog_reports = 0
        self._wifi_level = None
        self._set_status_label(
            self.ui.wifi_status, "No WiFi\nwaiting for Orin status",
            STATUS_STYLE['bad'] + TWO_LINE_FONT)
        self._set_status_label(
            self.ui.cpu_status, "Orin CPU: waiting for status", "color: red;" + TWO_LINE_FONT)

    def _update_system_status(self):
        data = self.ros_object.data_struct
        if not self.lock.tryLock():
            return
        seq = data.system_status_seq
        status = data.current_system_status
        received = data.last_system_status_time
        self.lock.unlock()

        if seq != self._system_status_seq:
            self._system_status_seq = seq
            backlog = ((status.get('wifi') or {}).get('qdisc_backlog_max_pkts') or 0)
            self._wifi_backlog_reports = self._wifi_backlog_reports + 1 if backlog > 0 else 0

        age = time.monotonic() - received
        if received == 0.0 or age >= SYSTEM_STATUS_STALE_S:
            level = 'bad'
            since = "yet" if received == 0.0 else f"for {int(age)} s"
            wifi_text = f"No WiFi\nno status from Orin {since}"
            cpu_text, cpu_style = "Orin CPU: no status", "color: red;"
        else:
            level, wifi_text = wifi_summary(status.get('wifi'), self._wifi_backlog_reports)
            cpu_text = cpu_summary(status.get('cpu'))
            cpu_style = ""
            if cpu_text is None:
                cpu_text, cpu_style = "Orin CPU: no load data", "color: red;"

        self._set_status_label(self.ui.wifi_status, wifi_text, STATUS_STYLE[level] + TWO_LINE_FONT)
        self._set_status_label(self.ui.cpu_status, cpu_text, cpu_style + TWO_LINE_FONT)
        # Losing the link to the Orin is worth an alarm and a line in the flight log;
        # good <-> fair is not, since it can flip every second while the signal sits near
        # a threshold. Nothing fires before the first report, so launching without the
        # Orin stays quiet (the label is red) and startup never says "restored".
        if received > 0.0:
            if self._wifi_level is not None and (level == 'bad') != (self._wifi_level == 'bad'):
                if level == 'bad':
                    self._announcer.announce('wifi', "No WiFi", siren=True)
                    self.log_message(wifi_text.replace("\n", ": "))
                else:
                    self._announcer.announce('wifi', "WiFi restored")
                    self.log_message("WiFi restored")
            self._wifi_level = level

    # ------------------------------------------------------------------
    # Controller tab
    #
    # The table is populated from the node's list_controllers service, not
    # hardcoded: which modes exist is a property of whichever autopilot node is
    # running. Only the direct-actuation node implements the interface today, so
    # under the baseline stack the tab correctly reports it as unavailable.
    #
    # SAFE SWITCH -- the guard is deliberately ASYMMETRIC:
    #   * Entering a mode that takes control away from PX4 requires an explicit
    #     confirmation naming the consequence, and the button is disabled unless a
    #     genuinely selectable row is chosen and no request is in flight.
    #   * Returning to the safe/baseline mode is always one click, never gated on
    #     the table selection and never behind a dialog. An abort you have to
    #     confirm is an abort you cannot use.
    # Neither path arms the vehicle or changes the PX4 flight mode.
    # ------------------------------------------------------------------

    # A mode is treated as "gives this node authority PX4 would otherwise have" if
    # it is not the safe one. Matching on the safe name (rather than listing every
    # dangerous name) keeps a newly added mode guarded by default.
    SAFE_CONTROLLER_NAMES = ("Baseline (Safety)", "Baseline", "SAFETY")

    # DISPLAY aliases (2026-09-18, user request): the operator sees the three
    # roles -- Baseline, Decoupled (the geometric+L1 aerial-manipulator law of
    # Cai et al.), Whole-Body (the coupled law) -- while every service call and
    # the safe-name test keep using the registry's own names. A fork not
    # listed here is shown under its registry name unchanged.
    CONTROLLER_DISPLAY_ALIASES = {
        "Baseline (Safety)": "Baseline",
        "Geometric+L1 Direct Actuation": "Decoupled",
        "Whole-Body Direct Actuation": "Whole-Body",
    }

    @classmethod
    def _controller_display_name(cls, name):
        alias = cls.CONTROLLER_DISPLAY_ALIASES.get(name)
        return f"{alias} ({name})" if alias else (name or "Unknown")

    def _is_safe_controller(self, name):
        return name in self.SAFE_CONTROLLER_NAMES

    def _setup_controller_switch(self):
        table = self.ui.avaliable_controllers
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(("Mode", "Control owner", "Status"))
        table.setRowCount(0)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setSelectionMode(QAbstractItemView.SingleSelection)
        table.horizontalHeader().setStretchLastSection(True)
        table.itemSelectionChanged.connect(self._update_controller_buttons)
        self._render_controller_table()
        self._update_controller_buttons()

    def _render_controller_table(self):
        """Redraw the table from the last successful discovery response."""
        table = self.ui.avaliable_controllers
        prev = self._selected_controller_name()
        table.blockSignals(True)
        table.setRowCount(len(self._controllers))
        for row, (name, desc, active, selectable, reason) in enumerate(self._controllers):
            if active:
                status = "ACTIVE"
            elif selectable:
                status = "available"
            else:
                status = reason or "unavailable"
            for col, text in enumerate((self._controller_display_name(name), desc, status)):
                item = QTableWidgetItem(text)
                if col == 0:
                    item.setToolTip(f"registry name: {name}")
                if active:
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
                    item.setForeground(QBrush(QColor("#2f7d4f")))
                elif not selectable:
                    item.setForeground(QBrush(QColor("#999999")))
                table.setItem(row, col, item)
        table.blockSignals(False)

        # Keep the operator's selection across refreshes; otherwise fall back to the
        # first row that can actually be activated.
        if prev and self._row_for_name(prev) is not None:
            table.selectRow(self._row_for_name(prev))
        else:
            for row, entry in enumerate(self._controllers):
                if entry[3]:
                    table.selectRow(row)
                    break

    def _row_for_name(self, name):
        for row, entry in enumerate(self._controllers):
            if entry[0] == name:
                return row
        return None

    def _selected_controller_name(self):
        rows = self.ui.avaliable_controllers.selectionModel().selectedRows() \
            if self.ui.avaliable_controllers.selectionModel() else []
        if not rows:
            return None
        row = rows[0].row()
        if 0 <= row < len(self._controllers):
            return self._controllers[row][0]
        return None

    def _selected_controller_entry(self):
        name = self._selected_controller_name()
        row = self._row_for_name(name) if name else None
        return self._controllers[row] if row is not None else None

    def _update_controller_buttons(self):
        services_ready = self.ros_object.controller_services_ready()
        busy = self._controller_request_pending
        entry = self._selected_controller_entry()

        can_activate = bool(entry) and entry[3] and services_ready and not busy
        self.ui.buttom_activate_controller.setEnabled(can_activate)

        if entry and not self._is_safe_controller(entry[0]):
            # Dangerous direction: make the button look like what it does.
            self.ui.buttom_activate_controller.setText(f"Switch to {entry[0]}")
            self.ui.buttom_activate_controller.setStyleSheet(
                "background-color: #d9534f; color: white; font-weight: bold;"
                if can_activate else ""
            )
        else:
            self.ui.buttom_activate_controller.setText("Activate selected")
            self.ui.buttom_activate_controller.setStyleSheet("")

        # The abort path. Enabled whenever a non-safe controller is live and the
        # service is up -- independent of the table selection, and never disabled by
        # a pending activation, so it stays usable if a switch hangs.
        safe_name = self._safe_controller_name()
        back_enabled = (services_ready and safe_name is not None
                        and not self._is_safe_controller(self._controller_type))
        self.ui.buttom_back_to_baseline.setEnabled(back_enabled)
        self.ui.buttom_back_to_baseline.setStyleSheet(
            "background-color: #f0ad4e; font-weight: bold;" if back_enabled else ""
        )

    def _safe_controller_name(self):
        """Name of the safe/baseline mode as reported by the node, if it has one."""
        for entry in self._controllers:
            if self._is_safe_controller(entry[0]):
                return entry[0]
        return None

    def _update_controller_switch(self, controller_type):
        """Live status from the controller_type topic.

        Called every GUI tick (30 Hz), so it must do nothing when nothing changed:
        re-rendering the table on every tick would both burn cycles and fight the
        operator's row selection. Service availability is polled too, because the
        autopilot node can appear or disappear at any time.
        """
        services_ready = self.ros_object.controller_services_ready()
        changed = (controller_type != self._controller_type
                   or services_ready != self._controller_services_ready)
        self._controller_type = controller_type
        self._controller_services_ready = services_ready

        if not changed:
            return

        # Keep the table's ACTIVE marker honest between refreshes.
        if self._controllers:
            self._controllers = [
                (n, d, n == controller_type,
                 s if n != controller_type else False,
                 "already active" if n == controller_type else r)
                for (n, d, _a, s, r) in self._controllers
            ]
            self._render_controller_table()
        self._update_controller_buttons()

        # Discover once as soon as the node shows up, so the tab is populated
        # without the operator having to press Refresh first.
        if services_ready and not self._controllers and not self._controller_request_pending:
            self.ros_object.queue_controller_list()

    def _refresh_controller_switch(self):
        if not self.ros_object.controller_services_ready():
            self.log_message("Controller services unavailable "
                             "(is the direct-actuation node running?)")
            self._update_controller_buttons()
            return
        self.log_message("Querying available controllers...")
        self.ros_object.queue_controller_list()

    def _handle_controllers_listed(self, success, message, controllers):
        if not success:
            # Keep whatever was on screen; a failed refresh must not look like
            # "there are no controllers".
            self.log_message(f"Controller discovery failed: {message}")
            self._update_controller_buttons()
            return
        self._controllers = list(controllers)
        self._render_controller_table()
        self._update_controller_buttons()
        names = ", ".join(c[0] for c in self._controllers) or "none"
        self.log_message(f"Available controllers: {names}")

    def _request_controller_switch(self):
        if self._controller_request_pending:
            return
        entry = self._selected_controller_entry()
        if entry is None:
            self.log_message("Select a controller first")
            return
        name, _desc, _active, selectable, reason = entry
        if not selectable:
            self.log_message(f"'{name}' cannot be activated: {reason or 'unavailable'}")
            return

        if not self._is_safe_controller(name):
            answer = QMessageBox.warning(
                None,
                f"Switch to {name}?",
                f"'{name}' takes attitude, body rate and motor mixing away from PX4 "
                "— PX4 will run no controller or mixer.\n\n"
                "Continue only in the approved flight-test setup.\n"
                "'Return to baseline' remains available to abort.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                self.log_message(f"Switch to {name} cancelled")
                return

        self._begin_activation(name)

    def _request_back_to_baseline(self):
        """Dedicated abort path — no dialog, no dependence on the selection."""
        name = self._safe_controller_name()
        if name is None:
            self.log_message("No baseline/safety controller reported by the node")
            return
        if self._is_safe_controller(self._controller_type):
            self.log_message(f"Already in {self._controller_type}")
            return
        self._begin_activation(name)

    def _begin_activation(self, name):
        self._controller_request_pending = True
        self._pending_activation = name
        self.ui.buttom_activate_controller.setEnabled(False)
        self.ui.buttom_activate_controller.setText(f"Requesting {name}...")
        self.log_message(f"Requesting controller: {name}")
        self.ros_object.queue_controller_activation(name)

    def _handle_controller_activated(self, success, message, active):
        requested = self._pending_activation
        self._controller_request_pending = False
        self._pending_activation = None
        prefix = "Controller switch accepted" if success else "Controller switch rejected"
        self.log_message(f"{prefix}: {message}")
        # Trust the service's reported active mode, and the controller_type topic
        # after it — never assume the request took effect.
        if active:
            self._update_controller_switch(active)
        else:
            self._update_controller_buttons()
        if success and requested and active and active != requested:
            self.log_message(
                f"WARNING: requested '{requested}' but node reports '{active}'")
        # Re-read the list so selectable/reason reflect the new active mode.
        if self.ros_object.controller_services_ready():
            self.ros_object.queue_controller_list()

    def _handle_direct_mode_result(self, success, message):
        # Legacy SetBool path, kept so the direct-actuation runbook's
        # set_direct_mode calls still surface in the log.
        self._mode_request_pending = False
        prefix = "Mode switch accepted" if success else "Mode switch rejected"
        self.log_message(f"{prefix}: {message}")
        self._update_controller_buttons()

    # --- Real-time inertial X/Y/Z/yaw and position-error plots -------------------
    def _setup_position_plot(self):
        if not _HAS_PYQTGRAPH:
            print("[PLOT] pyqtgraph not found - position plot disabled")
            return
        try:
            self._setup_position_plot_impl()
            print("[PLOT] Position plot setup OK")
        except Exception as e:
            import traceback
            print(f"[PLOT] Position plot setup failed: {e}")
            traceback.print_exc()

    def _setup_position_plot_impl(self):
        pg.setConfigOptions(antialias=True)

        # Pin the PlotWidget to the exact pixel bounds of the container widget
        # (display_x_y_z in single_drone_flight.ui).
        container = self.ui.display_x_y_z
        self._pos_plot = pg.PlotWidget(parent=container)
        self._pos_plot.move(0, 0)
        self._pos_plot.setFixedSize(container.width(), container.height())
        self._pos_plot.show()

        self._pos_plot.setBackground('w')
        # Seconds-ago axis, fixed once here and never moved again. Profiling under real
        # DIRECT load (2026-08-04) put setXRange at 49% of ALL GUI-thread work -- 3 ms a
        # call, twice per redraw -- because moving the view re-runs axis layout, the SI
        # prefix check and a setHtml label re-render every tick. Plotting against relative
        # time keeps the view static and the data scrolling, so that whole cascade is gone.
        # Units are in the label text, not units=, for the same reason as the angle plot.
        self._pos_plot.setLabel('bottom', 'Time (s, relative)')
        self._pos_plot.getAxis('bottom').enableAutoSIPrefix(False)
        self._pos_plot.setXRange(-POSITION_PLOT_HISTORY_S, 0, padding=0)
        self._pos_plot.setLabel('left', 'Position', units='m')
        self._pos_plot.addLegend(offset=(5, -5))
        # Push plot content down so the top margin isn't clipped by the container edge
        self._pos_plot.getPlotItem().layout.setContentsMargins(0, 15, 0, 0)

        # X/Y/Z position (m) only: solid = actual, dashed = commanded. Yaw used to
        # share this plot on a second right-hand axis; it now lives on the body-angle
        # plot alongside roll and pitch, so a single left axis in metres is the whole
        # story here and the second ViewBox (and its resize syncing) is gone.
        dashed = Qt.DashLine
        self._curve_x = self._pos_plot.plot(pen=pg.mkPen('#CC0000', width=2), name='X (m)')
        self._curve_y = self._pos_plot.plot(pen=pg.mkPen('#008800', width=2), name='Y (m)')
        self._curve_z = self._pos_plot.plot(pen=pg.mkPen('#0055AA', width=2), name='Z (m)')
        self._curve_x_cmd = self._pos_plot.plot(
            pen=pg.mkPen('#CC0000', width=2, style=dashed), name='X cmd (m)')
        self._curve_y_cmd = self._pos_plot.plot(
            pen=pg.mkPen('#008800', width=2, style=dashed), name='Y cmd (m)')
        self._curve_z_cmd = self._pos_plot.plot(
            pen=pg.mkPen('#0055AA', width=2, style=dashed), name='Z cmd (m)')

    def _setup_body_angle_plot(self):
        """Body attitude in degrees: roll, pitch, yaw, plus the yaw command.

        Replaces the old position-error plot in this container. Only yaw has a
        command to compare against — roll and pitch are not commanded directly by
        the operator, they fall out of the position controller — so yaw is the only
        curve with a dashed counterpart.
        """
        if not _HAS_PYQTGRAPH:
            return
        container = self.ui.display_body_angle
        self._angle_plot = pg.PlotWidget(parent=container)
        self._angle_plot.setGeometry(container.rect())
        self._angle_plot.setBackground('w')
        # Fixed seconds-ago axis -- see the position plot for why moving it is costly.
        self._angle_plot.setLabel('bottom', 'Time (s, relative)')
        self._angle_plot.getAxis('bottom').enableAutoSIPrefix(False)
        self._angle_plot.setXRange(-POSITION_PLOT_HISTORY_S, 0, padding=0)
        # Degrees deliberately in the label text, not units='deg': pyqtgraph would
        # SI-prefix a units string and render "mdeg"/"kdeg" as the range changes.
        self._angle_plot.setLabel('left', 'Body angle (deg)')
        self._angle_plot.addLegend(offset=(5, -5))
        self._angle_plot.getPlotItem().layout.setContentsMargins(0, 15, 0, 0)

        dashed = Qt.DashLine
        self._curve_roll = self._angle_plot.plot(
            pen=pg.mkPen('#CC0000', width=2), name='Roll (deg)')
        self._curve_pitch = self._angle_plot.plot(
            pen=pg.mkPen('#008800', width=2), name='Pitch (deg)')
        self._curve_yaw = self._angle_plot.plot(
            pen=pg.mkPen('#AA00AA', width=2), name='Yaw (deg)')
        self._curve_yaw_cmd = self._angle_plot.plot(
            pen=pg.mkPen('#AA00AA', width=2, style=dashed), name='Yaw cmd (deg)')

        # Fixed range: all three angles are already normalised to +-180, and letting
        # this autoscale makes a level hover look like violent oscillation because the
        # view zooms into millidegree noise.
        self._angle_plot.setYRange(-180, 180, padding=0)
        self._angle_plot.show()

    def _append_position_plot(self):
        if not _HAS_PYQTGRAPH or not hasattr(self, '_curve_x'):
            return
        now = time.monotonic()
        if self._plot_t0_pos is None:
            self._plot_t0_pos = now
        t = now - self._plot_t0_pos

        # All three angles arrive from common.py's quat_to_euler already in DEGREES;
        # roll and pitch are already +-180, but yaw is wrapped there to [0, 360), so
        # only yaw needs re-normalising to the signed range the plot uses.
        roll = self.imu_msg.roll
        pitch = self.imu_msg.pitch
        yaw = self.imu_msg.yaw
        if yaw > 180:
            yaw -= 360

        # Commanded X/Y/Z: held constant at the last value actually sent
        # via send_coordinates() (see self._last_*_cmd), not the live text field.
        self._plot_t_pos.append(t)
        self._plot_x.append(self.local_pos_msg.x)
        self._plot_y.append(self.local_pos_msg.y)
        self._plot_z.append(self.local_pos_msg.z)
        self._plot_roll.append(roll)
        self._plot_pitch.append(pitch)
        self._plot_yaw.append(yaw)
        self._plot_yaw_cmd.append(self._last_yaw_cmd)
        self._plot_x_cmd.append(self._last_x_cmd)
        self._plot_y_cmd.append(self._last_y_cmd)
        self._plot_z_cmd.append(self._last_z_cmd)
        self._plot_err_x.append(self.position_error[0])
        self._plot_err_y.append(self.position_error[1])
        self._plot_err_z.append(self.position_error[2])

        # Trim samples outside the history window
        cutoff = t - POSITION_PLOT_HISTORY_S
        while self._plot_t_pos and self._plot_t_pos[0] < cutoff:
            self._plot_t_pos.popleft()
            self._plot_x.popleft()
            self._plot_y.popleft()
            self._plot_z.popleft()
            self._plot_roll.popleft()
            self._plot_pitch.popleft()
            self._plot_yaw.popleft()
            self._plot_yaw_cmd.popleft()
            self._plot_x_cmd.popleft()
            self._plot_y_cmd.popleft()
            self._plot_z_cmd.popleft()
            self._plot_err_x.popleft()
            self._plot_err_y.popleft()
            self._plot_err_z.popleft()

        # Everything above is deque bookkeeping and costs nothing; everything below is the
        # curve update, which used to make the ground station unusable in DIRECT flight.
        # It was root-caused on 2026-08-04 to the per-tick setXRange this method used to
        # end with -- 49% of all GUI-thread work. With the fixed relative-time axis that
        # replaced it, frames over 50 ms went from 17-22% to 0.0-1.7%.
        #
        # Do not try to optimise the drawing itself: antialiasing, batched updates, fixed
        # Y range, setDownsampling/setClipToView, point count (300 -> 60) and redraw
        # decimation (30 -> 3 Hz) were each measured and none changed the dropped-frame
        # rate. Message load is not the cause either -- with zero ROS traffic and these
        # curves live it was just as bad. See docs/gui_responsiveness.md.
        #
        # Skipping the update entirely while the plots are off-screen predates the axis
        # fix and is kept as defence in depth. The deques above keep filling, so the
        # history is complete the instant the operator switches back.
        pos_visible = self._pos_plot.isVisible()
        angle_visible = self._angle_plot.isVisible()
        if not (pos_visible or angle_visible):
            return

        # Decimated redraw; see PLOT_REDRAW_EVERY for why this is a CPU saving and not
        # the lag fix. The appends above still run every tick, so no data is lost.
        self._plot_tick += 1
        if self._plot_tick % self.PLOT_REDRAW_EVERY:
            return

        # Seconds ago, so the newest sample sits at x=0 and the view never has to move.
        # The axis range is fixed once at setup; scrolling the DATA instead of the VIEW
        # is what removes setXRange from this path.
        t_list = [ti - t for ti in self._plot_t_pos]
        if pos_visible:
            # Position plot: X/Y/Z and their commands, nothing else.
            self._curve_x.setData(t_list, list(self._plot_x))
            self._curve_y.setData(t_list, list(self._plot_y))
            self._curve_z.setData(t_list, list(self._plot_z))
            self._curve_x_cmd.setData(t_list, list(self._plot_x_cmd))
            self._curve_y_cmd.setData(t_list, list(self._plot_y_cmd))
            self._curve_z_cmd.setData(t_list, list(self._plot_z_cmd))
        if angle_visible:
            # Body-angle plot: roll/pitch/yaw in degrees, with the yaw command dashed.
            self._curve_roll.setData(t_list, list(self._plot_roll))
            self._curve_pitch.setData(t_list, list(self._plot_pitch))
            self._curve_yaw.setData(t_list, list(self._plot_yaw))
            self._curve_yaw_cmd.setData(t_list, list(self._plot_yaw_cmd))

    # --- Position-command step response -----------------------------------------
    def _setup_step_response_controls(self):
        self.ui.scrollbar_pref.setMinimum(1)
        self.ui.scrollbar_pref.setMaximum(STEP_RESPONSE_MAX_WINDOW_S)
        self.ui.scrollbar_pref.setValue(STEP_RESPONSE_DEFAULT_WINDOW_S)
        self.ui.scrollbar_pref.setSingleStep(1)
        self.ui.scrollbar_pref.setPageStep(5)
        self.ui.buttom_pref.setText("Reset")
        self._update_pref_window(STEP_RESPONSE_DEFAULT_WINDOW_S)

    def _setup_step_response_plot(self):
        if not _HAS_PYQTGRAPH:
            print("[PLOT] pyqtgraph not found - step response plot disabled")
            return
        container = self.ui.x_y_z_pref
        self._pref_plot = pg.PlotWidget(parent=container)
        self._pref_plot.setGeometry(container.rect())
        self._pref_plot.setBackground('w')
        self._pref_plot.setLabel('bottom', 'Time after command', units='s')
        self._pref_plot.setLabel('left', 'Inertial position', units='m')
        self._pref_plot.addLegend(offset=(5, -5))
        self._pref_plot.getPlotItem().layout.setContentsMargins(0, 15, 0, 0)
        dashed = Qt.DashLine
        self._pref_curve_x = self._pref_plot.plot(
            pen=pg.mkPen('#CC0000', width=2), name='X (m)')
        self._pref_curve_y = self._pref_plot.plot(
            pen=pg.mkPen('#008800', width=2), name='Y (m)')
        self._pref_curve_z = self._pref_plot.plot(
            pen=pg.mkPen('#0055AA', width=2), name='Z (m)')
        self._pref_curve_x_cmd = self._pref_plot.plot(
            pen=pg.mkPen('#CC0000', width=2, style=dashed), name='X cmd (m)')
        self._pref_curve_y_cmd = self._pref_plot.plot(
            pen=pg.mkPen('#008800', width=2, style=dashed), name='Y cmd (m)')
        self._pref_curve_z_cmd = self._pref_plot.plot(
            pen=pg.mkPen('#0055AA', width=2, style=dashed), name='Z cmd (m)')
        self._pref_plot.setXRange(0, self._pref_window_s, padding=0)
        self._pref_plot.show()

    def _update_pref_window(self, seconds):
        self._pref_window_s = int(seconds)
        self.ui.label_pref.setText(f"Window Size: {self._pref_window_s}s")
        if hasattr(self, '_pref_plot'):
            self._pref_plot.setXRange(0, self._pref_window_s, padding=0)

    def _clear_step_response(self):
        for samples in (
                self._pref_t, self._pref_x, self._pref_y, self._pref_z,
                self._pref_x_cmd, self._pref_y_cmd, self._pref_z_cmd):
            samples.clear()
        if hasattr(self, '_pref_curve_x'):
            for curve in (
                    self._pref_curve_x, self._pref_curve_y, self._pref_curve_z,
                    self._pref_curve_x_cmd, self._pref_curve_y_cmd,
                    self._pref_curve_z_cmd):
                curve.setData([], [])

    def _reset_step_response(self):
        self._pref_waiting = True
        self._pref_recording = False
        self._pref_t0 = None
        self._clear_step_response()
        self.ui.buttom_pref.setText("Reset")

    def _toggle_enable_log(self):
        self._pref_armed = not self._pref_armed
        if self._pref_armed:
            self.ui.buttom_enable_log.setText("Waiting...")
            self.log_message("Step response logging armed: waiting for next position command")
        else:
            self.ui.buttom_enable_log.setText("Enable")
            self.log_message("Step response logging disarmed")

    def _start_step_response(self, x, y, z):
        self._clear_step_response()
        self._pref_command = (x, y, z)
        self._pref_t0 = time.monotonic()
        self._pref_waiting = False
        self._pref_recording = True
        self.ui.buttom_pref.setText("Reset")

    def _append_step_response(self):
        # A step that finished while this tab was hidden still owes the operator one
        # redraw the first time they look at it, or they would see a partial trace.
        # Only when idle -- doing this while recording would defeat the decimation below.
        if (not self._pref_recording and getattr(self, '_pref_dirty', False)
                and hasattr(self, '_pref_curve_x') and self._pref_plot.isVisible()):
            self._redraw_step_response()
        if not self._pref_recording or self._pref_t0 is None:
            return
        elapsed = time.monotonic() - self._pref_t0
        if elapsed > self._pref_window_s:
            self._pref_recording = False
            self._pref_waiting = True
            return

        x_cmd, y_cmd, z_cmd = self._pref_command
        self._pref_t.append(elapsed)
        self._pref_x.append(self.local_pos_msg.x)
        self._pref_y.append(self.local_pos_msg.y)
        self._pref_z.append(self.local_pos_msg.z)
        self._pref_x_cmd.append(x_cmd)
        self._pref_y_cmd.append(y_cmd)
        self._pref_z_cmd.append(z_cmd)

        if not hasattr(self, '_pref_curve_x'):
            return
        # Same reasoning as _append_position_plot: the appends above are cheap and must
        # keep running so the recorded step is complete, but the redraw is not, and while
        # a step is recording this is a third plot's worth of work on top of the other
        # two. Skip it when off-screen or on a decimated tick; the flush at the top of
        # this method makes the trace whole again once the tab is shown.
        self._pref_dirty = True
        self._pref_tick = getattr(self, '_pref_tick', 0) + 1
        if not self._pref_plot.isVisible() or self._pref_tick % self.PLOT_REDRAW_EVERY:
            return
        self._redraw_step_response()

    def _redraw_step_response(self):
        self._pref_dirty = False
        t_values = list(self._pref_t)
        self._pref_curve_x.setData(t_values, list(self._pref_x))
        self._pref_curve_y.setData(t_values, list(self._pref_y))
        self._pref_curve_z.setData(t_values, list(self._pref_z))
        self._pref_curve_x_cmd.setData(t_values, list(self._pref_x_cmd))
        self._pref_curve_y_cmd.setData(t_values, list(self._pref_y_cmd))
        self._pref_curve_z_cmd.setData(t_values, list(self._pref_z_cmd))

    # define the signal-slot combination of ros and pyqt GUI
    def set_ros_callbacks(self):
        # feedbacks from ros
        self.ros_object.connect_update_gui(self.update_gui_data)
        self.ros_object.direct_mode_result.connect(self._handle_direct_mode_result)
        self.ros_object.controllers_listed.connect(self._handle_controllers_listed)
        self.ros_object.controller_activated.connect(self._handle_controller_activated)

        # callbacks from GUI
        self.ui.SendPositionUAV.clicked.connect(self.send_coordinates)
        self.ui.GetCurrentPositionUAV.clicked.connect(self.get_coordinates)
        self.ui.buttom_pref.clicked.connect(self._reset_step_response)
        self.ui.scrollbar_pref.valueChanged.connect(self._update_pref_window)
        self.ui.buttom_enable_log.clicked.connect(self._toggle_enable_log)
        self.ui.buttom_refresh_options.clicked.connect(self._refresh_controller_switch)
        self.ui.buttom_activate_controller.clicked.connect(self._request_controller_switch)
        # Was previously never connected, so the dedicated abort path did nothing.
        self.ui.buttom_back_to_baseline.clicked.connect(self._request_back_to_baseline)

    # update GUI data
    def update_gui_data(self):
        if not self.lock.tryLock():
            print("SingleDroneRosThread: lock failed")
            return
        # store to local variables for fast lock release
        self.imu_msg = self.ros_object.data_struct.current_imu
        self.local_pos_msg = self.ros_object.data_struct.current_local_pos
        vel_msg = self.ros_object.data_struct.current_vel
        state_msg = self.ros_object.data_struct.current_state
        alttitude_targ_msg = self.ros_object.data_struct.current_attitude_target
        vehicle_name = self.ros_object.data_struct.current_vehicle_name
        yaw_align = self.ros_object.data_struct.current_yaw_align
        controller_type = self.ros_object.data_struct.current_controller_type
        motor_commands = tuple(self.ros_object.data_struct.current_motor_commands)
        position_error = self.ros_object.data_struct.current_position_error
        self.position_error = (position_error.x, position_error.y, position_error.z)
        self.lock.unlock()

        self.ui.label_vehicle_type.setText(f"Vehicle Type: {vehicle_name}")
        controller_type_display = self._controller_display_name(controller_type)
        self.ui.label_controller_type.setText(f"Controller: {controller_type_display}")
        # Substring, not equality: every direct-actuation variant reports a
        # name ENDING in "Direct Actuation" but prefixes it with its law
        # ("Geometric Direct Actuation"). An equality test silently zeroed the
        # rotor display and mislabelled the control mode on the geometric
        # nodes, which publish real motor commands just like the classic one.
        direct_actuation = "Direct Actuation" in (controller_type or "")
        self._update_controller_switch(controller_type)

        # accelerometer data
        self.ui.X_DISP.display("{:.2f}".format(self.imu_msg.roll, 2))
        self.ui.Y_DISP.display("{:.2f}".format(self.imu_msg.pitch, 2))
        self.ui.Z_DISP.display("{:.2f}".format(self.imu_msg.yaw, 2))

        self.ui.TargROLL_DISP.display("{:.2f}".format(alttitude_targ_msg.roll, 2))
        self.ui.TargPITCH_DISP.display("{:.2f}".format(alttitude_targ_msg.pitch, 2))
        self.ui.TargYAW_DISP.display("{:.2f}".format(alttitude_targ_msg.yaw, 2))

        throttle_pct = max(0, min(100, int(alttitude_targ_msg.thrust * 100)))
        self.ui.bar_normalized_throttle.setValue(throttle_pct)
        if direct_actuation:
            self.ui.label_total_nt.setText("Using direct\nactuation...")
        else:
            self.ui.label_total_nt.setText(f"Total N/T: {throttle_pct}%")

        # GATE ON THE MOTOR STREAM ITSELF, NOT ON `connected`. `connected` is PX4's
        # pre_flight_checks_pass (see common.py), which is false for an ENTIRE armed
        # DIRECT flight on the AM-T650 rigs -- EKF aligned, commander "Ready for
        # takeoff!", vehicle flying -- and pinned these pies at 0.0% with nothing
        # wrong anywhere. Measured 2026-08-24/25. Freshness of motors_debug is the
        # question actually being asked here: is a direct-actuation node streaming
        # commands right now? It also zeroes correctly on reverting to SAFETY, when
        # the stream stops, which is the behaviour the old gate was reaching for.
        motors_live = self.ros_object.data_struct.motor_commands_fresh()
        displayed_motor_commands = (
            motor_commands
            if direct_actuation and motors_live
            else (0.0, 0.0, 0.0, 0.0)
        )
        self._motor_display.set_commands(displayed_motor_commands)

        # local position data
        self.ui.RelX_DISP.display("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.RelY_DISP.display("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.AGL_DISP.display("{:.2f}".format(self.local_pos_msg.z, 2))

        self._append_position_plot()
        self._append_step_response()
        self._update_camera_view()
        self._update_optitrack_status()
        self._update_system_status()

        self.ui.TargROLL_RATE_DISP.display("{:.2f}".format(alttitude_targ_msg.roll_rate, 2))
        self.ui.TargPITCH_RATE_DISP.display("{:.2f}".format(alttitude_targ_msg.pitch_rate, 2))
        self.ui.TargYAW_RATE_DISP.display("{:.2f}".format(alttitude_targ_msg.yaw_rate, 2))

        # velocity data
        self.ui.U_Vel_DISP.display("{:.2f}".format(vel_msg.x, 2))
        self.ui.V_Vel_DISP.display("{:.2f}".format(vel_msg.y, 2))
        self.ui.W_Vel_DISP.display("{:.2f}".format(vel_msg.z, 2))

        # state updates
        if state_msg:
            self.ui.StateARM.setText("Armed" if state_msg.armed else "Disarmed")
            self.ui.StateARM.setStyleSheet("color: red" if state_msg.armed else "color: green")
            self.ui.StateConnected.setText("Connected" if state_msg.connected else "Disconnected")
            self.ui.StateConnected.setStyleSheet("color: green" if state_msg.connected else "color: red")
            self.ui.StateMode.setText(state_msg.mode)
        else:
            self.ui.StateARM.setText("Unknown")
            self.ui.StateConnected.setText("Disconnected")
            self.ui.StateMode.setText("Unknown")

        self.ui.StateYawAlign.setText("Yaw Align: {}".format(yaw_align))
        self.ui.StateYawAlign.setStyleSheet("color: green" if yaw_align else "color: red")
        self._yaw_align = bool(yaw_align)

        # flight log: record arming, mode (e.g. OFFBOARD), and yaw-alignment
        # transitions as they're observed from telemetry (send_coordinates()
        # logs commanded positions separately)
        if state_msg:
            if self._prev_armed is not None and state_msg.armed != self._prev_armed:
                self.log_message("Vehicle armed" if state_msg.armed else "Vehicle disarmed")
            self._prev_armed = state_msg.armed

            if self._prev_mode is not None and state_msg.mode != self._prev_mode:
                self.log_message(f"Mode changed: {self._prev_mode} -> {state_msg.mode}")
            self._prev_mode = state_msg.mode

        if self._prev_yaw_align is not None and yaw_align != self._prev_yaw_align:
            self.log_message("Yaw alignment complete" if yaw_align else "Yaw alignment lost")
        self._prev_yaw_align = yaw_align

        if (self._prev_controller_type is not None
                and controller_type != self._prev_controller_type):
            self.log_message(
                f"Controller changed: {self._prev_controller_type} -> {controller_type}"
            )
        self._prev_controller_type = controller_type

        # Direct actuation still publishes an attitude debug setpoint, so the
        # debug-message mode alone would incorrectly display "Attitude".
        if direct_actuation:
            self.ui.ControlMode.setText("Direct Actuation")
        elif alttitude_targ_msg.mode == 0:
            self.ui.ControlMode.setText("Not Started")
        elif alttitude_targ_msg.mode == 1:
            self.ui.ControlMode.setText("Attitude")
        elif alttitude_targ_msg.mode == 2:
            self.ui.ControlMode.setText("Bodyrate")

    ### callback functions for modifying GUI elements ###
    def send_coordinates(self):
        if not self._yaw_align:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Yaw is not aligned. Cannot move the drone.")
            msg.setWindowTitle("Yaw Not Aligned")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            self.log_message("Position command rejected: yaw is not aligned")
            return

        # if text is inalid, warn user
        try :
            x = float(self.ui.XPositionUAV.text())
            y = float(self.ui.YPositionUAV.text())
            z = float(self.ui.ZPositionUAV.text())
            yaw = float(self.ui.YAWUAV.text())
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Invalid input, make sure values are numbers")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return
        
        # if values are outside the geofence, warn user
        if abs(x) > float(self.ros_object.config[0]) or abs(y) > float(self.ros_object.config[1]) or abs(z) > float(self.ros_object.config[2]) or z <= 0:
            ## pop up dialog
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Position is outside the geofence bounds")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        # Queued, not published inline: this runs on the Qt thread and must not touch
        # rclpy. It is issued by _drain_requests() on the next ROS tick (<=33 ms).
        self.ros_object.queue_coordinates(x, y, z, yaw)
        self.log_message(f"Position command sent: {x}, {y}, {z}, {yaw}")

        # Update the dashed command lines in the X/Y/Z plot to the setpoint sent.
        self._last_x_cmd = x
        self._last_y_cmd = y
        self._last_z_cmd = z
        self._last_yaw_cmd = ((yaw + 180.0) % 360.0) - 180.0

        if self._pref_armed:
            self._pref_armed = False
            self.ui.buttom_enable_log.setText("Enable")
            self._start_step_response(x, y, z)
            self.log_message("Step response logging started")

    def get_coordinates(self):
        # get current relative position
        self.ui.XPositionUAV.setText("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.YPositionUAV.setText("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.ZPositionUAV.setText("{:.2f}".format(self.local_pos_msg.z, 2))
        self.ui.YAWUAV.setText("{:.2f}".format(self.imu_msg.yaw, 2))
