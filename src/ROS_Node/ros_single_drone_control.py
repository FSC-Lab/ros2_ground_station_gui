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

import base64
import re
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
from PyQt5.QtCore import (QEvent, QObject, pyqtSignal, QProcess, QThread, QDateTime, QTimer, Qt,
                          QPointF, QRectF, QUrl)
from PyQt5.QtGui import (QBrush, QColor, QFont, QFontMetrics, QImage, QPainter, QPen,
                         QTextCharFormat, QTextCursor, QTextDocument, QTextDocumentFragment,
                         QTextImageFormat)
from PyQt5.QtWidgets import QAbstractItemView, QMessageBox, QTableWidgetItem, QWidget
import Common
from Common.llm_client import LlmClient
from geometry_msgs.msg import Point, PoseStamped

try:
    import os
    os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')
    import pyqtgraph as pg
    _HAS_PYQTGRAPH = True
except ImportError:
    _HAS_PYQTGRAPH = False

POSITION_PLOT_HISTORY_S = 10.0  # seconds of history shown in the live plots
# CCM Flight Log: long enough for two laps of the default 8 s circle / figure-8.
CCM_PLOT_HISTORY_S = 20.0
# A CCM stream older than this is treated as absent (the plots break the line).
CCM_STREAM_FRESH_S = 0.5
# Latched planner info is re-published at 2 Hz; older than this the planner is gone.
CCM_PLANNER_INFO_FRESH_S = 3.0
CCM_PLANNER_TIMEOUT_S = 3.0
CCM_PLANNER_NS = '/uav_0/quadrotor_planner'
CCM_NODE_NS = '/uav_0/fsc_autopilot_ros2/ccm_direct_actuation'
# The rotor-thrust CCM node (single_drone_ccm_rotor_actuation): same interface, own namespace
# and state message. Only one CCM node ever runs, so both feed the same CommonData fields.
CCM_ROTOR_NODE_NS = '/uav_0/fsc_autopilot_ros2/ccm_rotor_actuation'
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
# LLM status assistant (VLM tab). The proxy on the LLM host, reached over NetBird; it has
# no authentication, so only NetBird peers the access policy allows can reach it.
LLM_BASE_URL = 'http://100.103.152.77:8080'
LLM_MODEL = 'qwen2.5vl:7b'
LLM_HEALTH_INTERVAL_MS = 15000  # keeps Api_status honest while connected
LLM_HISTORY_TURNS = 6           # question/answer pairs resent as context
LLM_CHATLOG_MAX_BLOCKS = 2000   # bounded, like the flight log
LLM_FLUSH_MS = 100              # streamed text reaches the widget at most 10x a second
# "Is there a <object> in view?" -> VLM camera check (look pipeline). The Orin's detector
# process answers snapshot requests with the latest frame + that frame's detections; the
# namespace follows its --ros-ns, like the VISION_* topics above.
VISION_SNAPSHOT_REQUEST_TOPIC = '/uav_0/snapshot/request'
VISION_SNAPSHOT_RESPONSE_TOPIC = '/uav_0/snapshot/response'
# Room-frame positions use /uav_0/mocap, the pose the camera-to-drone extrinsic was
# calibrated against (the estimator's odom lags it by up to ~10 cm / 5 deg in flight).
MOCAP_POSE_TOPIC = '/uav_0/mocap'
LOOK_SNAPSHOT_QUALITY = 95
LOOK_SNAPSHOT_TIMEOUT_MS = 5000
LOOK_POSE_MAX_AGE_S = 0.5       # older than this -> no room position, only drone-relative
LOOK_STILL_M = 0.05             # pose change across the request -> "drone was moving"
LOOK_STILL_DEG = 3.0
LOOK_MAX_MOVES = 5              # relocation attempts per search, each operator-confirmed
LOOK_STEP_M = 0.3
LOOK_STEP_UP_M = 0.2
LOOK_YAW_STEP_DEG = 30.0
LOOK_SETTLE_POS_M = 0.10        # "arrived": within this of the target...
LOOK_SETTLE_YAW_DEG = 6.0
LOOK_SETTLE_HOLD_S = 1.0        # ...for this long
LOOK_SETTLE_TIMEOUT_S = 10.0    # then take the picture anyway, and say so
LOOK_THUMBNAILS_KEPT = 12       # snapshot thumbnails kept in LLM_chatlog (memory bound)
# The only moves the VLM may propose. Horizontal steps are in the drone's heading frame
# (ENU yaw: 0 = +x, anticlockwise positive -- matches /uav_0/mocap within 0.7 deg).
LOOK_MOVES = {
    'yaw_left': (f"turn left {LOOK_YAW_STEP_DEG:.0f}°", 0.0, 0.0, 0.0, +LOOK_YAW_STEP_DEG),
    'yaw_right': (f"turn right {LOOK_YAW_STEP_DEG:.0f}°", 0.0, 0.0, 0.0, -LOOK_YAW_STEP_DEG),
    'forward': (f"move forward {LOOK_STEP_M:.1f} m", LOOK_STEP_M, 0.0, 0.0, 0.0),
    'back': (f"move back {LOOK_STEP_M:.1f} m", -LOOK_STEP_M, 0.0, 0.0, 0.0),
    'left': (f"move left {LOOK_STEP_M:.1f} m", 0.0, LOOK_STEP_M, 0.0, 0.0),
    'right': (f"move right {LOOK_STEP_M:.1f} m", 0.0, -LOOK_STEP_M, 0.0, 0.0),
    'up': (f"climb {LOOK_STEP_UP_M:.1f} m", 0.0, 0.0, LOOK_STEP_UP_M, 0.0),
    'down': (f"descend {LOOK_STEP_UP_M:.1f} m", 0.0, 0.0, -LOOK_STEP_UP_M, 0.0),
}
# The VLM only judges the image. Tested 2026-09-27: asked for visibility, the index of
# the matching detector box and a move in one JSON reply, qwen2.5vl:7b answered from the
# detector list and called a plainly visible bottle "not visible". Describe-first plus
# a two-field JSON fixed it; matching the detector result is done in code (by label).
LOOK_SYSTEM_PROMPT = (
    "You look at one image from a drone's camera, which points forward and about 45 "
    "degrees down, to check for an object the operator asked about.\n"
    "Reply in exactly two lines:\n"
    "Line 1: one short sentence naming the main things you can see in the image.\n"
    'Line 2: only this JSON: {"visible": true or false, "move": "yaw_left", "yaw_right", '
    '"forward", "back", "left", "right", "up", "down" or null}\n'
    "visible: true if the object is anywhere in the image, even partly.\n"
    "move: the one move that would give a closer or clearer view of the object: yaw_left "
    "or yaw_right to turn toward something at that edge, forward if it is small or far, "
    "up to see further, back if it is cut off at the bottom. null if it is already large "
    "and clear, or if no move would help."
)
LLM_SYSTEM_PROMPT = (
    "You are the status assistant in the ground-station GUI of an indoor quadrotor "
    "(uav_0) flying under OptiTrack motion capture.\n"
    "Every question arrives with a telemetry snapshot (JSON) that the ground station took "
    "when the question was asked. Answer only from that snapshot and this conversation.\n"
    "- If a value is missing, null or 'no data received', say it is not available. Never "
    "guess or invent a number.\n"
    "- age_s is seconds since that source was last received. If it is more than a few "
    "seconds, say the value may be out of date.\n"
    "- Units: metres, m/s, degrees, percent, dBm, milliseconds. Position and velocity are "
    "in the local motion-capture frame: x and y horizontal, z up (height). Yaw is -180 to "
    "180 degrees.\n"
    "- optitrack: whether raw motion-capture frames reach the ground station. wifi: the "
    "drone computer's own view of its WiFi link (good, fair or bad, with the reason); bad "
    "also means its reports have stopped arriving. onboard_feed_status: the drone "
    "computer's measurement of its own mocap and odometry input.\n"
    "- px4_status.preflight_checks_pass is PX4's pre-flight check flag. It can stay false "
    "for a whole normal flight on these vehicles; do not call that a fault on its own.\n"
    "- You cannot control the drone: you cannot send commands, arm, move it or change "
    "modes. If asked to, say that at this stage you can only report status.\n"
    "- Exception, the camera: if the operator asks whether something is in the camera's "
    "view, or asks you to look for, find or check for an object, do not answer from the "
    "snapshot. Reply with only this JSON and nothing else: "
    '{"action": "look", "object": "<the object, in a few words>"}. '
    "Do this every time, even if the same object was checked before: the drone or the "
    "object may have moved. The ground station then checks the camera and reports back. "
    "Results of earlier "
    "checks are in recent_camera_checks, and objects found (with positions) in "
    "found_objects.\n"
    "Reply in plain text without markdown, in one to four short sentences unless the "
    "operator asks for detail."
)
# from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandLong, SetMode
from px4_msgs.msg import ActuatorMotors, VehicleStatus,VehicleAttitudeSetpoint,VehicleAttitude, VehicleGlobalPosition, BatteryStatus,VehicleRatesSetpoint, EstimatorStatusFlags
from fsc_autopilot_ros2_msgs.msg import CcmReference, CcmRotorState, CcmState, Mocap, PositionControllerReference, PositionControllerState, VehicleInfo
from fsc_autopilot_ros2_msgs.srv import ActivateController, ListControllers

# from mavros_msgs.msg import State, AttitudeTarget
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from rclpy.serialization import deserialize_message
import json
# from fsc_autopilot_msgs.msg import TrackingReference
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger


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


def quat_to_matrix(q):
    """3x3 rotation from a unit quaternion (x, y, z, w), same form as the Orin's
    extrinsics._quat_to_matrix used to calibrate the camera."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def body_to_world(p_body, position, orientation):
    """Drone-body (FLU) point -> room (/uav_0/mocap ENU) point: R(q) p + t, the
    composition scripts/solve_extrinsic.py calibrated against."""
    return quat_to_matrix(orientation) @ np.asarray(p_body, dtype=float) + np.asarray(position, dtype=float)


# Explicit camera requests, recognised in code before the chat model is asked. Tested
# 2026-09-27: with an earlier result in its snapshot, qwen2.5 answered a repeat "look for
# a person" from memory 4 times out of 4 instead of routing it to the camera. Past-tense
# questions ("did you find ...?") do not match and are answered by the model from
# recent_camera_checks.
_LOOK_PATTERNS = [re.compile(pattern, re.I) for pattern in (
    r"^(?:please\s+)?(?:look|search|scan)\s+(?:around\s+)?for\s+(?P<obj>.+)$",
    r"^(?:please\s+)?find\s+(?P<obj>.+)$",
    r"^(?:can|could|do)\s+you\s+see\s+(?P<obj>.+?)(?:\s+(?:now|in\s+(?:the\s+)?"
    r"(?:view|frame|camera|image|picture)))?$",
    r"^(?:is|are)\s+there\s+(?P<obj>.+?)\s+(?:in\s+(?:the\s+)?"
    r"(?:view|frame|camera|image|picture|sight)|visible)(?:\s+now)?$",
    r"^(?:does|can)\s+the\s+camera\s+see\s+(?P<obj>.+)$",
    r"^check\s+(?:if|whether)\s+(?:the\s+camera\s+(?:can\s+)?sees?|you\s+can\s+see)\s+(?P<obj>.+)$",
)]
_LOOK_LEADING_WORDS = re.compile(r"^(?:a|an|the|any|my|some)\s+", re.I)
# "can you see the WiFi status?" is a telemetry question, not a camera request.
_LOOK_NOT_OBJECTS = re.compile(
    r"\b(?:wifi|battery|cpu|position|status|altitude|height|speed|velocity|mode|signal|"
    r"ping|telemetry|log|optitrack|mocap|controller|yaw|attitude)\b", re.I)


def parse_look_request(text):
    """The object in an explicit camera request ("is there a bottle in view?"), or None."""
    sentence = text.strip().rstrip('?.! ').strip()
    for pattern in _LOOK_PATTERNS:
        match = pattern.match(sentence)
        if match:
            obj = _LOOK_LEADING_WORDS.sub('', match.group('obj').strip()).strip()
            if obj and len(obj) <= 40 and not _LOOK_NOT_OBJECTS.search(obj):
                return obj
    return None


# Irregular names for COCO labels ("bottles" already contains "bottle").
_LABEL_SYNONYMS = {'people': 'person', 'persons': 'person', 'man': 'person', 'men': 'person',
                   'woman': 'person', 'women': 'person', 'human': 'person', 'humans': 'person'}


def object_matches_label(obj, label):
    """Does the operator's object name refer to this detector label? ('blue bottle' ->
    'bottle', 'people' -> 'person')."""
    name, label = obj.lower(), label.lower()
    names = [name] + [_LABEL_SYNONYMS[w] for w in re.findall(r"[a-z]+", name) if w in _LABEL_SYNONYMS]
    return any(label in n or n in label for n in names)


def parse_json_object(text):
    """The first {...} object in a model reply, or None. Models sometimes wrap JSON in
    prose or code fences despite being told not to."""
    start, end = text.find('{'), text.rfind('}')
    if start < 0 or end <= start:
        return None
    try:
        obj = json.loads(text[start:end + 1])
    except ValueError:
        return None
    return obj if isinstance(obj, dict) else None


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
        "OptiTrack regained" during the siren and then the stale "No OptiTrack" after it.
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
    # One parsed snapshot/response from the Orin, plus the /uav_0/mocap pose sampled on
    # this (ROS) thread when the request went out and when the response came in.
    snapshot_received = pyqtSignal(dict)
    # (action, success, message) of a quadrotor_ccm_planner Trigger call.
    ccm_planner_result = pyqtSignal(str, bool, str)

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
        # (ccm_direct_actuation: the torque-mode CCM node, its own fork and
        # namespace; its controller_type "CCM Direct Actuation" passes the
        # substring gate.)
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
                f'{CCM_NODE_NS}/motors_debug',
                f'{CCM_ROTOR_NODE_NS}/motors_debug',
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
        # Camera snapshots for the VLM look pipeline. RELIABLE both ways: requests are
        # rare and must arrive; a response (~85 kB) is being actively waited for.
        self.snapshot_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        self.snapshot_request_pub = self.create_publisher(
            String, VISION_SNAPSHOT_REQUEST_TOPIC, self.snapshot_qos_profile)
        self.snapshot_response_sub = self.create_subscription(
            String, VISION_SNAPSHOT_RESPONSE_TOPIC, self.snapshot_response_callback,
            self.snapshot_qos_profile)
        # /uav_0/mocap arrives at ~120 Hz but is only needed when a snapshot is taken, so
        # keep the latest serialized message (raw=True) and decode it on demand.
        self._mocap_raw = None  # (bytes, monotonic receive time)
        self.mocap_sub = self.create_subscription(
            Mocap, MOCAP_POSE_TOPIC, self.mocap_callback, self.optitrack_qos_profile, raw=True)
        self._snapshot_request_pose = {}  # request id -> pose when it was sent

        # Torque-mode CCM pipeline: the pair the CCM actually tracks -- x* from the
        # planner and x from the flight node (the message the model loader evaluates
        # the controller on) -- plus the flight node's mode and the planner's latched
        # status / info / trajectory list for the CCM Trajectory tab.
        self.ccm_reference_sub = self.create_subscription(
            CcmReference, f'{CCM_PLANNER_NS}/ccm_reference', self.ccm_reference_callback, 10)
        # 250 Hz, depth 1 and best effort: only the newest state is ever displayed.
        self.ccm_state_sub = self.create_subscription(
            CcmState, f'{CCM_NODE_NS}/state', self.ccm_state_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.ccm_mode_sub = self.create_subscription(
            String, f'{CCM_NODE_NS}/mode', self.ccm_mode_callback, self.controller_type_qos_profile)
        # Rotor-thrust node: CcmRotorState has the same position / velocity fields.
        self.ccm_rotor_state_sub = self.create_subscription(
            CcmRotorState, f'{CCM_ROTOR_NODE_NS}/state', self.ccm_state_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.ccm_rotor_mode_sub = self.create_subscription(
            String, f'{CCM_ROTOR_NODE_NS}/mode', self.ccm_mode_callback, self.controller_type_qos_profile)
        self.ccm_planner_status_sub = self.create_subscription(
            String, f'{CCM_PLANNER_NS}/status', self.ccm_planner_status_callback,
            self.controller_type_qos_profile)
        self.ccm_planner_info_sub = self.create_subscription(
            String, f'{CCM_PLANNER_NS}/info', self.ccm_planner_info_callback,
            self.controller_type_qos_profile)
        self.ccm_trajectories_sub = self.create_subscription(
            String, f'{CCM_PLANNER_NS}/available_trajectories', self.ccm_trajectories_callback,
            self.controller_type_qos_profile)

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
        # quadrotor_ccm_planner (fsc_trajectory_planner). The planner never arms, never
        # changes the PX4 mode and never switches the controller; these only move the
        # reference it streams.
        self.ccm_select_pub = self.create_publisher(String, f'{CCM_PLANNER_NS}/select', 10)
        self.ccm_time_scale_pub = self.create_publisher(Float64, f'{CCM_PLANNER_NS}/time_scale', 10)
        self.ccm_planner_clients = {
            action: self.create_client(Trigger, f'{CCM_PLANNER_NS}/{action}')
            for action in ('hold', 'go_to_start', 'start', 'back_to_hover', 'release')
        }
        # action -> (future, deadline) of the call in flight; enforced in
        # _check_ccm_planner_deadlines() for the same reason as the controller calls.
        self._ccm_calls = {}
        self._ccm_planner_ready_cache = False

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

    def ccm_reference_callback(self, msg):
        self.data_struct.update_ccm_reference((
            msg.position.x, msg.position.y, msg.position.z,
            msg.velocity.x, msg.velocity.y, msg.velocity.z,
            msg.phase, msg.trajectory, msg.t))

    def ccm_state_callback(self, msg):
        self.data_struct.update_ccm_state((
            msg.position.x, msg.position.y, msg.position.z,
            msg.velocity.x, msg.velocity.y, msg.velocity.z))

    def ccm_mode_callback(self, msg):
        self.data_struct.update_ccm_mode(msg.data)

    def ccm_planner_status_callback(self, msg):
        self.data_struct.update_ccm_planner_status(msg.data)

    def ccm_planner_info_callback(self, msg):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(info, dict):
            self.data_struct.update_ccm_planner_info(info)

    def ccm_trajectories_callback(self, msg):
        try:
            entries = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(entries, list):
            self.data_struct.update_ccm_trajectories(
                [e for e in entries if isinstance(e, dict) and e.get('name')])

    def position_error_callback(self, msg):
        self.data_struct.update_position_error(
            msg.position_error.x,
            msg.position_error.y,
            msg.position_error.z
        )

    def optitrack_callback(self, _serialized_msg):
        self.data_struct.update_optitrack()

    def mocap_callback(self, serialized_msg):
        self._mocap_raw = (serialized_msg, time.monotonic())  # one tuple swap, no lock

    def _latest_mocap_pose(self):
        """ROS thread: the latest /uav_0/mocap pose as plain values, or None."""
        latest = self._mocap_raw
        if latest is None:
            return None
        msg = deserialize_message(latest[0], Mocap)
        p, q = msg.pose.position, msg.pose.orientation
        return {'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w],
                'age_s': time.monotonic() - latest[1]}

    def _send_snapshot_request(self, request_id):
        # Sample the pose as the request leaves: the frame the Orin answers with is its
        # latest one, a few tens of ms older than the request's arrival.
        self._snapshot_request_pose[request_id] = self._latest_mocap_pose()
        while len(self._snapshot_request_pose) > 20:  # unanswered requests
            self._snapshot_request_pose.pop(next(iter(self._snapshot_request_pose)))
        msg = String()
        msg.data = json.dumps({'id': request_id, 'quality': LOOK_SNAPSHOT_QUALITY})
        self.snapshot_request_pub.publish(msg)

    def snapshot_response_callback(self, msg):
        try:
            response = json.loads(msg.data)
            if not isinstance(response, dict):
                raise ValueError('not a JSON object')
        except ValueError as e:
            self.get_logger().warn(f'Unparseable {VISION_SNAPSHOT_RESPONSE_TOPIC} message: {e}',
                                   throttle_duration_sec=5.0)
            return
        response['_pose_at_request'] = self._snapshot_request_pose.pop(response.get('id'), None)
        response['_pose_at_response'] = self._latest_mocap_pose()
        self.snapshot_received.emit(response)

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
        self._ccm_planner_ready_cache = all(
            c.service_is_ready() for c in self.ccm_planner_clients.values())

    def ccm_planner_ready(self):
        """Cached -- safe to call from the Qt thread at GUI rate."""
        return self._ccm_planner_ready_cache

    # -- called FROM THE QT THREAD: enqueue only, never touch rclpy ---------
    def queue_controller_list(self):
        with self._request_lock:
            self._pending_requests.append(("list", None))

    def queue_controller_activation(self, name):
        with self._request_lock:
            self._pending_requests.append(("activate", name))

    def queue_snapshot_request(self, request_id):
        with self._request_lock:
            self._pending_requests.append(("snapshot", request_id))

    def queue_coordinates(self, x, y, z, yaw):
        # Same reason as the two above: publish_coordinates() touches the clock, a
        # publisher and the logger, and doing that from the Qt thread while the executor
        # spins means contending with it for the rclpy locks. The controller buttons were
        # moved onto this queue when the switch button stalled ~1 s; this one was missed.
        with self._request_lock:
            self._pending_requests.append(("coords", (x, y, z, yaw)))

    def queue_ccm_planner_call(self, action):
        with self._request_lock:
            self._pending_requests.append(("ccm_call", action))

    def queue_ccm_select(self, name):
        with self._request_lock:
            self._pending_requests.append(("ccm_select", name))

    def queue_ccm_time_scale(self, value):
        with self._request_lock:
            self._pending_requests.append(("ccm_time_scale", value))

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
            elif kind == "snapshot":
                self._send_snapshot_request(arg)
            elif kind == "ccm_call":
                self.request_ccm_planner_call(arg)
            elif kind == "ccm_select":
                self.ccm_select_pub.publish(String(data=arg))
            elif kind == "ccm_time_scale":
                self.ccm_time_scale_pub.publish(Float64(data=float(arg)))

    def request_ccm_planner_call(self, action):
        client = self.ccm_planner_clients.get(action)
        if client is None or not client.service_is_ready():
            self.ccm_planner_result.emit(
                action, False, f"{CCM_PLANNER_NS}/{action} is unavailable "
                "(is quadrotor_ccm_planner running?)")
            return
        if action in self._ccm_calls:
            self.ccm_planner_result.emit(action, False, "previous request still pending")
            return
        future = client.call_async(Trigger.Request())
        self._ccm_calls[action] = (future, time.monotonic() + CCM_PLANNER_TIMEOUT_S)
        future.add_done_callback(lambda f, a=action: self._ccm_planner_response(a, f))

    def _ccm_planner_response(self, action, future):
        entry = self._ccm_calls.get(action)
        if entry is None or entry[0] is not future:
            return  # timed out and already reported
        del self._ccm_calls[action]
        if future.cancelled():
            return
        try:
            response = future.result()
        except Exception as exc:
            self.ccm_planner_result.emit(action, False, f"call failed: {exc}")
            return
        self.ccm_planner_result.emit(action, response.success, response.message)

    def _check_ccm_planner_deadlines(self):
        now = time.monotonic()
        for action, (future, deadline) in list(self._ccm_calls.items()):
            if now > deadline:
                del self._ccm_calls[action]
                future.cancel()
                # Not a failure claim: the planner may have acted on it. Its status says.
                self.ccm_planner_result.emit(
                    action, False,
                    f"no response after {CCM_PLANNER_TIMEOUT_S:.0f}s; confirm from the planner status")

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
        self._check_ccm_planner_deadlines()
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
        self._last_cmd_time = None  # monotonic time of the last sent command, if any
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
        self._setup_llm()
        self._setup_controller_switch()
        self._setup_ccm_flight_log()
        self._setup_ccm_trajectory_tab()

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
            # The label shows the state; the announcement names the event. "Regained"
            # only after a loss -- present at launch is just "normal".
            event = "OptiTrack regained" if was_ok is False else "OptiTrack normal"
            self._announcer.announce('optitrack', event)
            self.log_message(event)
        else:
            self._set_status_label(
                self.ui.optitrack_status, "No OptiTrack", STATUS_STYLE['bad'])
            # Siren on every "No OptiTrack", the launch verdict included (operator's
            # choice, 2026-09-26): launching at the desk with no mocap sounds it too.
            self._announcer.announce('optitrack', "No OptiTrack", siren=True)
            self.log_message(
                f"No OptiTrack: nothing on {OPTITRACK_TOPIC} for {OPTITRACK_TIMEOUT_S:.1f} s")

    def _setup_system_status(self):
        self._system_status_seq = 0
        # Consecutive reports showing packets queued for WiFi (see wifi_summary).
        self._wifi_backlog_reports = 0
        # None until the first verdict, then whether the last announced state was "lost".
        self._wifi_lost = None
        self._system_status_started = time.monotonic()
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
        # a threshold. At launch, no verdict until a report has had time to arrive (DDS
        # discovery plus up to 1 s to the next report); after that, launching with the
        # Orin off is a loss like any other (operator's choice, 2026-09-26). A good first
        # verdict is not announced.
        if (received == 0.0
                and time.monotonic() - self._system_status_started < SYSTEM_STATUS_STALE_S):
            return
        lost = level == 'bad'
        if lost == self._wifi_lost:
            return
        was_lost = self._wifi_lost
        self._wifi_lost = lost
        if lost:
            self._announcer.announce('wifi', "No WiFi", siren=True)
            self.log_message(wifi_text.replace("\n", ": "))
        elif was_lost:
            self._announcer.announce('wifi', "WiFi regained")
            self.log_message("WiFi regained")


    # ------------------------------------------------------------------
    # LLM status assistant (VLM tab)
    #
    # Read-only by design: the model sees a snapshot of what this GUI has received and
    # answers in text. Nothing here publishes, calls a ROS service, or uses the proxy's
    # /api/parse -- commands from an LLM would need the geofence and pose checks that
    # live in nl_commander, not here. All network I/O is QNetworkAccessManager on this
    # (GUI) thread, which never blocks and never touches rclpy.
    # ------------------------------------------------------------------

    LLM_TEXT_COLORS = {
        'info': '#666666',
        'user': '#1F5FBF',
        'llm': '#1E7B34',
        'error': '#C62828',
    }

    def _setup_llm(self):
        self._llm = LlmClient(LLM_BASE_URL, LLM_MODEL, self)
        self._llm.health_checked.connect(self._on_llm_health)
        self._llm.chat_delta.connect(self._on_llm_delta)
        self._llm.chat_finished.connect(self._on_llm_finished)
        # disconnected | connecting | loading | online | offline
        self._llm_state = 'disconnected'
        self._llm_warmup = False      # the in-flight chat is the model preload, not a question
        self._llm_history = []        # plain question/answer turns; snapshots are not kept
        self._llm_question = ''
        self._llm_answer = ''
        self._llm_pending = ''        # streamed text not yet in the widget
        self._llm_look_inflight = False  # the in-flight chat is a camera-search VLM verdict
        self._look = None             # the active camera search (see _start_look), or None
        self._look_reply = ''
        self._found_objects = []      # finished searches that found something, newest last
        self._camera_checks = []      # every finished search's result text, newest last
        self._thumb_urls = deque()    # snapshot thumbnails currently held by LLM_chatlog
        self._thumb_seq = 0
        self._look_timeout_timer = QTimer(self)
        self._look_timeout_timer.setSingleShot(True)
        self._look_timeout_timer.setInterval(LOOK_SNAPSHOT_TIMEOUT_MS)
        self._look_timeout_timer.timeout.connect(self._on_look_snapshot_timeout)
        self._look_settle_timer = QTimer(self)
        self._look_settle_timer.setInterval(100)
        self._look_settle_timer.timeout.connect(self._look_settle_tick)
        self._llm_health_timer = QTimer(self)
        self._llm_health_timer.setInterval(LLM_HEALTH_INTERVAL_MS)
        self._llm_health_timer.timeout.connect(self._llm.check_health)
        self._llm_flush_timer = QTimer(self)
        self._llm_flush_timer.setSingleShot(True)
        self._llm_flush_timer.setInterval(LLM_FLUSH_MS)
        self._llm_flush_timer.timeout.connect(self._flush_llm_text)

        # Api_status is optional so a .ui without it cannot stop the station from
        # starting; the connection state then only shows in the chat log.
        self._api_status = getattr(self.ui, 'Api_status', None)
        if self._api_status is None:
            print("[LLM] no Api_status label in single_drone_flight.ui; "
                  "LLM connection state is shown in the chat log only")
        self.ui.LLM_chatlog.document().setMaximumBlockCount(LLM_CHATLOG_MAX_BLOCKS)
        self.ui.LLM_input.setPlaceholderText(
            "Ask about the drone's status. Enter sends, Shift+Enter adds a line.")
        self.ui.LLM_input.installEventFilter(self)
        self.ui.buttom_connect_VLM.clicked.connect(self._toggle_llm_connection)
        self.ui.LLM_send.clicked.connect(self._on_llm_send_clicked)
        # LLM Control (buttom_LLM_commit_2) gates camera-search moves and is OFF at every
        # launch; Commit Action (buttom_LLM_commit) sends the one pending move. Optional
        # like Api_status: without them the search still reports, but never moves.
        self._llm_control_button = getattr(self.ui, 'buttom_LLM_commit_2', None)
        self._llm_commit_button = getattr(self.ui, 'buttom_LLM_commit', None)
        self._llm_control_label = getattr(self.ui, 'LLM_control_status', None)
        self._llm_control = False
        if self._llm_control_button is None or self._llm_commit_button is None:
            print("[LLM] no LLM Control / Commit Action buttons in single_drone_flight.ui; "
                  "camera search will not propose moves")
        else:
            self._llm_control_button.setCheckable(True)
            self._llm_control_button.toggled.connect(self._set_llm_control)
            self._llm_commit_button.clicked.connect(lambda: self._on_look_move_answer(True))
        self._set_llm_control(False)
        self._update_commit_button()
        self._set_llm_state('disconnected')

    def eventFilter(self, obj, event):
        # Enter sends; Shift+Enter falls through and inserts a newline.
        if (obj is self.ui.LLM_input and event.type() == QEvent.KeyPress
                and event.key() in (Qt.Key_Return, Qt.Key_Enter)
                and not event.modifiers() & Qt.ShiftModifier):
            self._send_llm_question()
            return True
        return super().eventFilter(obj, event)

    def _set_llm_state(self, state, detail=''):
        self._llm_state = state
        if self._api_status is not None and detail:
            # Fit the error to the label as drawn (9 pt, whatever width Designer gives
            # it); the full message is in the chat log.
            font = QFont(self._api_status.font())
            font.setPointSize(9)
            detail = QFontMetrics(font).elidedText(
                detail, Qt.ElideRight, max(40, self._api_status.width() - 8))
        text, style = {
            'disconnected': ("LLM: not connected", "color: #555555;"),
            'connecting': ("LLM: connecting...", STATUS_STYLE['fair']),
            'loading': (f"LLM: loading model\n{LLM_MODEL}", STATUS_STYLE['fair']),
            'online': (f"LLM online\n{LLM_MODEL}", STATUS_STYLE['good']),
            'offline': (f"LLM unreachable\n{detail}", STATUS_STYLE['bad']),
        }[state]
        if self._api_status is not None:
            self._set_status_label(self._api_status, text, style + TWO_LINE_FONT)
        self.ui.buttom_connect_VLM.setText(
            "Connect to VLM" if state == 'disconnected' else "Disconnect")
        self._update_llm_buttons()

    def _update_llm_buttons(self):
        answering = (self._llm.busy() and not self._llm_warmup) or self._look is not None
        self.ui.LLM_send.setText("Stop" if answering else "Send")
        # A question during the preload is fine: it replaces the preload and loads the
        # model itself.
        self.ui.LLM_send.setEnabled(answering or self._llm_state in ('online', 'loading'))

    def _toggle_llm_connection(self):
        if self._llm_state == 'disconnected':
            self._set_llm_state('connecting')
            self._llm_chat_append(f"Connecting to {LLM_BASE_URL} ...", 'info')
            self._llm.check_health()
            self._llm_health_timer.start()
            return
        self._llm_health_timer.stop()
        self._abort_look("Camera search stopped: disconnected from the LLM.")
        self._set_llm_state('disconnected')  # first, so the abort below reports as such
        if self._llm.busy():
            self._llm.abort_chat('disconnected')
        self._llm_chat_append("Disconnected.", 'info')

    def _on_llm_health(self, ok, error, health):
        if self._llm_state == 'disconnected':
            return  # a check that was in flight when the operator disconnected
        was = self._llm_state
        # A line written now would land inside a streaming answer; Api_status shows it.
        quiet = self._llm.busy() and not self._llm_warmup
        if not ok:
            if was != 'offline' and not quiet:
                self._llm_chat_append(f"LLM server unreachable: {error}", 'error')
            self._set_llm_state('offline', error)
            return
        names = [m.get('name') for m in health.get('models', []) if isinstance(m, dict)]
        if LLM_MODEL not in names:
            if was != 'offline' and not quiet:
                self._llm_chat_append(
                    f"The LLM server is up but does not offer {LLM_MODEL} "
                    f"(it has: {', '.join(n for n in names if n) or 'nothing'}).", 'error')
            self._set_llm_state('offline', f"{LLM_MODEL} not on server")
            return
        if self._llm_warmup:
            return  # the preload's own result decides
        loaded = [m.get('name') if isinstance(m, dict) else m for m in health.get('loaded', [])]
        if LLM_MODEL in loaded or was == 'online' or self._llm.busy():
            # Online. If the model has since idled out (5 min), the next question
            # reloads it; the preload is only for a fresh connection.
            if was != 'online' and not quiet:
                self._llm_chat_append(f"Connected: {LLM_MODEL} is ready.", 'info')
            self._set_llm_state('online')
            return
        # Load the model now, so the first question does not wait 5-10 s for it. The
        # proxy rejects an empty message list, so this is a tiny real request.
        self._llm_warmup = True
        self._set_llm_state('loading')
        self._llm_chat_append(f"Loading {LLM_MODEL} on the LLM server (5-10 s) ...", 'info')
        self._llm.chat([{'role': 'user', 'content': 'Reply with the single word OK.'}])
        self._update_llm_buttons()

    def _set_llm_control(self, on):
        on = bool(on) and self._llm_control_button is not None and self._llm_commit_button is not None
        changed = on != self._llm_control
        self._llm_control = on
        button = self._llm_control_button
        if button is not None:
            if button.isChecked() != on:
                button.blockSignals(True)
                button.setChecked(on)
                button.blockSignals(False)
            button.setText("LLM Control: ON" if on else "LLM Control: OFF")
        if self._llm_control_label is not None:
            self._set_status_label(self._llm_control_label,
                                   "LLM control on" if on else "LLM control off",
                                   STATUS_STYLE['good'] if on else "color: #555555;")
        if changed:
            self.log_message("LLM control on: camera search may propose moves" if on
                             else "LLM control off")
            # Off means the LLM stops acting now: no pending move survives it, and a
            # search mid-move ends (the move already sent is not undone).
            if not on and self._look is not None and self._look['phase'] in ('confirm', 'moving'):
                self._end_look("Camera search stopped: LLM control was turned off.")

    def _update_commit_button(self):
        button = self._llm_commit_button
        if button is None:
            return
        look = self._look
        pending = look is not None and look['phase'] == 'confirm'
        button.setEnabled(pending)
        button.setText(f"Commit: {look['pending_move'][1]}" if pending else "Commit Action")

    def _on_llm_send_clicked(self):
        if self._look is not None:
            self._abort_look("Camera search stopped by the operator.")
        elif self._llm.busy() and not self._llm_warmup:
            self._llm.abort_chat('stopped')
        else:
            self._send_llm_question()

    def _send_llm_question(self):
        text = self.ui.LLM_input.toPlainText().strip()
        if not text or self._llm_state not in ('online', 'loading') or self._look is not None:
            return
        if self._llm.busy():
            if not self._llm_warmup:
                return  # an answer is still streaming
            self._llm.abort_chat('superseded')
        look_object = parse_look_request(text)
        if look_object is not None:
            # Straight to the camera, no routing call (see _LOOK_PATTERNS).
            self._llm_question = text
            self.ui.LLM_input.clear()
            self._llm_chat_append(f"You: {text}", 'user', bold=True)
            self._llm_chat_append("LLM: ", 'llm', bold=True)
            self._start_look(look_object, text)
            return
        snapshot = self._llm_telemetry_snapshot()
        compact = json.dumps(snapshot, separators=(',', ':'), ensure_ascii=False)
        prompt = (f"Telemetry snapshot:\n{compact}\n\n"
                  f"Operator question: {text}")
        messages = ([{'role': 'system', 'content': LLM_SYSTEM_PROMPT}]
                    + self._llm_history[-2 * LLM_HISTORY_TURNS:]
                    + [{'role': 'user', 'content': prompt}])
        if not self._llm.chat(messages):
            return
        self._llm_question = text
        self._llm_answer = ''
        self._llm_pending = ''
        self.ui.LLM_input.clear()
        self._llm_chat_append(f"You: {text}", 'user', bold=True)
        self._llm_chat_append("LLM: ", 'llm', bold=True)
        self._update_llm_buttons()

    def _on_llm_delta(self, chunk):
        if self._llm_warmup:
            return
        if self._llm_look_inflight:
            self._look_reply += chunk  # a verdict is JSON for the GUI, not for display
            return
        self._llm_answer += chunk
        if self._llm_answer.lstrip()[:1] in ('{', '`'):
            # Maybe a camera-look action (bare, or in a ``` fence as models often do):
            # hold it until complete so raw JSON never reaches the log.
            return
        self._llm_pending += chunk
        if not self._llm_flush_timer.isActive():
            self._llm_flush_timer.start()

    def _flush_llm_text(self):
        if self._llm_pending:
            self._llm_chat_insert(self._llm_pending)
            self._llm_pending = ''

    def _on_llm_finished(self, ok, error):
        if self._llm_warmup:
            self._llm_warmup = False
            if self._llm_state == 'disconnected':
                return
            if ok:
                self._llm_chat_append(
                    f"{LLM_MODEL} is loaded. Ask about the drone's status.", 'info')
                self._set_llm_state('online')
            elif error == 'superseded':
                self._set_llm_state('online')  # a question took over the preload
            else:
                self._llm_chat_append(f"Could not load {LLM_MODEL}: {error}", 'error')
                self._set_llm_state('offline', error)
            return
        if self._llm_look_inflight:
            self._llm_look_inflight = False
            if self._look is not None and self._look['phase'] == 'analyze':
                self._on_look_verdict(ok, error)
            self._update_llm_buttons()
            return
        self._llm_flush_timer.stop()
        held = self._llm_answer.lstrip()[:1] in ('{', '`')
        if ok and held:
            action = parse_json_object(self._llm_answer)
            obj = action.get('object') if action and action.get('action') == 'look' else None
            if isinstance(obj, str) and obj.strip():
                self._start_look(obj.strip()[:60], self._llm_question)
                return
        if held:
            self._llm_pending = self._llm_answer  # not an action after all: show it as text
        self._flush_llm_text()
        if ok:
            self._llm_history += [
                {'role': 'user', 'content': self._llm_question},
                {'role': 'assistant', 'content': self._llm_answer.strip()},
            ]
            self._llm_history = self._llm_history[-2 * LLM_HISTORY_TURNS:]
            self._render_llm_answer()
        else:
            self._llm_chat_insert(f"  [{error}]", 'error')
            if error not in ('stopped', 'disconnected', 'superseded'):
                # A transport failure: re-check now rather than at the next poll.
                self._llm.check_health()
        self._update_llm_buttons()

    def _render_llm_answer(self):
        # The model is told not to use markdown, but a 7B model still does for long
        # answers. Streamed text is shown raw; the finished answer is replaced by its
        # rendered form (lists, bold) so the log does not fill with ** and -.
        #
        # The answer is exactly the last len(answer) characters of the log (a newline is
        # one position, like the block break it becomes). That holds even while
        # LLM_CHATLOG_MAX_BLOCKS trims old lines off the top -- which a saved cursor
        # does not survive: Qt moves it when a newline is inserted at its position.
        # Only replace if the tail really is the answer, so nothing else is ever lost.
        cursor = QTextCursor(self.ui.LLM_chatlog.document())
        cursor.movePosition(QTextCursor.End)
        start = cursor.position() - len(self._llm_answer)
        if start < 0:
            return
        cursor.setPosition(start, QTextCursor.KeepAnchor)
        if cursor.selectedText().replace('\u2029', '\n') != self._llm_answer:
            return
        cursor.removeSelectedText()
        rendered = QTextDocument()
        rendered.setMarkdown(self._llm_answer.strip())
        cursor.insertFragment(QTextDocumentFragment(rendered))
        self._llm_chat_scroll()

    def _llm_text_format(self, kind, bold):
        fmt = QTextCharFormat()
        if kind is not None:
            fmt.setForeground(QColor(self.LLM_TEXT_COLORS[kind]))
        if bold:
            fmt.setFontWeight(QFont.Bold)
        return fmt

    def _llm_chat_append(self, text, kind=None, bold=False):
        """Start a new line in LLM_chatlog."""
        cursor = QTextCursor(self.ui.LLM_chatlog.document())
        cursor.movePosition(QTextCursor.End)
        if not self.ui.LLM_chatlog.document().isEmpty():
            cursor.insertBlock()
        cursor.insertText(text, self._llm_text_format(kind, bold))
        self._llm_chat_scroll()

    def _llm_chat_insert(self, text, kind=None):
        """Continue the current line (the streamed answer)."""
        cursor = QTextCursor(self.ui.LLM_chatlog.document())
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text, self._llm_text_format(kind, False))
        self._llm_chat_scroll()

    def _llm_chat_scroll(self):
        bar = self.ui.LLM_chatlog.verticalScrollBar()
        bar.setValue(bar.maximum())

    def _llm_telemetry_snapshot(self):
        """What this GUI currently knows, as a JSON-able dict for the LLM prompt.

        Copied from CommonData under its lock plus GUI-side state; nothing here touches
        rclpy. A source never received is reported as such, not as the zeros the data
        holders start with, and every source carries its age.
        """
        data = self.ros_object.data_struct
        # A bounded wait rather than the tick's bare tryLock: this runs once per
        # question, and a question should not go out without its telemetry.
        if not self.lock.tryLock(50):
            return {'error': 'telemetry unavailable (data lock busy); status is unknown'}
        try:
            last = dict(data.last_update)
            imu = (data.current_imu.roll, data.current_imu.pitch, data.current_imu.yaw)
            pos = (data.current_local_pos.x, data.current_local_pos.y, data.current_local_pos.z)
            vel = (data.current_vel.x, data.current_vel.y, data.current_vel.z)
            armed = data.current_state.armed
            mode = data.current_state.mode
            preflight = data.current_state.connected
            battery = (data.current_battery_status.percentage, data.current_battery_status.voltage)
            vehicle_name = data.current_vehicle_name
            yaw_align = data.current_yaw_align
            controller = data.current_controller_type
            motors = tuple(data.current_motor_commands)
            motors_time = data.last_motor_commands_time
            pos_err = (data.current_position_error.x, data.current_position_error.y,
                       data.current_position_error.z)
            system_status = data.current_system_status
            system_time = data.last_system_status_time
            optitrack_time = data.last_optitrack_time
            detections = next(reversed(data.vision_detections.values()), ())
            detections_time = data.last_vision_detections_time
        finally:
            self.lock.unlock()

        now = time.monotonic()
        no_data = 'no data received'

        def age(t):
            return round(now - t, 1) if t else None

        def r(value, digits=2):
            return None if value is None else round(float(value), digits)

        snap = {'ground_station_time': QDateTime.currentDateTime().toString('yyyy-MM-dd hh:mm:ss')}
        snap['vehicle'] = {
            'name': vehicle_name or None,
            'controller': (self._controller_display_name(controller)
                           if 'controller_type' in last else None),
            'px4_status': ({
                'armed': armed,
                'flight_mode': mode,
                'preflight_checks_pass': preflight,
                'yaw_aligned': bool(yaw_align),
                'age_s': age(last['state']),
            } if 'state' in last else no_data),
        }
        if 'odom' in last:
            snap['position_m'] = {'x': r(pos[0]), 'y': r(pos[1]), 'z_height': r(pos[2]),
                                  'age_s': age(last['odom'])}
            snap['velocity_mps'] = {'x': r(vel[0]), 'y': r(vel[1]), 'z': r(vel[2]),
                                    'speed': r(math.sqrt(sum(v * v for v in vel)))}
        else:
            snap['position_m'] = snap['velocity_mps'] = no_data
        if 'imu' in last:
            yaw = ((imu[2] + 180.0) % 360.0) - 180.0  # stored wrapped to [0, 360)
            snap['attitude_deg'] = {'roll': r(imu[0], 1), 'pitch': r(imu[1], 1),
                                    'yaw': r(yaw, 1), 'age_s': age(last['imu'])}
        else:
            snap['attitude_deg'] = no_data
        if self._last_cmd_time is not None:
            snap['last_position_command'] = {
                'x': self._last_x_cmd, 'y': self._last_y_cmd, 'z': self._last_z_cmd,
                'yaw_deg': r(self._last_yaw_cmd, 1), 'sent_s_ago': age(self._last_cmd_time)}
        else:
            snap['last_position_command'] = 'none sent this session'
        if 'position_error' in last:
            snap['position_error_m'] = {'x': r(pos_err[0]), 'y': r(pos_err[1]),
                                        'z': r(pos_err[2]), 'age_s': age(last['position_error'])}
        if 'battery' in last:
            fraction, volts = battery  # PX4 BatteryStatus.remaining: 0..1, negative = unknown
            snap['battery'] = {
                'remaining_pct': r(fraction * 100.0, 0) if fraction is not None and fraction >= 0 else None,
                'voltage_v': r(volts), 'age_s': age(last['battery'])}
        else:
            snap['battery'] = no_data
        if motors_time and now - motors_time < 0.5:
            snap['motor_commands_pct'] = [r(m * 100.0, 0) for m in motors]

        snap['optitrack'] = {
            'status': {True: 'normal', False: 'lost', None: 'unknown (station just started)'}[
                self._optitrack_ok],
            'last_frame_s_ago': age(optitrack_time) if optitrack_time else 'never received',
        }

        if system_status is None or now - system_time >= SYSTEM_STATUS_STALE_S:
            silent = ('never received' if system_status is None
                      else f'none for {int(now - system_time)} s')
            snap['wifi'] = {'level': 'bad',
                            'summary': f'No WiFi: reports from the drone computer: {silent}'}
            snap['orin_cpu'] = no_data
        else:
            wifi = system_status.get('wifi') or {}
            level, text = wifi_summary(wifi, self._wifi_backlog_reports)
            snap['wifi'] = {
                'level': level, 'summary': text.replace('\n', '; '),
                **{k: wifi.get(k) for k in ('ssid', 'signal_dbm', 'ping_ms', 'freq_mhz',
                                           'tx_mbps', 'rx_mbps', 'qdisc_backlog_max_pkts',
                                           'udp_txq_max_bytes')},
                'report_age_s': age(system_time)}
            cpu = system_status.get('cpu') or {}
            snap['orin_cpu'] = {
                'load_pct': cpu.get('load_pct'),
                'cores': 'cpu0-3 housekeeping, cpu4 uXRCE-DDS agent, cpu5 control node',
                'temp_c': cpu.get('temp_c')}
            snap['onboard_feed_status'] = {'mocap': system_status.get('mocap'),
                                           'odom': system_status.get('odom')}

        if detections_time:
            snap['camera_detections'] = {
                'objects': [{'label': d[0], 'confidence': r(d[1]), 'range_m': r(d[6])}
                            for d in detections],
                'age_s': age(detections_time)}
        else:
            snap['camera_detections'] = 'no detector data received'

        if self._camera_checks:
            snap['recent_camera_checks'] = self._camera_checks[-5:]
        if self._found_objects:
            snap['found_objects'] = self._found_objects[-5:]
        log = self.ui.list_cmd_log
        snap['recent_flight_log'] = [log.item(i).text()
                                     for i in range(max(0, log.count() - 8), log.count())]
        return snap

    # ------------------------------------------------------------------
    # Camera search ("is there a bottle in view?")
    #
    # The chat model routes the question here with {"action": "look", ...}. Then:
    #   1. snapshot: the Orin returns its latest frame + that frame's detections
    #      (camera and drone-body positions), with the /uav_0/mocap pose sampled on the
    #      ROS thread as the request left and as the reply arrived;
    #   2. verdict: the VLM sees the image and says whether the object is in view, and
    #      which one move from LOOK_MOVES would give a better view; the detector result
    #      is matched to the object by label, in code (_match_detection);
    #   3. found -> room position = R(q) p_body + t, stored in _found_objects; or
    #      not found -> if LLM Control is on, one move is proposed on the Commit Action
    #      button and sent ONLY when the operator clicks it, after the same checks as a
    #      manual command plus armed, OFFBOARD and OptiTrack. At most LOOK_MAX_MOVES.
    # Nothing moves the drone without that click. LLM Control (off at launch), Stop and
    # Disconnect end a search at any stage. There is no dialog, so every other control
    # (back to baseline included) stays usable while a move is pending.
    # ------------------------------------------------------------------

    def _start_look(self, obj, question):
        self._look = {
            'object': obj, 'question': question, 'moves': 0, 'snapshots': 0,
            'phase': None, 'request_id': None, 'snapshot': None, 'image': None,
            'pending_move': None, 'target': None, 'move_started': None,
            'settled_since': None,
        }
        self._llm_chat_insert(f"checking the camera for: {obj}.")
        self._look_capture()
        self._update_llm_buttons()

    def _look_capture(self):
        look = self._look
        look['snapshots'] += 1
        look['phase'] = 'capture'
        look['request_id'] = f"gs-{int(time.time() * 1000)}-{look['snapshots']}"
        self.ros_object.queue_snapshot_request(look['request_id'])
        self._look_timeout_timer.start()
        self._llm_chat_append(f"Camera: snapshot {look['snapshots']} requested ...", 'info')

    def _on_look_snapshot_timeout(self):
        if self._look is not None and self._look['phase'] == 'capture':
            self._end_look(
                f"No snapshot from the drone within {LOOK_SNAPSHOT_TIMEOUT_MS / 1000:.0f} s. "
                "Is the detector running with --ros on the Orin?", error=True)

    def _on_snapshot_received(self, response):
        look = self._look
        if (look is None or look['phase'] != 'capture'
                or response.get('id') != look['request_id']):
            return  # late, or someone else's
        self._look_timeout_timer.stop()
        if response.get('error'):
            self._end_look(f"The drone could not take a snapshot: {response['error']}.",
                           error=True)
            return
        try:
            image = QImage.fromData(base64.b64decode(response['jpeg_b64']))
        except (KeyError, TypeError, ValueError):
            image = QImage()
        if image.isNull():
            self._end_look("The snapshot from the drone was unreadable.", error=True)
            return
        look['snapshot'] = response
        look['image'] = image
        messages = [{'role': 'system', 'content': LOOK_SYSTEM_PROMPT},
                    {'role': 'user', 'content': f"Object to look for: {look['object']}",
                     'images': [response['jpeg_b64']]}]
        if not self._llm.chat(messages):
            self._end_look("The LLM is busy; ask again in a moment.", error=True)
            return
        look['phase'] = 'analyze'
        self._look_reply = ''
        self._llm_look_inflight = True
        self._llm_chat_append("VLM: looking at the image ...", 'info')

    def _on_look_verdict(self, ok, error):
        look = self._look
        if not ok:
            self._end_look(f"The VLM request failed: {error}.", error=True)
            return
        snapshot = look['snapshot']
        detections = snapshot.get('detections') or []
        verdict = parse_json_object(self._look_reply)
        if verdict is None:
            self._show_look_thumbnail(None)
            self._end_look(
                f"Could not read the VLM's verdict ({self._look_reply.strip()[:80]!r}). "
                f"The detector saw: {self._describe_detections(detections)}.", error=True)
            return
        visible = verdict.get('visible') is True
        move = verdict.get('move') if verdict.get('move') in LOOK_MOVES else None
        seen = self._look_reply[:self._look_reply.find('{')].strip().strip('`').strip()
        if seen:
            self._llm_chat_append(f"VLM: {seen[:200]}", 'info')
        match = self._match_detection(look['object'], detections)
        self._show_look_thumbnail(match)
        if match is not None:
            self._look_found(match, confirmed=visible)
            return
        classes = snapshot.get('detector_classes') or []
        if not any(object_matches_label(look['object'], c) for c in classes):
            self._end_look(
                f"The {look['object']} is {'in view' if visible else 'not in view'} "
                f"according to the VLM, but the detector cannot recognise it (it detects: "
                f"{', '.join(classes) or 'unknown'}), so there is no position for it.")
            return
        if move is None and not visible:
            # The VLM cannot know where an unseen object is, so it rarely suggests a move;
            # scan by turning instead. Still one confirmed step at a time.
            move = 'yaw_left'
        if move is None:
            self._end_look(f"The {look['object']} was seen but not detected, and the VLM "
                           "suggests no move that would help.")
            return
        if look['moves'] >= LOOK_MAX_MOVES:
            self._end_look(f"The {look['object']} was not localised after {LOOK_MAX_MOVES} moves.")
            return
        if not self._llm_control:
            self._end_look(
                f"The {look['object']} was {'seen but not detected' if visible else 'not found'}. "
                f"LLM control is off, so no move is proposed (the VLM would {LOOK_MOVES[move][0]}). "
                "Turn on LLM Control to let the search propose moves; each still needs Commit Action.")
            return
        self._propose_look_move(move, "it sees it, but it is not detected yet" if visible
                                else "not in view yet")

    @staticmethod
    def _match_detection(obj, detections):
        """The most confident detection whose label names the object ('blue bottle' ->
        'bottle'), or None. Deterministic on purpose: see LOOK_SYSTEM_PROMPT."""
        matches = [d for d in detections if d.get('label') and object_matches_label(obj, d['label'])]
        return max(matches, key=lambda d: d.get('confidence', 0.0)) if matches else None

    @staticmethod
    def _describe_detections(detections):
        return ', '.join(f"{d.get('label')} {d.get('confidence', 0):.2f}" for d in detections) or 'nothing'

    def _look_found(self, detection, confirmed):
        look = self._look
        response = look['snapshot']
        p_body = detection.get('position_body')
        pose = response.get('_pose_at_request')
        pose_after = response.get('_pose_at_response')
        world, note = None, ''
        if not confirmed:
            note = ("the VLM did not confirm it in the image; this rests on the detector "
                    f"alone (confidence {detection.get('confidence', 0):.2f})")
        if p_body is None:
            note = self._join_notes(note, "no camera-to-drone calibration on the Orin, so no room position")
        elif pose is None:
            note = self._join_notes(note, f"no OptiTrack pose on {MOCAP_POSE_TOPIC}, so no room position")
        elif pose['age_s'] > LOOK_POSE_MAX_AGE_S or self._optitrack_ok is not True:
            note = self._join_notes(note, "the OptiTrack pose is stale, so no room position")
        else:
            world = body_to_world(p_body, pose['position'], pose['orientation'])
            if pose_after is not None:
                shift = math.dist(pose['position'], pose_after['position'])
                dot = abs(float(np.dot(pose['orientation'], pose_after['orientation'])))
                turn = math.degrees(2.0 * math.acos(min(1.0, dot)))
                if shift > LOOK_STILL_M or turn > LOOK_STILL_DEG:
                    note = self._join_notes(note, "the drone was moving during the snapshot, so the position is approximate")
        record = {
            'object': look['object'], 'detector_label': detection.get('label'),
            'confidence': detection.get('confidence'),
            'room_xyz_m': [round(float(v), 2) for v in world] if world is not None else None,
            'from_drone_m': ({'ahead': round(p_body[0], 2), 'left': round(p_body[1], 2),
                              'up': round(p_body[2], 2)} if p_body is not None else None),
            'time': QDateTime.currentDateTime().toString('hh:mm:ss'),
            'moves_used': look['moves'],
            'vlm_confirmed': confirmed,
        }
        if note:
            record['note'] = note
        self._found_objects = (self._found_objects + [record])[-20:]
        text = (f"Found the {look['object']} (detector: {detection.get('label')} "
                f"{detection.get('confidence', 0):.2f}).")
        if world is not None:
            text += f" Room position x={world[0]:.2f}, y={world[1]:.2f}, z={world[2]:.2f} m."
        if p_body is not None:
            ahead, left, up = p_body
            text += (f" It is {abs(ahead):.2f} m {'ahead' if ahead >= 0 else 'behind'},"
                     f" {abs(left):.2f} m to the {'left' if left >= 0 else 'right'} and"
                     f" {abs(up):.2f} m {'above' if up >= 0 else 'below'} the drone.")
        if note:
            text += f" Note: {note}."
        self._end_look(text, found=True)

    @staticmethod
    def _join_notes(first, second):
        return f"{first}; {second}" if first else second

    def _look_move_base(self):
        """(x, y, z, yaw) to step from, or (None, why not). Same gates as a manual
        command (yaw aligned, geofence later) plus armed, OFFBOARD and OptiTrack."""
        data = self.ros_object.data_struct
        if not self.lock.tryLock(50):
            return None, "telemetry is busy"
        try:
            armed = data.current_state.armed
            mode = data.current_state.mode or ''
            odom_time = data.last_update.get('odom', 0.0)
            pos = (data.current_local_pos.x, data.current_local_pos.y, data.current_local_pos.z)
            yaw = ((data.current_imu.yaw + 180.0) % 360.0) - 180.0
        finally:
            self.lock.unlock()
        if armed is not True:
            return None, "the drone is not armed"
        if 'OFFBOARD' not in mode:
            return None, f"the drone is not in OFFBOARD mode ({mode or 'unknown'})"
        if not self._yaw_align:
            return None, "yaw is not aligned"
        if self._optitrack_ok is not True:
            return None, "OptiTrack is not normal"
        if not odom_time or time.monotonic() - odom_time > 0.5:
            return None, "there is no current position"
        # Step from the setpoint the drone is holding, if it is holding the last one we
        # sent; otherwise from where it is. Avoids creeping by the hover error each move.
        if (self._last_cmd_time is not None
                and math.dist(pos, (self._last_x_cmd, self._last_y_cmd, self._last_z_cmd)) < 0.3):
            return (self._last_x_cmd, self._last_y_cmd, self._last_z_cmd, self._last_yaw_cmd), None
        return (pos[0], pos[1], pos[2], yaw), None

    def _propose_look_move(self, move, reason):
        look = self._look
        desc, d_ahead, d_left, d_up, d_yaw = LOOK_MOVES[move]
        base, problem = self._look_move_base()
        if problem:
            self._end_look(f"The VLM suggests to {desc}, but {problem}. "
                           "Reposition the drone yourself and ask again.")
            return
        x0, y0, z0, yaw0 = base
        heading = math.radians(yaw0)
        x = round(x0 + d_ahead * math.cos(heading) - d_left * math.sin(heading), 2)
        y = round(y0 + d_ahead * math.sin(heading) + d_left * math.cos(heading), 2)
        z = round(z0 + d_up, 2)
        yaw = round(((yaw0 + d_yaw + 180.0) % 360.0) - 180.0, 1)
        if not self._within_geofence(x, y, z):
            self._end_look(f"The VLM suggests to {desc}, but that would leave the geofence.")
            return
        look['phase'] = 'confirm'
        look['pending_move'] = (move, desc, (x, y, z, yaw), base)
        number = look['moves'] + 1
        self._llm_chat_append(
            f"VLM suggests: {desc} (move {number} of {LOOK_MAX_MOVES}; {reason}). From "
            f"x={x0:.2f} y={y0:.2f} z={z0:.2f} yaw={yaw0:.1f} to x={x:.2f} y={y:.2f} "
            f"z={z:.2f} yaw={yaw:.1f}. Click Commit Action to move, or Stop.", 'info')
        # No dialog: the operator answers on the tab itself, so every other control
        # (back to baseline included) stays usable while a move is pending.
        self._update_commit_button()

    def _on_look_move_answer(self, accepted):
        look = self._look
        if look is None or look['phase'] != 'confirm':
            return
        if not accepted:
            self._end_look("Camera search stopped by the operator.")
            return
        move, desc, target, base = look['pending_move']
        # The dialog may have been open a while: re-check before anything is sent, and
        # never send a target computed from a pose the drone has since left.
        now_base, problem = self._look_move_base()
        if problem:
            self._end_look(f"Not moving: {problem}.")
            return
        drift = math.dist(now_base[:3], base[:3])
        turned = abs(((now_base[3] - base[3] + 180.0) % 360.0) - 180.0)
        if drift > 0.2 or turned > 10.0:
            self._end_look("Not moving: the drone moved while the confirmation was open. "
                           "Ask again for a fresh suggestion.")
            return
        look['moves'] += 1
        x, y, z, yaw = target
        self._issue_position_command(
            x, y, z, yaw, source=f"Camera search move {look['moves']}/{LOOK_MAX_MOVES} ({desc})")
        look['phase'] = 'moving'
        look['target'] = target
        look['move_started'] = time.monotonic()
        look['settled_since'] = None
        self._llm_chat_append(f"Moving: {desc} ...", 'info')
        self._look_settle_timer.start()
        self._update_commit_button()
        self._update_llm_buttons()

    def _look_settle_tick(self):
        look = self._look
        if look is None or look['phase'] != 'moving':
            self._look_settle_timer.stop()
            return
        if self._optitrack_ok is False:
            self._look_settle_timer.stop()
            self._end_look("OptiTrack was lost during the move; camera search stopped.",
                           error=True)
            return
        data = self.ros_object.data_struct
        if not self.lock.tryLock():
            return  # next tick
        pos = (data.current_local_pos.x, data.current_local_pos.y, data.current_local_pos.z)
        yaw = ((data.current_imu.yaw + 180.0) % 360.0) - 180.0
        self.lock.unlock()
        x, y, z, target_yaw = look['target']
        distance = math.dist(pos, (x, y, z))
        yaw_error = abs(((yaw - target_yaw + 180.0) % 360.0) - 180.0)
        now = time.monotonic()
        if distance <= LOOK_SETTLE_POS_M and yaw_error <= LOOK_SETTLE_YAW_DEG:
            if look['settled_since'] is None:
                look['settled_since'] = now
            elif now - look['settled_since'] >= LOOK_SETTLE_HOLD_S:
                self._look_settle_timer.stop()
                self._look_capture()
                return
        else:
            look['settled_since'] = None
        if now - look['move_started'] > LOOK_SETTLE_TIMEOUT_S:
            self._look_settle_timer.stop()
            self._llm_chat_append(
                f"Not settled after {LOOK_SETTLE_TIMEOUT_S:.0f} s ({distance:.2f} m, "
                f"{yaw_error:.0f} deg off the target); taking the snapshot anyway.", 'info')
            self._look_capture()

    def _show_look_thumbnail(self, detection):
        """Put the analysed frame in the chat log, with the matched box drawn."""
        look = self._look
        if look is None or look['image'] is None:
            return
        thumb = look['image'].convertToFormat(QImage.Format_RGB32)
        if detection is not None and detection.get('bbox'):
            x1, y1, x2, y2 = detection['bbox']
            painter = QPainter(thumb)
            painter.setPen(QPen(CameraDetectionView.BOX_COLOR, 3))
            painter.drawRect(QRectF(x1, y1, x2 - x1, y2 - y1))
            painter.end()
        thumb = thumb.scaledToWidth(200, Qt.SmoothTransformation)
        document = self.ui.LLM_chatlog.document()
        self._thumb_seq += 1
        url = QUrl(f"snapshot://{self._thumb_seq}")
        document.addResource(QTextDocument.ImageResource, url, thumb)
        self._thumb_urls.append(url)
        while len(self._thumb_urls) > LOOK_THUMBNAILS_KEPT:
            # Free the oldest thumbnail's pixels; its line keeps an empty 1x1 image.
            blank = QImage(1, 1, QImage.Format_ARGB32)
            blank.fill(Qt.transparent)
            document.addResource(QTextDocument.ImageResource, self._thumb_urls.popleft(), blank)
        cursor = QTextCursor(document)
        cursor.movePosition(QTextCursor.End)
        cursor.insertBlock()
        image_format = QTextImageFormat()
        image_format.setName(url.toString())
        image_format.setWidth(thumb.width())
        cursor.insertImage(image_format)
        self._llm_chat_scroll()

    def _end_look(self, text, error=False, found=False):
        look = self._look
        if look is None:
            return
        self._look = None
        self._look_timeout_timer.stop()
        self._look_settle_timer.stop()
        self._update_commit_button()
        self._llm_chat_append(text, 'error' if error else 'llm', bold=found)
        self.log_message(f"Camera search ({look['object']}): {text}")
        self._camera_checks = (self._camera_checks + [{
            'object': look['object'], 'time': QDateTime.currentDateTime().toString('hh:mm:ss'),
            'result': text}])[-10:]
        # History keeps what the model actually replied (the action), so it keeps routing
        # repeat questions to the camera; the result reaches it via recent_camera_checks.
        # (With the result text as its reply, 'did you find a person earlier?' got
        # "no data received" -- tested 2026-09-27.)
        self._llm_history += [{'role': 'user', 'content': look['question']},
                              {'role': 'assistant', 'content': json.dumps(
                                  {'action': 'look', 'object': look['object']})}]
        self._llm_history = self._llm_history[-2 * LLM_HISTORY_TURNS:]
        self._update_llm_buttons()

    def _abort_look(self, reason):
        if self._look is None:
            return
        analyzing = self._look['phase'] == 'analyze'
        self._end_look(reason)
        if analyzing and self._llm.busy():
            self._llm.abort_chat('stopped')  # its finish is ignored: no search is active

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

    # ------------------------------------------------------------------
    # Torque-mode CCM: "CCM Flight Log" (Additional_Function) and "CCM Trajectory"
    # (tabWidget).
    #
    # Data: quadrotor_planner/ccm_reference (x*) against ccm_direct_actuation/state (x),
    # the pair the model loader evaluates the controller on. The two streams (100 Hz and
    # 250 Hz) are each carried forward by their own velocity to the GUI tick, so they are
    # compared at one instant rather than up to 10 ms apart. The plots keep the Flight
    # Log conventions: a fixed seconds-ago x-axis, isVisible() gating, decimated redraw.
    #
    # Buttons: Trigger services of quadrotor_ccm_planner, queued to the ROS thread. The
    # planner only moves the reference it streams -- it never arms, changes the PX4 mode
    # or switches the controller. Engaging CCM stays on the Controller tab (with its
    # confirmation) and the always-available "Back to Baseline Control" button.
    # ------------------------------------------------------------------
    CCM_PHASES = {0: "IDLE", 1: "HOLD", 2: "RUNNING", 3: "FINISHED", 4: "TRANSITION"}

    def _setup_ccm_flight_log(self):
        self._ccm_t0 = None
        self._ccm_t = deque()
        # actual X/Y/Z, reference X/Y/Z, error X/Y/Z, |error|
        self._ccm_series = [deque() for _ in range(10)]
        self._ccm_tick = 0
        self._ccm_run = None        # stats of the run in progress (planner RUNNING)
        self._ccm_last_run = ""
        self._ccm_last_run_short = ""
        self._ccm_prev_phase = None
        self._ccm_mode = ""
        self._ccm_path_key = None
        self._ccm_error_now = None
        if not _HAS_PYQTGRAPH:
            return
        try:
            self._setup_ccm_flight_log_impl()
        except Exception as e:
            import traceback
            print(f"[PLOT] CCM flight log setup failed: {e}")
            traceback.print_exc()

    @staticmethod
    def _ccm_plot(container, left_label, bottom_label):
        plot = pg.PlotWidget(parent=container)
        plot.setGeometry(container.rect())
        plot.setBackground('w')
        plot.getPlotItem().layout.setContentsMargins(0, 15, 0, 0)
        # Units in the label text with SI prefixes off, as on the Flight Log plots.
        plot.setLabel('left', left_label)
        plot.setLabel('bottom', bottom_label)
        plot.getAxis('left').enableAutoSIPrefix(False)
        plot.getAxis('bottom').enableAutoSIPrefix(False)
        plot.show()
        return plot

    def _setup_ccm_flight_log_impl(self):
        dashed = Qt.DashLine
        colours = ('#CC0000', '#008800', '#0055AA')

        # Reference (dashed) vs actual (solid) X/Y/Z.
        p = self._ccm_xyz_plot = self._ccm_plot(
            self.ui.display_ccm_xyz, 'Position (m)', 'Time (s, relative)')
        p.setXRange(-CCM_PLOT_HISTORY_S, 0, padding=0)
        p.addLegend(offset=(5, -5), colCount=3)
        self._ccm_curves_act = [
            p.plot(pen=pg.mkPen(c, width=2), name=f'{a}', connect='finite')
            for c, a in zip(colours, 'XYZ')]
        self._ccm_curves_ref = [
            p.plot(pen=pg.mkPen(c, width=2, style=dashed), name=f'{a} ref', connect='finite')
            for c, a in zip(colours, 'XYZ')]

        # Top-down path: geofence, the planner's preview of the selected shape, the
        # reference and actual trails, and the start / hover points. The view is set
        # only when the preview changes (never per tick) and stays aspect-locked.
        xy = self._ccm_xy_plot = self._ccm_plot(self.ui.display_ccm_xy, 'Y (m)', 'X (m)')
        xy.setAspectLocked(True)
        xy.disableAutoRange()
        gx, gy = float(self.ros_object.config[0]), float(self.ros_object.config[1])
        xy.plot([gx, -gx, -gx, gx, gx], [gy, gy, -gy, -gy, gy], pen=pg.mkPen('#999999', width=1))
        self._ccm_xy_path = xy.plot(pen=pg.mkPen('#999999', width=1, style=dashed))
        self._ccm_xy_ref = xy.plot(pen=pg.mkPen('#0055AA', width=2, style=dashed), connect='finite')
        self._ccm_xy_act = xy.plot(pen=pg.mkPen('#CC0000', width=2), connect='finite')
        self._ccm_xy_marks = pg.ScatterPlotItem(pxMode=True)
        xy.addItem(self._ccm_xy_marks)
        self._ccm_xy_now = pg.ScatterPlotItem(size=8, brush=pg.mkBrush('#CC0000'), pen=None)
        xy.addItem(self._ccm_xy_now)
        xy.setRange(xRange=(-gx, gx), yRange=(-gy, gy), padding=0.02)

        # Tracking error, reference minus actual.
        e = self._ccm_err_plot = self._ccm_plot(
            self.ui.display_ccm_err, 'Error (m)', 'Time (s, relative)')
        e.setXRange(-CCM_PLOT_HISTORY_S, 0, padding=0)
        e.addLegend(offset=(5, -5), colCount=2)
        # Transparent anchors keep the autoscaled span at least +-5 cm, so hover noise
        # does not fill the plot (the angle plot pins its range for the same reason).
        e.plot([-CCM_PLOT_HISTORY_S, 0], [-0.05, 0.05], pen=pg.mkPen((0, 0, 0, 0)))
        self._ccm_err_curves = [
            e.plot(pen=pg.mkPen(c, width=1), name=f'e{a}', connect='finite')
            for c, a in zip(colours, 'xyz')]
        self._ccm_err_norm = e.plot(pen=pg.mkPen('#000000', width=2), name='|e|', connect='finite')

    @staticmethod
    def _ccm_now(sample, arrival, now):
        """Position carried forward to `now` by the sample's own velocity (<= 50 ms)."""
        if sample is None or now - arrival > CCM_STREAM_FRESH_S:
            return None
        dt = min(max(now - arrival, 0.0), 0.05)
        return (sample[0] + sample[3] * dt, sample[1] + sample[4] * dt, sample[2] + sample[5] * dt)

    def _append_ccm_flight_log(self, ccm, now):
        nan = float('nan')
        act = self._ccm_now(ccm['state'], ccm['state_time'], now)
        ref = self._ccm_now(ccm['ref'], ccm['ref_time'], now)
        phase = ccm['ref'][6] if (ccm['ref'] is not None and ref is not None) else None
        err = None
        if act is not None and ref is not None:
            err = tuple(r - a for r, a in zip(ref, act))
        self._ccm_error_now = err
        self._track_ccm_run(phase, ccm, err)

        if self._ccm_t0 is None:
            self._ccm_t0 = now
        t = now - self._ccm_t0
        self._ccm_t.append(t)
        row = ((act or (nan,) * 3) + (ref or (nan,) * 3) + (err or (nan,) * 3)
               + ((math.sqrt(sum(v * v for v in err)) if err else nan),))
        for series, value in zip(self._ccm_series, row):
            series.append(value)
        cutoff = t - CCM_PLOT_HISTORY_S
        while self._ccm_t and self._ccm_t[0] < cutoff:
            self._ccm_t.popleft()
            for series in self._ccm_series:
                series.popleft()

        if not _HAS_PYQTGRAPH or not hasattr(self, '_ccm_xyz_plot'):
            return
        self._update_ccm_xy_preview(ccm['info'])
        visible = (self._ccm_xyz_plot.isVisible(), self._ccm_xy_plot.isVisible(),
                   self._ccm_err_plot.isVisible())
        if not any(visible):
            return
        self._ccm_tick += 1
        if self._ccm_tick % self.PLOT_REDRAW_EVERY:
            return
        t_list = [ti - t for ti in self._ccm_t]
        s = [list(series) for series in self._ccm_series]
        if visible[0]:
            for i in range(3):
                self._ccm_curves_act[i].setData(t_list, s[i])
                self._ccm_curves_ref[i].setData(t_list, s[3 + i])
        if visible[1]:
            self._ccm_xy_act.setData(s[0], s[1])
            self._ccm_xy_ref.setData(s[3], s[4])
            if act:
                self._ccm_xy_now.setData([act[0]], [act[1]])
            else:
                self._ccm_xy_now.setData([], [])
        if visible[2]:
            for i in range(3):
                self._ccm_err_curves[i].setData(t_list, s[6 + i])
            self._ccm_err_norm.setData(t_list, s[9])

    def _update_ccm_xy_preview(self, info):
        """Planner's XY path of the selection + start / hover marks; re-ranges on change."""
        path = (info or {}).get('path') or []
        start = (info or {}).get('start')
        hover = (info or {}).get('hover')
        key = (len(path), tuple(path[0]) if path else None, tuple(path[-1]) if path else None,
               tuple(start or ()), tuple(hover or ()))
        if key == self._ccm_path_key:
            return
        self._ccm_path_key = key
        self._ccm_xy_path.setData([p[0] for p in path], [p[1] for p in path])
        spots = []
        if start:
            spots.append({'pos': (start[0], start[1]), 'size': 11, 'symbol': 'o',
                          'brush': pg.mkBrush('#24A148'), 'pen': None})
        if hover:
            spots.append({'pos': (hover[0], hover[1]), 'size': 11, 'symbol': 'x',
                          'brush': pg.mkBrush('#222222'), 'pen': pg.mkPen('#222222')})
        self._ccm_xy_marks.setData(spots)
        xs = [p[0] for p in path] + ([start[0]] if start else []) + ([hover[0]] if hover else [])
        ys = [p[1] for p in path] + ([start[1]] if start else []) + ([hover[1]] if hover else [])
        if xs:
            margin = 0.5
            self._ccm_xy_plot.setRange(xRange=(min(xs) - margin, max(xs) + margin),
                                       yRange=(min(ys) - margin, max(ys) + margin), padding=0)

    def _track_ccm_run(self, phase, ccm, err):
        """RMS / max error over each planner RUNNING segment, logged when it ends."""
        # (Mode changes are already logged through controller_type.)
        mode = ccm['mode']
        running = phase == 2
        if running and self._ccm_run is None:
            info = ccm['info'] or {}
            self._ccm_run = {'name': ccm['ref'][7], 'scale': info.get('time_scale'),
                             'sq': 0.0, 'max': 0.0, 'n': 0, 'modes': set()}
        if self._ccm_run is not None and running:
            self._ccm_run['modes'].add(mode or '?')
            if err is not None:
                e2 = sum(v * v for v in err)
                self._ccm_run['sq'] += e2
                self._ccm_run['max'] = max(self._ccm_run['max'], math.sqrt(e2))
                self._ccm_run['n'] += 1
        if self._ccm_run is not None and not running:
            run, self._ccm_run = self._ccm_run, None
            if run['n'] > 0:
                scale = f" x{run['scale']:.2f}" if isinstance(run['scale'], (int, float)) else ""
                modes = "/".join(sorted(run['modes']))
                rms = 100 * math.sqrt(run['sq'] / run['n'])
                self._ccm_last_run = (f"{run['name']}{scale} [{modes}]: RMS {rms:.1f} cm, "
                                      f"max {100 * run['max']:.1f} cm")
                self._ccm_last_run_short = f"{run['name']}{scale}: RMS {rms:.1f} cm"
                self.log_message(f"CCM run finished: {self._ccm_last_run}")
        self._ccm_prev_phase = phase

    def _ccm_status_text(self, ccm):
        err = self._ccm_error_now
        info = ccm['info'] if ccm['info_fresh'] else None
        parts = [f"Node: {ccm['mode'] or '--'}",
                 f"Planner: {info.get('phase', '--') if info else '--'}",
                 f"|e| {100 * math.sqrt(sum(v * v for v in err)):.1f} cm" if err else "|e| --"]
        if self._ccm_run is not None and self._ccm_run['n']:
            parts.append(f"run RMS {100 * math.sqrt(self._ccm_run['sq'] / self._ccm_run['n']):.1f} cm")
        elif self._ccm_last_run:
            parts.append(f"last {self._ccm_last_run_short}")
        return "  ·  ".join(parts)

    # --- CCM Trajectory tab -------------------------------------------------------
    def _setup_ccm_trajectory_tab(self):
        self._ccm_traj_seq = -1
        self._ccm_info_seq = -1
        self._ccm_ts_range_set = False
        # (value, time) the operator last sent, until the planner's info echoes it.
        self._ccm_ts_sent = None
        self._ccm_info = None
        self._ccm_info_phase = None
        self._ccm_pending = set()
        combo = self.ui.combo_ccm_trajectory
        combo.clear()
        combo.addItem("(waiting for the planner)", None)
        combo.setEnabled(False)
        self.ui.progress_ccm.setValue(0)
        self.ui.label_ccm_info.setText("")
        self._update_ccm_time_scale_label()
        self._update_ccm_buttons(None, False)

    def _ccm_selected_name(self):
        return self.ui.combo_ccm_trajectory.currentData()

    def _ccm_slider_scale(self):
        return self.ui.slider_ccm_time_scale.value() / 100.0

    def _on_ccm_trajectory_selected(self, _index):
        # `activated` fires on user choice only, so this never echoes a programmatic sync.
        name = self._ccm_selected_name()
        if name:
            self.ros_object.queue_ccm_select(name)
            self.log_message(f"CCM trajectory selected: {name}")

    def _on_ccm_time_scale_moved(self, _value):
        self._update_ccm_time_scale_label()
        if not self.ui.slider_ccm_time_scale.isSliderDown():
            self._send_ccm_time_scale()  # keyboard / click steps; a drag sends on release

    def _send_ccm_time_scale(self):
        scale = self._ccm_slider_scale()
        self._ccm_ts_sent = (scale, time.monotonic())
        self.ros_object.queue_ccm_time_scale(scale)

    def _update_ccm_time_scale_label(self):
        scale = self._ccm_slider_scale()
        info = self._ccm_info or {}
        text = f"Time scale {scale:.2f}x"
        style = ""
        ts0 = info.get('time_scale')
        if isinstance(ts0, (int, float)) and ts0 > 0 and 'lap_time' in info:
            # Lap time goes as 1/s, cruise speed as s and acceleration as s^2.
            k = scale / ts0
            lap = info['lap_time'] / k
            v = info.get('cruise_speed', 0.0) * k
            a = info.get('cruise_accel', 0.0) * k * k
            text += f"  · lap {lap:.1f} s · {v:.2f} m/s · {a:.2f} m/s²"
            # Yaw-along-path shapes: yaw rate also goes as s, yaw acceleration as s^2.
            yr = info.get('yaw_rate_max', 0.0) * k
            ya = info.get('yaw_accel_max', 0.0) * k * k
            if info.get('yaw_tangent'):
                text += f" · yaw {yr:.2f} rad/s"
            if (v > info.get('max_speed', float('inf')) or a > info.get('max_accel', float('inf'))
                    or yr > info.get('max_yaw_rate', float('inf')) or ya > info.get('max_yaw_accel', float('inf'))):
                text += "  OVER LIMITS"
                style = "color: #d9534f; font-weight: bold"
        self.ui.label_ccm_time_scale.setText(text)
        self.ui.label_ccm_time_scale.setStyleSheet(style)

    def _ccm_call(self, action, label):
        self._ccm_pending.add(action)
        self.log_message(f"CCM planner: {label}...")
        self.ros_object.queue_ccm_planner_call(action)
        self._update_ccm_buttons(self._ccm_info, self.ros_object.ccm_planner_ready())

    def _request_ccm_hold(self):
        self._ccm_call('hold', "hold here")

    def _request_ccm_fly_to_start(self):
        self._ccm_call('go_to_start', f"fly to the {self._ccm_selected_name()} start")

    def _request_ccm_start(self):
        mode = self._ccm_mode
        if mode != "CCM":
            answer = QMessageBox.question(
                None, "CCM is not engaged",
                f"The CCM node is in {mode or 'an unknown mode'}, so the SAFETY baseline "
                "would fly this trajectory (it follows the planner's position reference).\n\n"
                "Engage CCM on the Controller tab first, or start anyway?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                self.log_message("CCM trajectory start cancelled (CCM not engaged)")
                return
        self._ccm_call('start', f"start {self._ccm_selected_name()} x{self._ccm_slider_scale():.2f}")

    def _request_ccm_back_to_hover(self):
        # The safe direction: one click, no dialog.
        self._ccm_call('back_to_hover', "back to hover")

    def _request_ccm_release(self):
        # Only offered outside CCM: in CCM, a released (idle) reference is an abort.
        self._ccm_call('release', "release (stop streaming)")

    def _handle_ccm_planner_result(self, action, success, message):
        self._ccm_pending.discard(action)
        self.log_message(f"CCM planner {action}: {'OK' if success else 'REFUSED'} - {message}")

    def _update_ccm_trajectory_tab(self, ccm):
        ready = self.ros_object.ccm_planner_ready()
        info = ccm['info'] if ccm['info_fresh'] else None

        if ccm['traj_seq'] != self._ccm_traj_seq and ccm['trajectories']:
            self._ccm_traj_seq = ccm['traj_seq']
            combo = self.ui.combo_ccm_trajectory
            combo.blockSignals(True)
            combo.clear()
            for entry in ccm['trajectories']:
                combo.addItem(entry.get('label') or entry['name'], entry['name'])
            combo.blockSignals(False)
            self._ccm_info_seq = -1  # re-sync the selection below

        if ccm['info_seq'] != self._ccm_info_seq and info is not None:
            self._ccm_info_seq = ccm['info_seq']
            self._ccm_info = info
            self._sync_ccm_controls(info)
        elif info is None:
            self._ccm_info = None

        phase = info.get('phase') if info else None
        if phase != self._ccm_info_phase:
            if self._ccm_info_phase is not None or phase is not None:
                self.log_message(f"CCM planner: {self._ccm_info_phase or 'offline'} -> {phase or 'offline'}")
            self._ccm_info_phase = phase

        if info is None:
            self.ui.label_ccm_status.setText(
                "Planner: no data (is quadrotor_ccm_planner running?)")
            self.ui.label_ccm_info.setText("")
            self.ui.progress_ccm.setValue(0)
        else:
            self.ui.label_ccm_status.setText(f"Planner: {ccm['status']}")
            parts = []
            if info.get('start'):
                parts.append("start [" + ", ".join(f"{v:.2f}" for v in info['start']) + "]")
            if info.get('hover'):
                parts.append("hover [" + ", ".join(f"{v:.2f}" for v in info['hover']) + "]")
            if 'duration' in info:
                parts.append(f"run {info['duration']:.1f} s")
            if info.get('pos_ref'):
                parts.append("SAFETY baseline follows the planner")
            self.ui.label_ccm_info.setText("   ".join(parts))
            T = info.get('T') or 0.0
            if phase in ("RUNNING", "TRANSITION") and T > 0:
                self.ui.progress_ccm.setValue(int(round(100 * min(1.0, info.get('t', 0.0) / T))))
            elif phase == "FINISHED":
                self.ui.progress_ccm.setValue(100)
            else:
                self.ui.progress_ccm.setValue(0)
        self._update_ccm_buttons(info, ready)

    def _sync_ccm_controls(self, info):
        combo = self.ui.combo_ccm_trajectory
        # Follow the planner's selection unless the operator has the list open.
        selected = info.get('selected')
        if selected and not combo.view().isVisible():
            index = combo.findData(selected)
            if index >= 0 and index != combo.currentIndex():
                combo.blockSignals(True)
                combo.setCurrentIndex(index)
                combo.blockSignals(False)
        slider = self.ui.slider_ccm_time_scale
        lo, hi = info.get('time_scale_min'), info.get('time_scale_max')
        if not self._ccm_ts_range_set and isinstance(lo, (int, float)) and isinstance(hi, (int, float)):
            slider.blockSignals(True)
            slider.setRange(int(round(100 * lo)), int(round(100 * hi)))
            slider.blockSignals(False)
            self._ccm_ts_range_set = True
        ts = info.get('time_scale')
        if isinstance(ts, (int, float)) and not slider.isSliderDown():
            if self._ccm_ts_sent is not None:
                sent, when = self._ccm_ts_sent
                if abs(ts - sent) < 0.005:
                    self._ccm_ts_sent = None  # echoed: the planner has it
                elif time.monotonic() - when > 2.0:
                    self._ccm_ts_sent = None
                    self.log_message(f"CCM time scale {sent:.2f}x not applied; planner has {ts:.2f}x")
            if self._ccm_ts_sent is None:
                slider.blockSignals(True)
                slider.setValue(int(round(100 * ts)))
                slider.blockSignals(False)
        self._update_ccm_time_scale_label()

    def _update_ccm_buttons(self, info, ready):
        phase = (info or {}).get('phase')
        live = info is not None and ready
        idle_like = phase in ("IDLE", "HOLD", "FINISHED")
        selected = self._ccm_selected_name()
        # The selection and time scale the planner reports must match what is on
        # screen before a move or start, since a topic and a service call are not
        # ordered with respect to each other.
        in_sync = (info is not None and selected == info.get('selected')
                   and self._ccm_ts_sent is None
                   and abs(self._ccm_slider_scale() - (info.get('time_scale') or 0.0)) < 0.005)
        pending = self._ccm_pending
        self.ui.combo_ccm_trajectory.setEnabled(live and idle_like and selected is not None)
        self.ui.slider_ccm_time_scale.setEnabled(info is not None and ready and phase != "RUNNING")
        self.ui.buttom_ccm_hold.setEnabled(live and idle_like and 'hold' not in pending)
        self.ui.buttom_ccm_fly_to_start.setEnabled(
            live and idle_like and in_sync and not info.get('at_start', False)
            and 'go_to_start' not in pending)
        can_start = (live and phase in ("HOLD", "FINISHED") and in_sync and info.get('at_start', False)
                     and info.get('within_limits', False) and 'start' not in pending)
        self.ui.buttom_ccm_start.setEnabled(can_start)
        self.ui.buttom_ccm_start.setStyleSheet(
            "background-color: #24A148; color: white; font-weight: bold;" if can_start else "")
        back = (live and 'back_to_hover' not in pending
                and (phase == "RUNNING" or (phase in ("HOLD", "FINISHED") and bool(info.get('hover')))))
        self.ui.buttom_ccm_back_to_hover.setEnabled(back)
        self.ui.buttom_ccm_back_to_hover.setStyleSheet(
            "background-color: #f0ad4e; font-weight: bold;" if back else "")
        self.ui.buttom_ccm_release.setEnabled(
            live and phase not in (None, "IDLE") and self._ccm_mode != "CCM" and 'release' not in pending)

    # define the signal-slot combination of ros and pyqt GUI
    def set_ros_callbacks(self):
        # feedbacks from ros
        self.ros_object.connect_update_gui(self.update_gui_data)
        self.ros_object.direct_mode_result.connect(self._handle_direct_mode_result)
        self.ros_object.controllers_listed.connect(self._handle_controllers_listed)
        self.ros_object.controller_activated.connect(self._handle_controller_activated)
        self.ros_object.snapshot_received.connect(self._on_snapshot_received)
        self.ros_object.ccm_planner_result.connect(self._handle_ccm_planner_result)

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
        # CCM Trajectory tab
        self.ui.combo_ccm_trajectory.activated.connect(self._on_ccm_trajectory_selected)
        self.ui.slider_ccm_time_scale.valueChanged.connect(self._on_ccm_time_scale_moved)
        self.ui.slider_ccm_time_scale.sliderReleased.connect(self._send_ccm_time_scale)
        self.ui.buttom_ccm_hold.clicked.connect(self._request_ccm_hold)
        self.ui.buttom_ccm_fly_to_start.clicked.connect(self._request_ccm_fly_to_start)
        self.ui.buttom_ccm_start.clicked.connect(self._request_ccm_start)
        self.ui.buttom_ccm_back_to_hover.clicked.connect(self._request_ccm_back_to_hover)
        self.ui.buttom_ccm_release.clicked.connect(self._request_ccm_release)

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
        ds = self.ros_object.data_struct
        ccm = {
            'ref': ds.ccm_ref, 'ref_time': ds.ccm_ref_time,
            'state': ds.ccm_state, 'state_time': ds.ccm_state_time,
            'mode': ds.ccm_mode, 'status': ds.ccm_planner_status,
            'info': ds.ccm_planner_info, 'info_time': ds.ccm_planner_info_time,
            'info_seq': ds.ccm_planner_info_seq,
            'trajectories': ds.ccm_trajectories, 'traj_seq': ds.ccm_trajectories_seq,
        }
        self.lock.unlock()

        now = time.monotonic()
        ccm['info_fresh'] = (ccm['info'] is not None
                             and now - ccm['info_time'] < CCM_PLANNER_INFO_FRESH_S)
        self._ccm_mode = ccm['mode']
        self._append_ccm_flight_log(ccm, now)
        self._update_ccm_trajectory_tab(ccm)
        self.ui.label_ccm_log_status.setText(self._ccm_status_text(ccm))

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
        if not self._within_geofence(x, y, z):
            ## pop up dialog
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Position is outside the geofence bounds")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        self._issue_position_command(x, y, z, yaw)

        if self._pref_armed:
            self._pref_armed = False
            self.ui.buttom_enable_log.setText("Enable")
            self._start_step_response(x, y, z)
            self.log_message("Step response logging started")

    def _within_geofence(self, x, y, z):
        return (abs(x) <= float(self.ros_object.config[0])
                and abs(y) <= float(self.ros_object.config[1])
                and abs(z) <= float(self.ros_object.config[2]) and z > 0)

    def _issue_position_command(self, x, y, z, yaw, source="Position command sent"):
        """The one way a position setpoint leaves the GUI (operator or camera search)."""
        # Queued, not published inline: this runs on the Qt thread and must not touch
        # rclpy. It is issued by _drain_requests() on the next ROS tick (<=33 ms).
        self.ros_object.queue_coordinates(x, y, z, yaw)
        self.log_message(f"{source}: {x}, {y}, {z}, {yaw}")

        # Update the dashed command lines in the X/Y/Z plot to the setpoint sent.
        self._last_x_cmd = x
        self._last_y_cmd = y
        self._last_z_cmd = z
        self._last_yaw_cmd = ((yaw + 180.0) % 360.0) - 180.0
        self._last_cmd_time = time.monotonic()

    def get_coordinates(self):
        # get current relative position
        self.ui.XPositionUAV.setText("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.YPositionUAV.setText("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.ZPositionUAV.setText("{:.2f}".format(self.local_pos_msg.z, 2))
        self.ui.YAWUAV.setText("{:.2f}".format(self.imu_msg.yaw, 2))
