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
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QDateTime, QTimer, Qt, QPointF, QRectF
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QMessageBox, QWidget
import Common
from geometry_msgs.msg import Point

try:
    import os
    os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')
    import pyqtgraph as pg
    _HAS_PYQTGRAPH = True
except ImportError:
    _HAS_PYQTGRAPH = False

POSITION_PLOT_HISTORY_S = 10.0  # seconds of history shown in the X/Y/Z/Yaw plot
# from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandLong, SetMode
from px4_msgs.msg import ActuatorMotors, VehicleStatus,VehicleAttitudeSetpoint,VehicleAttitude, VehicleGlobalPosition, BatteryStatus,VehicleRatesSetpoint, EstimatorStatusFlags
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference, VehicleInfo

# from mavros_msgs.msg import State, AttitudeTarget
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import json
# from fsc_autopilot_msgs.msg import TrackingReference
from std_msgs.msg import Bool, String


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


class SingleDroneRosNode(Node, QObject):
    ## define signals
    update_data = pyqtSignal(int)

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
        
        # Define subscribers
        self.imu_sub = self.create_subscription(VehicleAttitude, '/uav_0/fmu/out/vehicle_attitude', self.imu_callback, self.px4_qos_profile)
        self.pos_global_sub = self.create_subscription(VehicleGlobalPosition, '/uav_0/fmu/out/vehicle_global_position', self.pos_global_callback, self.px4_qos_profile)
        self.pos_local_adjusted_sub = self.create_subscription(Odometry, '/uav_0/state_estimator/local_position/odom', self.pos_local_callback, 10)
        self.vel_sub = self.create_subscription(Odometry, '/uav_0/state_estimator/local_position/odom', self.vel_callback, 10)
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
        self.motor_commands_sub = self.create_subscription(
            ActuatorMotors,
            '/uav_0/fsc_autopilot_ros2/direct_actuation/motors_debug',
            self.motor_commands_callback,
            10
        )

        # Define publishers / services
        # self.coords_pub = self.create_publisher(TrackingReference, 'position_controller/target', 10)
        self.geofence_pub = self.create_publisher(Marker, 'tracking_controller/geofence', 10)
        self.position_com_pub = self.create_publisher(
            PositionControllerReference, '/uav_0/fsc_autopilot_ros2/position_controller/reference', 10)

        # self.set_home_service = self.create_client(CommandHome, 'mavros/cmd/set_home')

        # self.arming_service = self.create_client(CommandLong, 'mavros/cmd/command')
        # self.land_service = self.create_client(CommandLong, 'mavros/cmd/command')
        # self.set_mode_service = self.create_client(SetMode, 'mavros/set_mode')

        # Timer for main loop (will be started when thread runs)
        self.timer = None

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
    
    def pos_local_callback(self, msg):
        self.data_struct.update_local_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

    def vel_callback(self, msg):
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

class SingleDroneRosThread:
    def __init__(self, ui):
        super().__init__()
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
        self._yaw_align = False

        # position/yaw plot data buffers
        self._plot_t0_pos = None
        self._plot_t_pos = deque()
        self._plot_x = deque()
        self._plot_y = deque()
        self._plot_z = deque()
        self._plot_yaw = deque()
        self._plot_x_cmd = deque()
        self._plot_y_cmd = deque()
        self._plot_z_cmd = deque()
        self._plot_yaw_cmd = deque()
        # last setpoint actually sent via send_coordinates(); held constant
        # between sends so the dashed command lines stay flat until updated
        self._last_x_cmd = 0.0
        self._last_y_cmd = 0.0
        self._last_z_cmd = 0.0
        self._last_yaw_cmd = 0.0
        self._setup_position_plot()
        self._setup_motor_display()

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
        self.ui.list_cmd_log.addItem(formatted_message)
        self.ui.list_cmd_log.scrollToBottom()

    def _setup_motor_display(self):
        container = self.ui.display_quad_rotor_NT
        self._motor_display = QuadrotorThrottleWidget(container)
        self._motor_display.setGeometry(container.rect())
        self._motor_display.set_commands((0.0, 0.0, 0.0, 0.0))
        self._motor_display.show()

    # --- Real-time X/Y/Z position + yaw plot -------------------------------------
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
        # (display_x_y_z_yaw in single_drone_flight.ui).
        container = self.ui.display_x_y_z_yaw
        self._pos_plot = pg.PlotWidget(parent=container)
        self._pos_plot.move(0, 0)
        self._pos_plot.setFixedSize(container.width(), container.height())
        self._pos_plot.show()

        self._pos_plot.setBackground('w')
        self._pos_plot.setLabel('bottom', 'Time', units='s')
        self._pos_plot.setLabel('left', 'Position', units='m')
        self._pos_plot.showAxis('right')
        self._pos_plot.setLabel('right', 'Yaw (deg)', color='#AA00AA')
        self._pos_plot.addLegend(offset=(5, -5))
        # Push plot content down so the top margin isn't clipped by the container edge
        self._pos_plot.getPlotItem().layout.setContentsMargins(0, 15, 0, 0)

        # Left axis - X/Y/Z position (m), solid = actual, dashed = commanded
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

        # Right axis - yaw (deg), separate ViewBox; solid = actual, dashed = commanded
        self._vb_yaw = pg.ViewBox()
        self._vb_yaw.invertY(False)  # standalone ViewBox defaults to image coords (Y-down)
        self._pos_plot.scene().addItem(self._vb_yaw)
        self._pos_plot.getAxis('right').linkToView(self._vb_yaw)
        self._vb_yaw.setXLink(self._pos_plot)
        self._curve_yaw = pg.PlotDataItem(pen=pg.mkPen('#AA00AA', width=2), name='Yaw (deg)')
        self._curve_yaw_cmd = pg.PlotDataItem(
            pen=pg.mkPen('#AA00AA', width=2, style=dashed), name='Yaw cmd (deg)')
        self._vb_yaw.addItem(self._curve_yaw)
        self._vb_yaw.addItem(self._curve_yaw_cmd)
        self._vb_yaw.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)
        self._vb_yaw.setYRange(-180, 180, padding=0)
        self._pos_plot.getViewBox().sigResized.connect(self._sync_position_plot_views)

        # Initialise the yaw ViewBox geometry after the event loop starts
        QTimer.singleShot(0, self._sync_position_plot_views)

    def _sync_position_plot_views(self):
        self._vb_yaw.setGeometry(self._pos_plot.getViewBox().sceneBoundingRect())
        self._vb_yaw.linkedViewChanged(self._pos_plot.getViewBox(), self._vb_yaw.XAxis)

    def _append_position_plot(self):
        if not _HAS_PYQTGRAPH or not hasattr(self, '_curve_x'):
            return
        now = time.monotonic()
        if self._plot_t0_pos is None:
            self._plot_t0_pos = now
        t = now - self._plot_t0_pos

        # current_imu.yaw is wrapped to [0, 360) (see Common.quat_to_euler);
        # the plot's yaw axis is fixed to [-180, 180], so re-wrap to match.
        yaw = self.imu_msg.yaw
        if yaw > 180:
            yaw -= 360

        # Commanded X/Y/Z/Yaw: held constant at the last value actually sent
        # via send_coordinates() (see self._last_*_cmd), not the live text field.
        self._plot_t_pos.append(t)
        self._plot_x.append(self.local_pos_msg.x)
        self._plot_y.append(self.local_pos_msg.y)
        self._plot_z.append(self.local_pos_msg.z)
        self._plot_yaw.append(yaw)
        self._plot_x_cmd.append(self._last_x_cmd)
        self._plot_y_cmd.append(self._last_y_cmd)
        self._plot_z_cmd.append(self._last_z_cmd)
        self._plot_yaw_cmd.append(self._last_yaw_cmd)

        # Trim samples outside the history window
        cutoff = t - POSITION_PLOT_HISTORY_S
        while self._plot_t_pos and self._plot_t_pos[0] < cutoff:
            self._plot_t_pos.popleft()
            self._plot_x.popleft()
            self._plot_y.popleft()
            self._plot_z.popleft()
            self._plot_yaw.popleft()
            self._plot_x_cmd.popleft()
            self._plot_y_cmd.popleft()
            self._plot_z_cmd.popleft()
            self._plot_yaw_cmd.popleft()

        t_list = list(self._plot_t_pos)
        self._curve_x.setData(t_list, list(self._plot_x))
        self._curve_y.setData(t_list, list(self._plot_y))
        self._curve_z.setData(t_list, list(self._plot_z))
        self._curve_yaw.setData(t_list, list(self._plot_yaw))
        self._curve_x_cmd.setData(t_list, list(self._plot_x_cmd))
        self._curve_y_cmd.setData(t_list, list(self._plot_y_cmd))
        self._curve_z_cmd.setData(t_list, list(self._plot_z_cmd))
        self._curve_yaw_cmd.setData(t_list, list(self._plot_yaw_cmd))
        self._pos_plot.setXRange(max(0.0, t - POSITION_PLOT_HISTORY_S), t, padding=0)

    # define the signal-slot combination of ros and pyqt GUI
    def set_ros_callbacks(self):
        # feedbacks from ros
        self.ros_object.connect_update_gui(self.update_gui_data)

        # callbacks from GUI
        self.ui.SendPositionUAV.clicked.connect(self.send_coordinates)
        self.ui.GetCurrentPositionUAV.clicked.connect(self.get_coordinates)

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
        self.lock.unlock()

        self.ui.label_vehicle_type.setText(f"Vehicle Type: {vehicle_name}")
        controller_type_display = controller_type or "Unknown"
        self.ui.label_controller_type.setText(f"Controller: {controller_type_display}")
        direct_actuation = controller_type == "Direct Actuation"

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

        connected = bool(state_msg and state_msg.connected)
        displayed_motor_commands = (
            motor_commands
            if direct_actuation and connected
            else (0.0, 0.0, 0.0, 0.0)
        )
        self._motor_display.set_commands(displayed_motor_commands)

        # local position data
        self.ui.RelX_DISP.display("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.RelY_DISP.display("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.AGL_DISP.display("{:.2f}".format(self.local_pos_msg.z, 2))

        self._append_position_plot()

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

        self.ros_object.publish_coordinates(x, y, z, yaw)
        self.log_message(f"Position command sent: {x}, {y}, {z}, {yaw}")

        # Update the dashed command lines in the X/Y/Z/Yaw plot to the setpoint
        # just sent. Wrap yaw to [-180, 180] to match the plot's fixed yaw axis.
        self._last_x_cmd = x
        self._last_y_cmd = y
        self._last_z_cmd = z
        yaw_wrapped = yaw
        if yaw_wrapped > 180:
            yaw_wrapped -= 360
        elif yaw_wrapped < -180:
            yaw_wrapped += 360
        self._last_yaw_cmd = yaw_wrapped

    def get_coordinates(self):
        # get current relative position
        self.ui.XPositionUAV.setText("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.YPositionUAV.setText("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.ZPositionUAV.setText("{:.2f}".format(self.local_pos_msg.z, 2))
        self.ui.YAWUAV.setText("{:.2f}".format(self.imu_msg.yaw, 2))
