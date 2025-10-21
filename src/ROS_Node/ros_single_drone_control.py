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

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QDateTime
from PyQt5.QtWidgets import QMessageBox
import Common
from geometry_msgs.msg import Point
from std_msgs.msg import String
# from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandLong, SetMode
from px4_msgs.msg import VehicleStatus,VehicleAttitudeSetpoint,VehicleAttitude, VehicleGlobalPosition, BatteryStatus, VehicleRatesSetpoint
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import json
# from fsc_autopilot_msgs.msg import TrackingReference
from std_msgs.msg import Bool, Int32

class SingleDroneRosNode(Node, QObject):
    ## define signals
    update_data = pyqtSignal(int)
    mpc_log_signal = pyqtSignal(str)

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
        
        # Define subscribers
        self.imu_sub = self.create_subscription(VehicleAttitude, '/uav_0/fmu/out/vehicle_attitude', self.imu_callback, self.px4_qos_profile)
        self.pos_global_sub = self.create_subscription(VehicleGlobalPosition, '/uav_0/fmu/out/vehicle_global_position', self.pos_global_callback, self.px4_qos_profile)
        self.pos_local_adjusted_sub = self.create_subscription(Odometry, '/uav_0/state_estimator/local_position/odom', self.pos_local_callback, 10)
        self.vel_sub = self.create_subscription(Odometry, '/uav_0/state_estimator/local_position/odom', self.vel_callback, 10)
        self.bat_sub = self.create_subscription(BatteryStatus, '/uav_0/fmu/out/battery_status', self.bat_callback, self.px4_qos_profile)
        self.status_sub = self.create_subscription(VehicleStatus, '/uav_0/fmu/out/vehicle_status_v1', self.status_callback, self.px4_qos_profile)
        self.commanded_attitude_sub = self.create_subscription(VehicleAttitudeSetpoint, '/uav_0/fmu/in/vehicle_attitude_setpoint', self.commanded_attitude_callback, self.px4_input_qos_profile)
        self.commanded_bodyrate_callback = self.create_subscription(VehicleRatesSetpoint, '/uav_0/fmu/in/vehicle_rates_setpoint', self.commanded_bodyrate_callback, self.px4_input_qos_profile)
        self.estimator_type_sub = self.create_subscription(Bool, '/estimator_type', self.estimator_type_callback, 10)
        self.mpc_heartbeat_sub = self.create_subscription(Bool, '/uav_0/fsc_autopilot_ros2/mpc/heartbeat', self.mpc_heartbeat_callback, 10)
        self.mpc_solver_res_sub = self.create_subscription(VehicleRatesSetpoint, '/uav_0/fsc_autopilot_ros2/mpc/solver_res', self.mpc_solver_res_callback, 10)
        self.mpc_solver_status_sub = self.create_subscription(Int32, '/uav_0/fsc_autopilot_ros2/mpc/solver_status', self.mpc_solver_status_callback, 10)
        self.controller_mode_sub = self.create_subscription(String, '/uav_0/fsc_autopilot_ros2/controller_mode', self.controller_mode_callback, 10)
        # Define publishers / services
        # self.coords_pub = self.create_publisher(TrackingReference, 'position_controller/target', 10)
        self.geofence_pub = self.create_publisher(Marker, 'tracking_controller/geofence', 10)
        self.controller_mode_pub = self.create_publisher(String, '/uav_0/fsc_autopilot_ros2/controller_mode_cmd', 10)
        
        self.position_com_pub = self.create_publisher(
            PositionControllerReference, '/uav_0/fsc_autopilot_ros2/position_controller/reference', 10)
        self.set_home_override_service = self.create_client(Empty, 'state_estimator/override_set_home')
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
    
    def connect_mpc_log(self, callback):
        self.mpc_log_signal.connect(callback)

    ### define callback functions from ros topics ###
    def imu_callback(self, msg): 
        # get orientation and convert to euler angles
        # note that uses PX4 [w, x, y, z]
        # IMU from px4 is in NED frame, need to convert to ENU frame 
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

    def mpc_heartbeat_callback(self, msg):
        """Callback for MPC heartbeat (std_msgs/Bool)

        msg.data = True if MPC is fully activated, False otherwise
        """
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Update MPC heartbeat status
        self.data_struct.update_mpc_heartbeat(
            msg.data,  # True if fully activated
            current_time
        )

        # Log heartbeat status
        if msg.data:
            self.get_logger().debug("MPC Heartbeat: ACTIVE")
        else:
            self.get_logger().debug("MPC Heartbeat: INACTIVE")

    def mpc_solver_res_callback(self, msg):
        """Callback for MPC solver results (px4_msgs/VehicleRatesSetpoint)

        Contains the body rate commands and thrust from MPC
        """
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Extract thrust (convert from NED [-1,0] to normalized [0,1])
        thrust = -msg.thrust_body[2]

        # Store MPC commands with timestamp
        self.data_struct.update_mpc_commands(
            thrust,
            msg.roll,
            msg.pitch,
            msg.yaw,
            current_time
        )

        # Log solver results for MPC monitoring
        self.get_logger().debug(
            f"MPC Commands - Thrust: {thrust:.3f}, "
            f"Rates: [{msg.roll:.3f}, {msg.pitch:.3f}, {msg.yaw:.3f}]"
        )

        # Emit signal for GUI MPC logging
        log_message = (
            f"MPC Commands | Thrust: {thrust:.3f} | "
            f"Rates: [{msg.roll:.3f}, {msg.pitch:.3f}, {msg.yaw:.3f}]"
        )
        self.mpc_log_signal.emit(log_message)

    def mpc_solver_status_callback(self, msg):
        """Callback for MPC solver status (std_msgs/Int32)

        msg.data = 0 if solver succeeded, 1 if failed
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        solver_ok = (msg.data == 0)

        # Update solver status with timestamp
        self.data_struct.update_mpc_solver_status(solver_ok, current_time)

        # Log solver status
        if not solver_ok:
            self.get_logger().warn("MPC Solver FAILED - infeasible solution")
            self.mpc_log_signal.emit("MPC Solver: FAILED (infeasible)")
        else:
            self.get_logger().debug("MPC Solver: OK")
            self.mpc_log_signal.emit("MPC Solver: OK")

    def controller_mode_callback(self, msg):
        """Callback for actual controller mode from autopilot (std_msgs/String)

        msg.data = "Baseline" or "MPC"
        This provides ground truth of which controller is actually active in the autopilot.
        """
        actual_mode = msg.data
        self.data_struct.update_controller_mode(actual_mode)

        # Detect mode mismatch (GUI thinks one thing, autopilot doing another)
        expected_baseline = self.data_struct.controller_status.baseline_mode
        actual_baseline = (actual_mode == "baseline")

        if expected_baseline != actual_baseline:
            self.get_logger().warn(
                f"Controller mode mismatch! GUI expected: {'Baseline' if expected_baseline else 'MPC'}, "
                f"Autopilot actual: {actual_mode}"
            )
            # Sync GUI state to match reality
            self.data_struct.controller_status.baseline_mode = actual_baseline
            self.get_logger().info(f"GUI state synchronized to match autopilot: {actual_mode}")

    ### define publish functions to ros topics ###
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

        self.get_logger().info("Geofence published")
        self.geofence_pub.publish(marker)

    def publish_controller_mode(self, is_baseline):
        msg = String()
        if is_baseline:
            msg.data = "Baseline"
        else:
            msg.data = "MPC"
        self.controller_mode_pub.publish(msg)

    # Timer callback for main loop
    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Check for heartbeat timeout (MPC sends heartbeat at configured rate, timeout after 3 seconds)
        heartbeat_timeout = self.data_struct.check_mpc_heartbeat_timeout(current_time)

        if heartbeat_timeout:
            self.get_logger().warn("MPC heartbeat timeout detected")
            self.mpc_log_signal.emit("MPC node stopped - heartbeat timeout detected")
            # Auto-switch to baseline if heartbeat times out
            if not self.data_struct.controller_status.baseline_mode:
                self.data_struct.controller_status.baseline_mode = True
                self.get_logger().warn("Auto-switching to BASELINE due to heartbeat timeout")

        # Check for solver status timeout (MPC solver runs at control loop rate, timeout after 1 second)
        solver_timeout = self.data_struct.check_mpc_solver_timeout(current_time)

        if solver_timeout:
            self.get_logger().warn("MPC solver status timeout detected")
            self.mpc_log_signal.emit("MPC solver stopped - no status updates")
            # Auto-switch to baseline if solver times out
            if not self.data_struct.controller_status.baseline_mode:
                self.data_struct.controller_status.baseline_mode = True
                self.get_logger().warn("Auto-switching to BASELINE due to solver timeout")

        self.update_data.emit(0)
        # Continuously publish controller mode
        self.publish_controller_mode(self.data_struct.controller_status.baseline_mode)
    
    # main loop of ros node (for compatibility with thread)
    def run(self):
        # Start the timer when the thread begins
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz
        
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
        self.ui.Logging.append(formatted_message)

    def mpc_log_message(self, message):
        """Add timestamped message to GUI logging widget"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        formatted_message = f"[{timestamp}] {message}"
        self.ui.MpcLogging.append(formatted_message)
    

    # define the signal-slot combination of ros and pyqt GUI
    def set_ros_callbacks(self):
        # feedbacks from ros
        self.ros_object.connect_update_gui(self.update_gui_data)
        self.ros_object.mpc_log_signal.connect(self.mpc_log_message)

        # callbacks from GUI
        self.ui.SetHome.clicked.connect(self.send_set_home_request)
        self.ui.SwitchBaseline.clicked.connect(self.switch_baseline)
        self.ui.SwitchMPC.clicked.connect(self.switch_mpc)
        self.ui.SendPositionUAV.clicked.connect(self.send_coordinates)
        self.ui.GetCurrentPositionUAV.clicked.connect(self.get_coordinates)

        self.ui.EmergencyStop.clicked.connect(lambda: self.send_arming_request(False, 21196))

    # update GUI data
    def update_gui_data(self):
        if not self.lock.tryLock():
            print("SingleDroneRosThread: lock failed")
            return
        # store to local variables for fast lock release
        self.imu_msg = self.ros_object.data_struct.current_imu
        self.global_pos_msg = self.ros_object.data_struct.current_global_pos
        self.local_pos_msg = self.ros_object.data_struct.current_local_pos
        vel_msg = self.ros_object.data_struct.current_vel
        bat_msg = self.ros_object.data_struct.current_battery_status
        state_msg = self.ros_object.data_struct.current_state
        alttitude_targ_msg = self.ros_object.data_struct.current_attitude_target
        indoor_mode = self.ros_object.data_struct.indoor_mode
        controller_status_msg = self.ros_object.data_struct.controller_status

        self.lock.unlock()

        # accelerometer data
        self.ui.X_DISP.display("{:.2f}".format(self.imu_msg.roll, 2))
        self.ui.Y_DISP.display("{:.2f}".format(self.imu_msg.pitch, 2))
        self.ui.Z_DISP.display("{:.2f}".format(self.imu_msg.yaw, 2))

        self.ui.TargROLL_DISP.display("{:.2f}".format(alttitude_targ_msg.roll, 2))
        self.ui.TargPITCH_DISP.display("{:.2f}".format(alttitude_targ_msg.pitch, 2))
        self.ui.TargYAW_DISP.display("{:.2f}".format(alttitude_targ_msg.yaw, 2))
        self.ui.TargTHRUST_DISP.display("{:.2f}".format(alttitude_targ_msg.thrust, 2))

        self.ui.TargROLL_RATE_DISP.display("{:.2f}".format(alttitude_targ_msg.roll_rate, 2))
        self.ui.TargPITCH_RATE_DISP.display("{:.2f}".format(alttitude_targ_msg.pitch_rate, 2))
        self.ui.TargYAW_RATE_DISP.display("{:.2f}".format(alttitude_targ_msg.yaw_rate, 2))

        # global & local position data
        self.ui.LatGPS_DISP.display("{:.2f}".format(self.global_pos_msg.latitude, 2))
        self.ui.LongGPS_DISP.display("{:.2f}".format(self.global_pos_msg.longitude, 2))
        self.ui.AltGPS_DISP.display("{:.2f}".format(self.global_pos_msg.altitude, 2))
        self.ui.RelX_DISP.display("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.RelY_DISP.display("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.AGL_DISP.display("{:.2f}".format(self.local_pos_msg.z, 2))

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

        if indoor_mode:
            self.ui.StateSimulation.setText("INDOOR")
            self.ui.StateSimulation.setStyleSheet("color: green")
            self.ui.SetHome.setEnabled(False)
        else:
            self.ui.StateSimulation.setText("OUTDOOR")
            self.ui.StateSimulation.setStyleSheet("color: orange")
            self.ui.SetHome.setEnabled(True)

        # misc data
        if bat_msg: # takes long to initialize
            if self.ui.BatInd.isTextVisible() == False:
                self.ui.BatInd.setTextVisible(True)
            self.ui.BatInd.setValue(int(float(bat_msg.percentage)*100))
            self.ui.VOLT_DISP.display("{:.2f}".format(bat_msg.voltage, 2))

        # update seconds
        if state_msg:
            if not hasattr(self, 'armed_seconds'):
                self.armed_seconds = 0
            if not hasattr(self, 'last_time'):
                self.last_time = state_msg.seconds
            if state_msg.armed:
                self.armed_seconds = state_msg.seconds - self.last_time # time since armed
                self.ui.Sec_DISP.display("{}".format(self.armed_seconds, 1))
            else:
                self.last_time = state_msg.seconds
                self.armed_seconds = 0
            # update minutes
            if self.armed_seconds >= 60:
                self.ui.Min_DISP.display("{}".format(int(self.ui.Min_DISP.value() + 1), 1))
                self.last_time = state_msg.seconds

        # Update Controller Mode display using actual mode from autopilot
        if controller_status_msg.actual_controller_mode is not None:
            # Use ground truth from autopilot
            self.ui.ControllerMode.setText(f"Controller: {controller_status_msg.actual_controller_mode}")
        else:
            # Fallback to GUI state if topic not received yet
            if controller_status_msg.baseline_mode:
                self.ui.ControllerMode.setText("Controller: Baseline (GUI)")
            else:
                self.ui.ControllerMode.setText("Controller: MPC (GUI)")

        # Update Solver Mode display based on MPC status
        if not controller_status_msg.heartbeat_ever_received:
            # No heartbeat ever received - node not started
            self.ui.SolverMode.setText("Not Started")
        elif controller_status_msg.heartbeat_timeout:
            # Heartbeat was received before but now timed out - node stopped
            self.ui.SolverMode.setText("Heartbeat Timeout")
        elif controller_status_msg.solver_timeout:
            # Solver status messages timing out
            self.ui.SolverMode.setText("Solver Timeout")
        elif controller_status_msg.mpc_active:
            # MPC heartbeat says fully active - show solver status
            if not controller_status_msg.solver_ever_received:
                self.ui.SolverMode.setText("Waiting for Solver")
            elif controller_status_msg.solver_ok:
                self.ui.SolverMode.setText("Active & OK")
            else:
                self.ui.SolverMode.setText("Active but FAILED")
        else:
            # Heartbeat received but MPC not fully active
            self.ui.SolverMode.setText("Inactive")

        # update the control mode
        if alttitude_targ_msg.mode == 0:
            self.ui.ControlMode.setText("Not Started")
        elif alttitude_targ_msg.mode == 1:
            self.ui.ControlMode.setText("Attitude")
        elif alttitude_targ_msg.mode == 2:
            self.ui.ControlMode.setText("Bodyrate")

    

    def switch_baseline(self):
        if self.ros_object.data_struct.controller_status.baseline_mode:
            self.log_message("Current controller is baseline, switching failed")
        else:
            self.ros_object.data_struct.controller_status.baseline_mode = True
            self.log_message("Switching controller mode: BASELINE")
            # hold position after switching
            self.hold()
        

    def switch_mpc(self):
        # Check if MPC is available before switching
        controller_status = self.ros_object.data_struct.controller_status

        if not controller_status.heartbeat_ever_received:
            self.log_message("MPC switch failed - MPC node not started")
        elif controller_status.heartbeat_timeout:
            self.log_message("MPC switch failed - MPC node timeout")
        elif not controller_status.mpc_active:
            # MPC not fully activated
            self.log_message("MPC switch failed - MPC not fully activated")
        elif controller_status.baseline_mode is False:
            # Already in MPC mode
            self.log_message("Already in MPC mode")
        else:
            # Switch to MPC mode
            self.ros_object.data_struct.controller_status.baseline_mode = False
            self.log_message("Switching controller mode: MPC")

            # hold position after switching
            self.hold()

            # Log current solver status
            if controller_status.solver_ok:
                self.log_message("MPC activated - solver OK")
            else:
                self.log_message("MPC activated - WARNING: solver FAILED")
                self.mpc_log_message("WARNING: Solver returning failed solutions")

    def send_set_home_request(self):
        if self.ros_object.set_home_override_service.wait_for_service(timeout_sec=1.0):
            request = Empty.Request()
            future = self.ros_object.set_home_override_service.call_async(request)
            self.log_message("Set home override request sent")
        else:
            self.log_message("Set home override service not available")
        
        # home_position = CommandHomeRequest()
        # home_position.latitude = self.global_pos_msg.latitude
        # home_position.longitude = self.global_pos_msg.longitude
        # home_position.altitude = self.global_pos_msg.altitude
        # if self.ros_object.set_home_service.wait_for_service(timeout_sec=1.0):
        #     future = self.ros_object.set_home_service.call_async(home_position)
        #     print("Set home request sent")
        # else:
        #     print("Set home service not available")

    def send_coordinates(self):
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
        # if values are not within 5 meters of current position warn user
        if abs(x) > int(self.ros_object.config[0]) or abs(y) > int(self.ros_object.config[1]) or abs(z) > int(self.ros_object.config[2]) or z <= 0:
            ## pop up dialog 
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Position is not within 5 meters of current position")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        self.ros_object.publish_coordinates(x, y, z, yaw)
        self.log_message(f"Position command sent: {x}, {y}, {z}, {yaw}")

    def get_coordinates(self):
        # get current relative position
        self.ui.XPositionUAV.setText("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.YPositionUAV.setText("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.ZPositionUAV.setText("{:.2f}".format(self.local_pos_msg.z, 2))
        self.ui.YAWUAV.setText("{:.2f}".format(self.imu_msg.yaw, 2))

    ### define publish / service functions to ros topics ###
    def send_arming_request(self, arm, param2):
        # if self.ros_object.arming_service.wait_for_service(timeout_sec=1.0):
        #     request = CommandLong.Request()
        #     request.command = 400
        #     request.confirmation = 0
        #     request.param1 = float(arm)
        #     request.param2 = float(param2)
        #     future = self.ros_object.arming_service.call_async(request)
        #     print(f"Arming request sent: {arm}")
        #     return True
        # else:
        #     print("Arming service not available")
        #     return False
        self.log_message(f"Arming request: {arm}, param2: {param2}")
        return True

    def send_takeoff_request(self, req_altitude):
        arm_response = self.send_arming_request(True, 0)
        # if armed takeoff
        if arm_response.result == 0:
            self.ros_object.publish_coordinates(0, 0, req_altitude)
            self.log_message(f"Takeoff request sent at {req_altitude} meters")

    def send_land_request(self):
        # if self.ros_object.land_service.wait_for_service(timeout_sec=1.0):
        #     request = CommandLong.Request()
        #     request.command = 21
        #     request.confirmation = 0
        #     request.param1 = 0.0
        #     request.param7 = 0.0
        #     future = self.ros_object.land_service.call_async(request)
        #     print("Land request sent")
        # else:
        #     print("Land service not available")
        self.ros_object.publish_coordinates(self.ros_object.data_struct.current_local_pos.x, self.ros_object.data_struct.current_local_pos.y, 0, 0)
        self.log_message("Land request sent")

    def switch_mode(self, mode):
        # if self.ros_object.set_mode_service.wait_for_service(timeout_sec=1.0):
        #     request = SetMode.Request()
        #     request.custom_mode = mode
        #     future = self.ros_object.set_mode_service.call_async(request)
        #     print(f"Mode switch request sent: {mode}")
        # else:
        #     print("Set mode service not available")
        self.log_message(f"Mode switch request: {mode}")

    def hold(self):
        self.ros_object.publish_coordinates(self.ros_object.data_struct.current_local_pos.x, self.ros_object.data_struct.current_local_pos.y, 2, self.ros_object.data_struct.current_imu.yaw)
        self.log_message("Hold position command sent")

