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
# from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandLong, SetMode
from px4_msgs.msg import VehicleStatus,VehicleAttitudeSetpoint,VehicleAttitude, VehicleGlobalPosition, BatteryStatus,VehicleRatesSetpoint
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference

# from mavros_msgs.msg import State, AttitudeTarget
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import json
# from fsc_autopilot_msgs.msg import TrackingReference
from std_msgs.msg import Bool
import functools

class MultiDroneRosNode(Node, QObject):
    ## define signals
    update_data = pyqtSignal(int)

    def __init__(self, drone_ids=[0]):
        Node.__init__(self, 'multi_drone_gui_node')
        QObject.__init__(self)

        # store list of drone ids (ints)
        self.drone_ids = list(drone_ids)

        # per-drone data structs
        self.data_structs = {i: Common.CommonData() for i in self.drone_ids}
        
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
        
        # per-drone pubs/subs containers
        self.position_com_pubs = {}
        self.geofence_pubs = {}
        self.drone_subscriptions = {}

        # create subscriptions and publishers for each drone id
        for i in self.drone_ids:
            self.drone_subscriptions.setdefault(i, {})
            self.drone_subscriptions[i]['imu'] = self.create_subscription(
                VehicleAttitude,
                f'/uav_{i}/fmu/out/vehicle_attitude',
                functools.partial(self.imu_callback, i),
                self.px4_qos_profile
            )
            self.drone_subscriptions[i]['pos_global'] = self.create_subscription(
                VehicleGlobalPosition,
                f'/uav_{i}/fmu/out/vehicle_global_position',
                functools.partial(self.pos_global_callback, i),
                self.px4_qos_profile
            )
            self.drone_subscriptions[i]['pos_local'] = self.create_subscription(
                Odometry,
                f'/uav_{i}/state_estimator/local_position/odom',
                functools.partial(self.pos_local_callback, i),
                10
            )
            self.drone_subscriptions[i]['vel'] = self.create_subscription(
                Odometry,
                f'/uav_{i}/state_estimator/local_position/odom',
                functools.partial(self.vel_callback, i),
                10
            )
            self.drone_subscriptions[i]['bat'] = self.create_subscription(
                BatteryStatus,
                f'/uav_{i}/fmu/out/battery_status',
                functools.partial(self.bat_callback, i),
                self.px4_qos_profile
            )
            self.drone_subscriptions[i]['status'] = self.create_subscription(
                VehicleStatus,
                f'/uav_{i}/fmu/out/vehicle_status_v1',
                functools.partial(self.status_callback, i),
                self.px4_qos_profile
            )
            self.drone_subscriptions[i]['cmd_att'] = self.create_subscription(
                VehicleAttitudeSetpoint,
                f'/uav_{i}/fsc_autopilot_ros2/attitude_setpoint_debug',
                functools.partial(self.commanded_attitude_callback, i),
                self.px4_input_qos_profile
            )
            self.drone_subscriptions[i]['cmd_rate'] = self.create_subscription(
                VehicleRatesSetpoint,
                f'/uav_{i}/fsc_autopilot_ros2/rate_setpoint_debug',
                functools.partial(self.commanded_bodyrate_callback, i),
                self.px4_input_qos_profile
            )

            # publishers
            self.position_com_pubs[i] = self.create_publisher(
                PositionControllerReference,
                f'/uav_{i}/fsc_autopilot_ros2/position_controller/reference',
                10
            )
            self.geofence_pubs[i] = self.create_publisher(Marker, f'/uav_{i}/tracking_controller/geofence', 10)

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

    ### define callback functions from ros topics ###
    def imu_callback(self, drone_id, msg):
        # get orientation and convert to euler angles
        # note that uses PX4 [w, x, y, z] 
        self.data_structs[drone_id].update_imu(msg.q[1], msg.q[2], msg.q[3], msg.q[0])

    def pos_global_callback(self, drone_id, msg):
        self.data_structs[drone_id].update_global_pos(msg.lat, msg.lon, msg.alt)

    def pos_local_callback(self, drone_id, msg):
        self.data_structs[drone_id].update_local_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

    def vel_callback(self, drone_id, msg):
        self.data_structs[drone_id].update_vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)

    def bat_callback(self, drone_id, msg):
        self.data_structs[drone_id].update_bat(msg.remaining, msg.voltage_v)

    def status_callback(self, drone_id, msg):
        self.data_structs[drone_id].update_state(msg.pre_flight_checks_pass, msg.arming_state, msg.nav_state, (msg.timestamp - msg.armed_time))

    def commanded_attitude_callback(self, drone_id, msg):
        # the attitude setpoint received from px4 
        self.data_structs[drone_id].update_attitude_target(msg.q_d[1], msg.q_d[2], msg.q_d[3], msg.q_d[0], msg.thrust_body[2])

    def commanded_bodyrate_callback(self, drone_id, msg):
        self.data_structs[drone_id].update_body_rate_target(msg.roll, msg.pitch, msg.yaw, msg.thrust_body[2])

    def estimator_type_callback(self, drone_id, msg):
        self.data_structs[drone_id].update_estimator_type(msg.data)

    def publish_coordinates(self, drone_id, x, y, z, yaw=0.0):
        msg = PositionControllerReference()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "ground"
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw
        msg.yaw_unit = PositionControllerReference.DEGREES
        self.position_com_pubs[drone_id].publish(msg)
        self.get_logger().info(f"Publishing coordinates uav_{drone_id}: {x}, {y}, {z}, {yaw}")


    def publish_geofence(self, drone_id, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "geofence"
        marker.id = drone_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        corners = [(x, y, 0), (x, -y, 0), (-x, -y, 0), (-x, y, 0), (x, y, z), (x, -y, z), (-x, -y, z), (-x, y, z)]
        edges = [
            (corners[0], corners[1]), (corners[1], corners[2]), (corners[2], corners[3]), (corners[3], corners[0]),
            (corners[4], corners[5]), (corners[5], corners[6]), (corners[6], corners[7]), (corners[7], corners[4]),
            (corners[0], corners[4]), (corners[1], corners[5]), (corners[2], corners[6]), (corners[3], corners[7])
        ]
        for start, end in edges:
            p0 = Point(); p0.x, p0.y, p0.z = start
            p1 = Point(); p1.x, p1.y, p1.z = end
            marker.points.append(p0); marker.points.append(p1)

        self.geofence_pubs[drone_id].publish(marker)

    # timer emits update for every drone
    def timer_callback(self):
        for i in self.drone_ids:
            self.update_data.emit(i)
    
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

class MultiDroneRosThread:
    def __init__(self, ui, drone_ids=[0]):
        super().__init__()
        self.drone_ids = list(drone_ids)
        self.ros_object = MultiDroneRosNode(drone_ids=self.drone_ids)
        self.thread = QThread()

        # setup signals
        self.ui = ui
        self.set_ros_callbacks()

        # Move ROS node to thread and start
        self.ros_object.moveToThread(self.thread)
        self.thread.started.connect(self.ros_object.run)

        # set geofence displays if widget present (suffix naming: Geofence_X_UAV{n})
        for i in self.drone_ids:
            try:
                getattr(self.ui, f"Geofence_X_UAV{i}").display(self.ros_object.config[0])
                getattr(self.ui, f"Geofence_Y_UAV{i}").display(self.ros_object.config[1])
                getattr(self.ui, f"Geofence_Z_UAV{i}").display(self.ros_object.config[2])
            except AttributeError:
                pass

    def start(self):
        self.thread.start()
    
    def log_message(self, message):
        """Add timestamped message to GUI logging widget"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        formatted_message = f"[{timestamp}] {message}"
        self.ui.Logging.append(formatted_message)

    # define the signal-slot combination of ros and pyqt GUI
    def set_ros_callbacks(self):
        # feedbacks from ros
        self.ros_object.connect_update_gui(self.update_gui_data)

        # callbacks from GUI
        self.ui.SetHome.clicked.connect(self.send_set_home_request)
        self.ui.SimulationMode.stateChanged.connect(self.toggle_simulation_mode)
        
        for i in self.drone_ids:
            send_btn = getattr(self.ui, f"SendPositionUAV_UAV{i}", None)
            if send_btn:
                send_btn.clicked.connect(functools.partial(self.send_coordinates, i))
            get_btn = getattr(self.ui, f"GetCurrentPositionUAV_UAV{i}", None)
            if get_btn:
                get_btn.clicked.connect(functools.partial(self.get_coordinates, i))
        # self.ui.SendPositionUAV.clicked.connect(self.send_coordinates)       
        # self.ui.GetCurrentPositionUAV.clicked.connect(self.get_coordinates)

        self.ui.ARM.clicked.connect(lambda: self.send_arming_request(True, 0))
        self.ui.DISARM.clicked.connect(lambda: self.send_arming_request(False, 0))
        self.ui.Takeoff.clicked.connect(lambda: self.send_takeoff_request(float(self.ui.TakeoffHeight.text())))
        self.ui.Land.clicked.connect(lambda: self.send_land_request())
        self.ui.EmergencyStop.clicked.connect(lambda: self.send_arming_request(False, 21196))

        self.ui.OFFBOARD.clicked.connect(lambda: self.switch_mode("OFFBOARD"))
        self.ui.POSCTL.clicked.connect(lambda: self.switch_mode("POSCTL"))
        self.ui.HOLD.clicked.connect(self.hold)

    # update GUI data
    def update_gui_data(self, drone_id):
        ds = self.ros_object.data_structs.get(drone_id)
        if ds is None:
            return
        lock = ds.lock
        if not lock.tryLock():
            print("MultiDroneRosThread: lock failed")
            return

        # store to local variables for fast lock release
        imu_msg = ds.current_imu
        global_pos_msg = ds.current_global_pos
        local_pos_msg = ds.current_local_pos
        vel_msg = ds.current_vel
        bat_msg = ds.current_battery_status
        state_msg = ds.current_state
        alttitude_targ_msg = ds.current_attitude_target
        indoor_mode = ds.indoor_mode
        lock.unlock()

        # helper to get widget by name with suffix
        def W(name):
            return getattr(self.ui, f"{name}_UAV{drone_id}", None)

        # accelerometer data
        if W("X_DISP"):
            W("X_DISP").display("{:.2f}".format(imu_msg.roll))
        if W("Y_DISP"):
            W("Y_DISP").display("{:.2f}".format(imu_msg.pitch))
        if W("Z_DISP"):
            W("Z_DISP").display("{:.2f}".format(imu_msg.yaw))

        if W("TargROLL_DISP"):
            W("TargROLL_DISP").display("{:.2f}".format(alttitude_targ_msg.roll))
        if W("TargPITCH_DISP"):
            W("TargPITCH_DISP").display("{:.2f}".format(alttitude_targ_msg.pitch))
        if W("TargYAW_DISP"):
            W("TargYAW_DISP").display("{:.2f}".format(alttitude_targ_msg.yaw))
        if W("TargTHRUST_DISP"):
            W("TargTHRUST_DISP").display("{:.2f}".format(alttitude_targ_msg.thrust))

        # global & local position data
        if W("LatGPS_DISP"):
            W("LatGPS_DISP").display("{:.6f}".format(getattr(global_pos_msg, "latitude", 0.0)))
        if W("LongGPS_DISP"):
            W("LongGPS_DISP").display("{:.6f}".format(getattr(global_pos_msg, "longitude", 0.0)))
        if W("AltGPS_DISP"):
            W("AltGPS_DISP").display("{:.2f}".format(getattr(global_pos_msg, "altitude", 0.0)))
        if W("RelX_DISP"):
            W("RelX_DISP").display("{:.2f}".format(getattr(local_pos_msg, "x", 0.0)))
        if W("RelY_DISP"):
            W("RelY_DISP").display("{:.2f}".format(getattr(local_pos_msg, "y", 0.0)))
        if W("AGL_DISP"):
            W("AGL_DISP").display("{:.2f}".format(getattr(local_pos_msg, "z", 0.0)))

        if W("TargROLL_RATE_DISP"):
            W("TargROLL_RATE_DISP").display("{:.2f}".format(getattr(alttitude_targ_msg, "roll_rate", 0.0)))
        if W("TargPITCH_RATE_DISP"):
            W("TargPITCH_RATE_DISP").display("{:.2f}".format(getattr(alttitude_targ_msg, "pitch_rate", 0.0)))
        if W("TargYAW_RATE_DISP"):
            W("TargYAW_RATE_DISP").display("{:.2f}".format(getattr(alttitude_targ_msg, "yaw_rate", 0.0)))

        # velocity data
        if W("U_Vel_DISP"):
            W("U_Vel_DISP").display("{:.2f}".format(getattr(vel_msg, "x", 0.0)))
        if W("V_Vel_DISP"):
            W("V_Vel_DISP").display("{:.2f}".format(getattr(vel_msg, "y", 0.0)))
        if W("W_Vel_DISP"):
            W("W_Vel_DISP").display("{:.2f}".format(getattr(vel_msg, "z", 0.0)))

        # state updates
        if state_msg:
            stARM = W("StateARM")
            stConn = W("StateConnected")
            stMode = W("StateMode")
            if stARM:
                stARM.setText("Armed" if state_msg.armed else "Disarmed")
                stARM.setStyleSheet("color: red" if state_msg.armed else "color: green")
            if stConn:
                stConn.setText("Connected" if state_msg.connected else "Disconnected")
                stConn.setStyleSheet("color: green" if state_msg.connected else "color: red")
            if stMode:
                stMode.setText(state_msg.mode)

        # simulation / indoor
        stSim = W("StateSimulation")
        setHomeBtn = getattr(self.ui, "SetHome", None)
        if indoor_mode:
            if stSim:
                stSim.setText("INDOOR"); stSim.setStyleSheet("color: green")
            if setHomeBtn: setHomeBtn.setEnabled(False)
        else:
            if stSim:
                stSim.setText("OUTDOOR"); stSim.setStyleSheet("color: orange")
            if setHomeBtn: setHomeBtn.setEnabled(True)

        # misc data
        batWidget = W("BatInd")
        volt = W("VOLT_DISP")
        if bat_msg and batWidget:
            if hasattr(batWidget, "setTextVisible") and batWidget.isTextVisible() == False:
                batWidget.setTextVisible(True)
            if hasattr(batWidget, "setValue"):
                batWidget.setValue(int(float(getattr(bat_msg, "percentage", 0.0))*100))
            if volt:
                volt.display("{:.2f}".format(getattr(bat_msg, "voltage", 0.0)))

        # update seconds
        secW = W("Sec_DISP")
        minW = W("Min_DISP")
        if state_msg and secW:
            armed_seconds_attr = f'armed_seconds_{drone_id}'
            last_time_attr = f'last_time_{drone_id}'
            if not hasattr(self, armed_seconds_attr):
                setattr(self, armed_seconds_attr, 0)
            if not hasattr(self, last_time_attr):
                setattr(self, last_time_attr, state_msg.seconds)
            armed_seconds = getattr(self, armed_seconds_attr)
            last_time = getattr(self, last_time_attr)
            if state_msg.armed:
                armed_seconds = state_msg.seconds - last_time
                secW.display("{}".format(armed_seconds))
                setattr(self, armed_seconds_attr, armed_seconds)
            else:
                setattr(self, last_time_attr, state_msg.seconds)
                setattr(self, armed_seconds_attr, 0)
            if armed_seconds >= 60 and minW:
                minW.display("{}".format(int(minW.value() + 1)))
        
         # update the control mode
        if alttitude_targ_msg.mode == 0:
            self.ui.ControlMode.setText("Not Started")
        elif alttitude_targ_msg.mode == 1:
            self.ui.ControlMode.setText("Attitude")
        elif alttitude_targ_msg.mode == 2:
            self.ui.ControlMode.setText("Bodyrate")

    ### callback functions for modifying GUI elements ###
    def toggle_simulation_mode(self, state):
        if state == 2:
            self.log_message("Simulation controls available")
            self.ui.ARM.setEnabled(True)
            self.ui.DISARM.setEnabled(True)
            self.ui.Takeoff.setEnabled(True)
            self.ui.Land.setEnabled(True)
            self.ui.TakeoffHeight.setEnabled(True)
            self.ui.EmergencyStop.setEnabled(True)
            self.ui.OFFBOARD.setEnabled(True)
            self.ui.POSCTL.setEnabled(True)

        else:
            self.log_message("Simulation controls disabled")
            self.ui.ARM.setEnabled(False)
            self.ui.DISARM.setEnabled(False)
            self.ui.Takeoff.setEnabled(False)
            self.ui.Land.setEnabled(False)
            self.ui.TakeoffHeight.setEnabled(False)
            self.ui.EmergencyStop.setEnabled(False)
            self.ui.OFFBOARD.setEnabled(False)
            self.ui.POSCTL.setEnabled(False)
    
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

    def send_coordinates(self, drone_id):
        # read line edits with suffixes; fall back to global names
        def L(name):
            return getattr(self.ui, f"{name}_UAV{drone_id}", getattr(self.ui, name, None))
        try:
            x = float(L("XPositionUAV").text())
            y = float(L("YPositionUAV").text())
            z = float(L("ZPositionUAV").text())
            yaw = float(L("YAWUAV").text())
        except Exception:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Invalid input, make sure values are numbers")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        # bounds check
        if abs(x) > int(self.ros_object.config[0]) or abs(y) > int(self.ros_object.config[1]) or abs(z) > int(self.ros_object.config[2]) or z <= 0:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Position is not within geofence or invalid altitude")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        self.ros_object.publish_coordinates(drone_id, x, y, z, yaw)
        self.log_message(f"Position command sent for uav_{drone_id}: {x}, {y}, {z}, {yaw}")

    # get current relative position
    def get_coordinates(self, drone_id):
        ds = self.ros_object.data_structs.get(drone_id)
        if ds is None:
            return
        lock = ds.lock
        if not lock.tryLock():
            return
        local = ds.current_local_pos
        imu = ds.current_imu
        lock.unlock()

        # set line edits (suffixed)
        def S(name, val):
            w = getattr(self.ui, f"{name}_UAV{drone_id}", getattr(self.ui, name, None))
            if w and hasattr(w, "setText"):
                w.setText("{:.2f}".format(val))

        S("XPositionUAV", getattr(local, "x", 0.0))
        S("YPositionUAV", getattr(local, "y", 0.0))
        S("ZPositionUAV", getattr(local, "z", 0.0))
        S("YAWUAV", getattr(imu, "yaw", 0.0))

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

