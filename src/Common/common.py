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
#!/usr/bin/env python
from PyQt5.QtCore import QMutex
from px4_msgs.msg import VehicleStatus

import ROS_Node.ros_common as ros_common
import numpy as np

class CommonData(): # store the data from the ROS nodes
    def __init__(self):
        self.msg = ""
        self.current_Time = ""
        
        self.current_distance = ""
        self.total_distance = 0

        self.current_imu = ros_common.IMUinfo()
        self.current_global_pos = ros_common.GlobalPositionInfo()
        self.current_local_pos = ros_common.Vector3()
        self.current_vel = ros_common.Vector3()
        self.current_battery_status = ros_common.BatteryInfo()
        self.current_state = ros_common.StateInfo()
        self.current_attitude_target = ros_common.AttitudeTarget()
        self.indoor_mode = False

        # water sampling
        self.encoder_raw = ros_common.Vector3()
        self.payload_pos = ros_common.Vector3()

        self.lock = QMutex()

    def update_imu(self, x, y, z, w):
        euler = self.quat_to_euler(x, y, z, w)
    
        if not self.lock.tryLock():
            return
        self.current_imu.roll = euler[0]
        self.current_imu.pitch = euler[1]
        self.current_imu.yaw = euler[2]
        self.lock.unlock()
        return
  
    def update_global_pos(self, latitude, longitude, altitude):
        if not self.lock.tryLock():
            return
        self.current_global_pos.latitude = latitude
        self.current_global_pos.longitude = longitude
        self.current_global_pos.altitude = altitude
        self.lock.unlock()
        return

    def update_local_pos(self, x, y, z):
        if not self.lock.tryLock():
            return
        self.current_local_pos.x = x
        self.current_local_pos.y = y
        self.current_local_pos.z = z
        self.lock.unlock()
        return
    
    def update_vel(self, vx, vy, vz):
        if not self.lock.tryLock():
            return
        self.current_vel.x = vx
        self.current_vel.y = vy
        self.current_vel.z = vz
        self.lock.unlock()
        return
    
    def update_bat(self, percentage, voltage):
        if not self.lock.tryLock():
            return
        self.current_battery_status.percentage = percentage
        self.current_battery_status.voltage = voltage
        self.lock.unlock()
        return
    

    def update_state(self, pre_flight_checks_pass, arming_state, nav_state, timestamp):

        if not self.lock.tryLock():
            return
        
        if arming_state == VehicleStatus.ARMING_STATE_ARMED:
            armed = True
        elif arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            armed = False
        else:
            armed = None

        self.current_state.connected = pre_flight_checks_pass

        self.current_state.armed = armed
        
        self.current_state.mode = self.decode_mode(nav_state)
        self.current_state.seconds = int(timestamp*1e-6)
        self.lock.unlock()
        return
    
    def update_attitude_target(self, x, y, z, w, thrust):
        euler = self.quat_to_euler(x, y, z, w)
        if not self.lock.tryLock():
            return
        self.current_attitude_target.roll = euler[0]
        self.current_attitude_target.pitch = euler[1]
        self.current_attitude_target.yaw = euler[2]
        self.current_attitude_target.thrust = -thrust
        self.current_attitude_target.roll_rate = 0
        self.current_attitude_target.pitch_rate = 0
        self.current_attitude_target.yaw_rate = 0
        self.current_attitude_target.mode = 1 # set to the mode with attitude control
        self.lock.unlock()
        return
    
    def update_body_rate_target(self, roll, pitch, yaw, thrust):
        if not self.lock.tryLock():
            return
        self.current_attitude_target.roll = 0
        self.current_attitude_target.pitch = 0
        self.current_attitude_target.yaw = 0
        self.current_attitude_target.thrust = -thrust
        self.current_attitude_target.roll_rate = roll
        self.current_attitude_target.pitch_rate = pitch
        self.current_attitude_target.yaw_rate = yaw
        self.current_attitude_target.mode = 2 # set to the mode with body rate control

        self.lock.unlock()
        return
    
    def quat_to_euler(self, x, y, z, w):
        # NED to ENU conversion
        quat_enu = self.ned_to_enu(x, y, z, w)
        nrm = abs(sum(q**2 for q in quat_enu) - 1.0)
        quat = [0, 0, 0, 1] if nrm > 1e-5 else quat_enu

        # Manual quaternion to Euler conversion (XYZ order)
        # quat is [x, y, z, w] format
        qx, qy, qz, qw = quat[0], quat[1], quat[2], quat[3]

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        euler = np.array([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])

        # convert to 360 coordinates
        if euler[2] < 0:
            euler[2] = euler[2] + 360

        return euler
    
    def ned_to_enu(self, x, y, z, w):
        # Manual quaternion operations without scipy
        q = [w, x, y, z]  # scalar first format

        # ---- Fixed transforms
        # ENU -> NED
        Rie = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

        # FLU -> FRD
        Rbv = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])

        # Convert rotation matrices to quaternions
        q_rie = self._rotation_matrix_to_quat(Rie)
        q_rbv = self._rotation_matrix_to_quat(Rbv)

        # Quaternion multiplication: q_rie * q * q_rbv
        q_temp = self._quat_multiply(q_rie, q)
        q_result = self._quat_multiply(q_temp, q_rbv)

        # Return as [x, y, z, w] format (standard format)
        return [q_result[1], q_result[2], q_result[3], q_result[0]]

    def _rotation_matrix_to_quat(self, R):
        """Convert rotation matrix to quaternion [w, x, y, z]"""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return [w, x, y, z]

    def _quat_multiply(self, q1, q2):
        """Multiply two quaternions [w, x, y, z]"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return [w, x, y, z]


    def update_estimator_type(self, indoor_mode):
        if not self.lock.tryLock():
            return
        self.indoor_mode = indoor_mode
        self.lock.unlock()
        return
    
    ## water sampling tab

    def update_encoder_raw(self, x, y, z):
        if not self.lock.tryLock():
            return
        self.encoder_raw.x = x
        self.encoder_raw.y = y
        self.encoder_raw.z = z
        self.lock.unlock()
        return
    
    def update_payload_pos(self, x, y, z):
        if not self.lock.tryLock():
            return
        self.payload_pos.x = x
        self.payload_pos.y = y
        self.payload_pos.z = z
        self.lock.unlock()
        return
        
    
    def decode_mode(self, mode):
        mode_dict = {
            0: 'STATE_MANUAL',
            1: 'STATE_ALTCTL',
            2: 'STATE_POSCTL',
            3: 'STATE_AUTO_MISSION',
            4: 'STATE_AUTO_LOITER',
            5: 'STATE_AUTO_RTL',
            6: 'STATE_POSITION_SLOW',
            7: 'STATE_FREE5',
            8: 'STATE_FREE4',
            9: 'STATE_FREE3',
            10: 'STATE_ACRO',
            11: 'STATE_FREE2',
            12: 'STATE_DESCEND',
            13: 'STATE_TERMINATION',
            14: 'STATE_OFFBOARD',
            15: 'STATE_STAB',
            16: 'STATE_FREE1',
            17: 'STATE_AUTO_TAKEOFF',
            18: 'STATE_AUTO_LAND',
            19: 'STATE_AUTO_FOLLOW_TARGET',
            20: 'STATE_AUTO_PRECLAND',
            21: 'STATE_ORBIT',
            22: 'STATE_AUTO_VTOL_TAKEOFF',
            23: 'STATE_EXTERNAL1',
            24: 'STATE_EXTERNAL2',
            25: 'STATE_EXTERNAL3',
            26: 'STATE_EXTERNAL4',
            27: 'STATE_EXTERNAL5',
            28: 'STATE_EXTERNAL6',
            29: 'STATE_EXTERNAL7',
            30: 'STATE_EXTERNAL8',
            31: 'STATE_MAX',
        }

        return mode_dict.get(mode, 'UNKNOWN_MODE')
