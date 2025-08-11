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
from scipy.spatial.transform import Rotation
from px4_msgs.msg import VehicleStatus

import ROS_Node.ros_common as ros_common


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
        self.current_attitude_target.thrust = thrust
        self.lock.unlock()
        return
    
    def quat_to_euler(self, x, y, z, w):
        # NED to ENU conversion
        quat_enu = [y, x, -z, w]
        nrm = abs(sum(q**2 for q in quat_enu) - 1.0)
        quat = [0, 0, 0, 1] if nrm > 1e-5 else quat_enu
        r = Rotation.from_quat(quat)
        euler = r.as_euler('xyz', degrees=True)

        # convert to 360 coordinates
        if euler[2] < 0:
            euler[2] = euler[2] + 360

        return euler
    
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
