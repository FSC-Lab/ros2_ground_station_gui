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

class IMUinfo:
    def __init__(self, x=0, y=0, z=0) -> None:
        self.roll = x
        self.pitch = y
        self.yaw = z

class GlobalPositionInfo:
    def __init__(self, latitude=0, longitude=0, altitude=0) -> None:
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

class Vector3:
    def __init__(self, x=0, y=0, z=0) -> None:
        self.x = x
        self.y = y
        self.z = z

class BatteryInfo:
    def __init__(self, percentage=0, voltage=0) -> None:
        self.percentage = percentage
        self.voltage = voltage

class StateInfo:
    def __init__(self, connected=False, armed=False, mode='', seconds=0) -> None:
        self.connected = connected
        self.armed = armed
        self.mode = mode
        self.seconds = seconds
        self.total_seconds = 0

class AttitudeTarget:
    def __init__(self, roll=0, pitch=0, yaw=0, thrust=0, roll_rate=0, pitch_rate=0, yaw_rate=0, mode = 0) -> None:
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_rate = roll_rate
        self.pitch_rate = pitch_rate
        self.yaw_rate = yaw_rate
        self.thrust = thrust
        self.mode = mode

class ControllerStatus:
    def __init__(self, baseline=True) -> None:
        self.baseline_mode = baseline

        # Actual controller mode from autopilot (ground truth)
        self.actual_controller_mode = None  # "Baseline" or "MPC" or None if not received yet

        # Heartbeat tracking (std_msgs/Bool @ 1-10Hz)
        self.mpc_active = False  # True if MPC fully activated
        self.last_heartbeat_time = 0.0
        self.heartbeat_ever_received = False
        self.heartbeat_timeout = False

        # Solver status tracking (std_msgs/Int32 @ 30-100Hz)
        self.solver_ok = False  # True if solver succeeded (status==0)
        self.last_solver_status_time = 0.0
        self.solver_ever_received = False
        self.solver_timeout = False

        # Solver results tracking (px4_msgs/VehicleRatesSetpoint)
        self.last_solver_res_time = 0.0
        self.mpc_thrust = 0.0
        self.mpc_roll_rate = 0.0
        self.mpc_pitch_rate = 0.0
        self.mpc_yaw_rate = 0.0
