# Rosbag recording for experiment comparison

Use this bag to compare a physical `uav_0` experiment with the equivalent Isaac
Sim run. It records the estimator output used by the ground station, raw PX4
state estimates, the requested position/yaw reference, and the setpoints sent
toward PX4.

Run the following from a terminal in which ROS 2 and the containing workspace
have been sourced:

```bash
ros2 bag record \
  --output experiment_uav_0 \
  /uav_0/state_estimator/local_position/odom \
  /uav_0/fmu/out/vehicle_local_position \
  /uav_0/fmu/out/vehicle_attitude \
  /uav_0/fmu/out/vehicle_angular_velocity \
  /uav_0/fmu/out/vehicle_odometry \
  /uav_0/fsc_autopilot_ros2/position_controller/reference \
  /uav_0/fsc_autopilot_ros2/attitude_setpoint_debug \
  /uav_0/fsc_autopilot_ros2/rate_setpoint_debug \
  /uav_0/fmu/in/trajectory_setpoint \
  /uav_0/fmu/in/vehicle_attitude_setpoint \
  /uav_0/fmu/in/vehicle_rates_setpoint \
  /uav_0/fmu/in/offboard_control_mode
```

Stop recording with `Ctrl+C`. The output directory must not already exist; use
a unique name for each run, for example `experiment_uav_0_isaac_01` and
`experiment_uav_0_flight_01`.

## What the topics contain

| Signal | Topic |
| --- | --- |
| Local pose and velocity used by this GUI | `/uav_0/state_estimator/local_position/odom` |
| Raw PX4 local position and velocity | `/uav_0/fmu/out/vehicle_local_position` |
| Raw PX4 attitude quaternion | `/uav_0/fmu/out/vehicle_attitude` |
| Raw PX4 body angular velocity | `/uav_0/fmu/out/vehicle_angular_velocity` |
| Combined raw PX4 pose, velocity, and frame metadata | `/uav_0/fmu/out/vehicle_odometry` |
| Requested position and yaw | `/uav_0/fsc_autopilot_ros2/position_controller/reference` |
| Attitude/thrust produced by the controller | `/uav_0/fsc_autopilot_ros2/attitude_setpoint_debug` and `/uav_0/fmu/in/vehicle_attitude_setpoint` |
| Body-rate/thrust setpoint produced by the controller | `/uav_0/fsc_autopilot_ros2/rate_setpoint_debug` and `/uav_0/fmu/in/vehicle_rates_setpoint` |
| PX4 trajectory and offboard-mode inputs, when used | `/uav_0/fmu/in/trajectory_setpoint` and `/uav_0/fmu/in/offboard_control_mode` |

The debug setpoint topics are included because the active GUI already consumes
them. The corresponding `fmu/in` topics show what is present at the PX4 ROS
interface. Depending on the selected controller, some input/setpoint topics may
have no publishers; rosbag can still run and will simply record no messages for
those topics.

## Check topic availability before a run

Topic availability can vary with the controller and PX4 message bridge version.
Check the running graph before starting the experiment:

```bash
ros2 topic list | sort
ros2 topic info --verbose /uav_0/fmu/in/vehicle_attitude_setpoint
ros2 topic info --verbose /uav_0/fsc_autopilot_ros2/position_controller/reference
```

After the run, verify that the expected streams were captured:

```bash
ros2 bag info experiment_uav_0
```

## Comparison notes

- Compare using message timestamps rather than bag receive order.
- `/uav_0/state_estimator/local_position/odom` is the most direct measured
  signal to compare with the GUI and normally contains position, attitude,
  linear velocity, and angular velocity in one message.
- PX4 `fmu/out` data commonly uses NED/FRD conventions. The state-estimator
  odometry and Isaac Sim data may use ENU/FLU. Convert both datasets into the
  same coordinate frames before calculating error.
- `PositionControllerReference.yaw` is published in degrees in this project.
  PX4 attitude/rate setpoint fields generally use radians or radians per second;
  confirm the installed message definitions before comparison.
- Use the same reference trajectory, controller configuration, estimator mode,
  and experiment start event in the real and simulated runs.

For another vehicle, replace every `uav_0` occurrence with its namespace, such
as `uav_1`. For a multi-drone experiment, list the same topic set once for each
vehicle namespace in a single `ros2 bag record` command.
