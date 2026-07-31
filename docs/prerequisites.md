# Prerequisites

Everything needed to run the GUI entry points in this repository
(`src/GroundControl.py` and `src/single_drone_ground_control.py`).

## 1. System / ROS 2

| Requirement | Notes |
|---|---|
| Ubuntu 22.04 | Matches the ROS 2 distro below |
| ROS 2 Humble (`ros-humble-desktop` recommended) | Source it (`source /opt/ros/humble/setup.bash`) before running the GUI |
| Python 3.10 | Ships with Ubuntu 22.04 / ROS 2 Humble |
| `MicroXRCEAgent` | Bridges PX4 (SITL or hardware) to ROS 2 over uXRCE-DDS. Not launched by this GUI, but required upstream for any live telemetry to reach it — see `fsc_autopilot_ros2`'s setup docs. |

## 2. Python packages (pip / apt)

| Package | Install | Required by |
|---|---|---|
| PyQt5 | `sudo apt install python3-pyqt5` | GUI framework — both entry points |
| NumPy | `sudo apt install python3-numpy` (or `pip install numpy`) | `Common/common.py` — quaternion/frame conversions |
| pyqtgraph | `pip install pyqtgraph` | **Optional.** Powers the real-time plots in `single_drone_ground_control.py` (`display_x_y_z` and `display_body_angle`). Import is guarded — the GUI runs without it, printing `[PLOT] pyqtgraph not found` and leaving those plots blank. |

## 3. ROS 2 message packages

Installed with a standard `ros-humble-desktop` setup, no extra action needed:

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `std_srvs`
- `visualization_msgs`

## 4. Workspace sibling packages (built with `colcon`, not pip-installable)

Per [this workspace's layout convention](../../fsc_autopilot_ros2/CLAUDE.md), the following
must be checked out and built as sibling packages under this same `fsc_autopilot_ws/src/`
directory — this GUI is never run against copies living anywhere else:

| Package | Source | Purpose |
|---|---|---|
| `px4_msgs` | [PX4/px4_msgs](https://github.com/PX4/px4_msgs), pinned at `release/1.16` | PX4 message definitions (`VehicleStatus`, `VehicleAttitude`, `VehicleGlobalPosition`, `BatteryStatus`, `VehicleRatesSetpoint`, `VehicleAttitudeSetpoint`) |
| `fsc_autopilot_ros2_msgs` | [FSC-Lab/fsc_autopilot_ros2_msgs](https://github.com/FSC-Lab/fsc_autopilot_ros2_msgs) | Custom controller messages (`PositionControllerReference`, `Mocap`, etc.) |
| `fsc_autopilot_ros2` | [FSC-Lab/fsc_autopilot_ros2](https://github.com/FSC-Lab/fsc_autopilot_ros2) | The flight-control node this GUI commands and monitors. Not a Python import dependency of this repo, but must be running (in sim or on hardware) for the GUI to show live data. |
| `fsc_drone_state_estimator_ros2` | [FSC-Lab/fsc_drone_state_estimator_ros2](https://github.com/FSC-Lab/fsc_drone_state_estimator_ros2) | Publishes `/uav_<id>/state_estimator/local_position/odom`, the local-position source both GUIs subscribe to. Indoor (OptiTrack) setups only need `fsc_optitrack_processor_ros2` in addition; see `fsc_autopilot_ros2/docs/required_repos.md` for the sim-vs-real breakdown. |

Build order: `px4_msgs` → `fsc_autopilot_ros2_msgs` → the rest, then this repository (it has no
`package.xml`/`setup.py` of its own — it is not a `colcon` package, just a plain Python
directory run directly, per `AGENTS.md`).

## 5. Not required for either active entry point

- `rospy` (ROS 1) — only imported by `src/ROS_Node/ros_water_sample_control.py`, a legacy
  module that neither `GroundControl.py` nor `single_drone_ground_control.py` starts.
