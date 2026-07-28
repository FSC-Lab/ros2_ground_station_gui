# AGENTS.md

## Project overview

This repository contains a PyQt5 ground-control GUI for ROS 2 and PX4-based drone experiments. There are two entry points: `src/GroundControl.py` launches a multi-drone station for UAV IDs `0`, `1`, and `2`; `src/single_drone_ground_control.py` launches a single-UAV station (`uav_0`) built around a dedicated layout for single-drone flight experiments.

The application is not packaged as an installable Python or ROS package. Imports and configuration paths assume the command is run from the repository root with `src` on `PYTHONPATH`.

## Repository map

- `src/GroundControl.py`: multi-drone application entry point; initializes `rclpy`, creates the Qt application, and starts `ros_multi_drone_control.MultiDroneRosThread`.
- `src/single_drone_ground_control.py`: single-drone application entry point; initializes `rclpy`, creates the Qt application from `GUI/single_drone_flight.py`, and starts `ros_single_drone_control.SingleDroneRosThread`.
- `src/ROS_Node/ros_multi_drone_control.py`: active ROS 2 multi-drone node, topics, publishers, Qt signal/slot connections, and GUI updates. Used by `GroundControl.py`.
- `src/ROS_Node/ros_single_drone_control.py`: active ROS 2 single-UAV (`uav_0`) node, topics, publishers, Qt signal/slot connections, and GUI updates. Used only by `single_drone_ground_control.py`.
- `src/ROS_Node/ros_water_sample_control.py`: legacy ROS 1 (`rospy`) water-sampling implementation; currently not exported or started.
- `src/ROS_Node/ros_common.py`: small data-holder classes shared by the controllers.
- `src/Common/common.py`: mutex-protected telemetry/state shared between ROS callbacks and Qt updates; includes frame and quaternion conversions.
- `src/GUI/GUI_Sampler.ui`: Qt Designer source of truth for the multi-drone GUI.
- `src/GUI/GUI_SamplerdotUI.py`: generated PyQt5 code imported by `GroundControl.py`.
- `src/GUI/single_drone_flight.ui`: Qt Designer source of truth for the single-drone GUI (an "Autonomous Flight" telemetry tab plus an "Initialization" tab with arm/disarm/takeoff/land/mode controls).
- `src/GUI/single_drone_flight.py`: generated PyQt5 code imported by `single_drone_ground_control.py`.
- `src/ROS_Node/geofence.json` and `slung_load.json`: runtime configuration.
- `scripts/convert_ui.sh`: converts a `.ui` file to Python with `pyuic5`.
- `docs/prerequisites.md`: ROS 2 Humble, Python, message-package, and sibling-workspace requirements.
- `GUI_SamplerdotUI.py`: legacy generated file at the repository root; the active import uses the copy under `src/GUI`.

## Environment and dependencies

Use a ROS 2 environment that provides the workspace's message packages. Important runtime dependencies include:

- Python 3, PyQt5, and NumPy
- ROS 2 Python (`rclpy`)
- `px4_msgs`
- `fsc_autopilot_ros2_msgs`
- `geometry_msgs`, `nav_msgs`, `std_msgs`, `std_srvs`, and `visualization_msgs`
- `pyqtgraph` (optional): powers the embedded real-time plots in the single-drone GUI (e.g. `display_x_y_z_yaw`). Import is guarded — the GUI runs without it, printing `[PLOT] pyqtgraph not found` and leaving that plot blank.

See `docs/prerequisites.md` for the expected ROS 2 workspace layout and upstream packages. In particular, `px4_msgs` and `fsc_autopilot_ros2_msgs` must be available from the sourced workspace.

Source the ROS installation and the containing workspace before running the GUI. From this repository root, launch the multi-drone station with:

```bash
PYTHONPATH=src python3 src/GroundControl.py
```

or the single-drone station with:

```bash
PYTHONPATH=src python3 src/single_drone_ground_control.py
```

Run from the repository root because the controllers currently open JSON configuration using paths such as `src/ROS_Node/geofence.json`.

## Editing rules

- Treat `src/GUI/GUI_Sampler.ui` and `src/GUI/single_drone_flight.ui` as the source of truth for GUI layout changes, for the multi-drone and single-drone stations respectively. Do not manually edit the corresponding generated `*.py` files; they contain a generated-file warning and will be overwritten.
- Regenerate the active UI after editing it (run from the repository root; the script reads from and writes to `src/GUI`):

  ```bash
  ./scripts/convert_ui.sh GUI_Sampler.ui GUI_SamplerdotUI.py
  ./scripts/convert_ui.sh single_drone_flight.ui
  ```

- Review the generated diff after regeneration to ensure the `.ui` and Python output remain synchronized.
- Keep Qt widget `objectName` values stable unless every corresponding reference in the ROS controller is updated.
- Preserve the ROS/Qt threading boundary. ROS executors run in `QThread`; GUI widgets must be updated through Qt signals/slots on the GUI side.
- Keep publishers, subscriptions, clients, timers, executors, and threads referenced for as long as they are needed. Shutdown must not leave an executor or worker thread running.
- Access shared `CommonData` state under its `QMutex`. Keep lock sections short, copy values locally, always unlock on every acquired path, and preserve the existing non-blocking `tryLock()` behavior unless deliberately changing the concurrency model.
- For each new UAV-specific topic, derive the namespace from the drone ID (`/uav_{i}/...`) and store subscriptions/publishers so ROS objects remain alive.
- Match PX4 QoS behavior: telemetry uses best-effort/transient-local/keep-last/depth-5, while PX4 command and setpoint debug streams use best-effort/volatile/keep-last/depth-5 unless the upstream interface explicitly requires otherwise.
- Check the sourced workspace's message definitions before changing custom message fields.
- Be explicit about coordinate frames and units. PX4 data is commonly NED/FRD, while the GUI and estimator may use ENU/FLU; quaternion ordering differs across interfaces, and `PositionControllerReference.yaw` is published in degrees. Do not silently change axis order, quaternion order, yaw wrapping, thrust signs, frame IDs, or other conversion conventions.
- Avoid broad cleanup of commented legacy ROS 1 (water-sampling) code unless the task specifically includes migration or removal.
- Keep configuration JSON valid and preserve its existing schema unless all readers are updated together.

## Validation

There is currently no automated test suite, formatter, linter, or dependency manifest in the repository. Follow the surrounding style rather than reformatting entire files, and use the checks appropriate to the change:

```bash
# Parse/compile all Python sources without starting ROS or Qt
python3 -m compileall -q src

# Validate JSON configuration
python3 -m json.tool src/ROS_Node/geofence.json >/dev/null
python3 -m json.tool src/ROS_Node/slung_load.json >/dev/null

# Inspect the patch and accidental generated-file changes
git diff --check
git diff --stat
```

For GUI or ROS changes, also perform a manual smoke test in a sourced ROS 2 workspace. Confirm the window opens, the intended UAV tabs update, topic names, message fields, coordinate frames, units, and QoS match the running system, buttons publish/call only their intended interfaces, and shutdown does not leave the executor running. If `pyqtgraph` is unavailable, confirm that the single-drone application still opens and degrades gracefully.

Hardware-affecting controls such as arm, disarm, takeoff, land, emergency stop, and position commands must not be exercised on a live vehicle without explicit authorization and the normal flight-safety setup. Prefer simulation for validation.

## Change hygiene

- Check `git status --short` before and after editing; preserve unrelated and untracked user files.
- Keep changes scoped. This codebase has no enforced style tool, so follow the surrounding Python style instead of reformatting whole files.
- When adding a dependency or changing setup steps, document it in `README.md` (and add an appropriate dependency manifest if the task calls for packaging).
- Summarize which validation was run and clearly identify checks that require unavailable ROS messages, a display server, simulation, or hardware.
