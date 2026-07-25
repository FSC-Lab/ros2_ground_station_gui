# Ground Station GUI for ROS 2

[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity)

### A python based ground station for indoor and outdoor quadrotor drone experiment

## How to use

- make sure the roscore has been launched
- run one of the following from the repository root to start the ground station GUI:

```bash
# multi-drone station (UAV IDs 0, 1, 2)
python3 src/GroundControl.py

# single-drone station (uav_0)
python3 src/single_drone_ground_control.py
```
