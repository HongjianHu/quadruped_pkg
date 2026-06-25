# quadruped_pkg

ROS2 quadruped robot controller package.

## Packages

- **quadruped_controller_msgs** — Custom messages for quadruped controller
- **quadruped_controller** — Quadruped robot controller implementation

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select quadruped_controller_msgs quadruped_controller
source install/setup.bash
```
