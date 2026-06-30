# quadruped_pkg

ROS2 quadruped robot control workspace for the Unitree Go2 simulation.

This project is still in progress. The current focus is to bring up a usable
MuJoCo + ros2_control pipeline, migrate the quadruped controller logic, and
validate the Go2 kinematics path step by step.

## Packages

- **go2_description** - Go2 URDF/Xacro, MuJoCo assets, and ros2_control
  hardware configuration.
- **quadruped_controller_msgs** - Custom controller input message.
- **quadruped_mujoco_hardware** - MuJoCo-backed ros2_control hardware
  interface.
- **quadruped_controller** - Main quadruped controller, FSM states,
  estimator, gait logic, and Go2 Pinocchio model wrapper.
- **keyboard_input** - Terminal keyboard node that publishes
  `quadruped_controller_msgs/msg/Inputs` to `/control_input`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to quadruped_controller keyboard_input
source install/setup.bash
```

If the workspace contains another `go2_description` package from a reference
project, build only this package path or temporarily ignore the duplicate
package before running `colcon build`.

## Run Simulation

```bash
ros2 launch quadruped_controller mujoco_control.launch.py
```

The launch file starts the robot description, ros2_control node, broadcasters,
MuJoCo hardware interface, and `quadruped_controller`.

## Keyboard Control

Run this in another terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

Basic keys:

- `1`: PASSIVE
- `2`: FIXEDSTAND
- `3`: TROTTING
- `4`: FREESTAND
- `A/D`: FreeStand roll input
- `W/S`: FreeStand pitch input
- `J/L`: FreeStand yaw input
- `I/K`: FreeStand height input
- `Space`: reset pose inputs

## Current Status

- FixedStand and FreeStand are used for basic simulation and IK checks.
- Go2 kinematics now goes through `quadruped_controller/robot/go2_robot_data`.
- Trotting and balance-control behavior still need more long-duration
  validation and tuning.
- This README is intentionally rough and will be expanded as the project
  stabilizes.
