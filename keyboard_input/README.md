# Keyboard Input

This package is a lightweight keyboard control node for the current
`quadruped_pkg` simulation.

It reads key presses from the terminal and publishes
`quadruped_controller_msgs/msg/Inputs` to `/control_input`, which is the topic
subscribed by `quadruped_controller`.

The project is still under development. At this stage the node is mainly used
to reduce repeated manual `ros2 topic pub` commands while testing mode switches
and FreeStand body pose inputs.

Tested environment:

* Ubuntu 22.04
  * ROS2 Humble

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to keyboard_input
source install/setup.bash
```

If the workspace contains another package with the same name as
`go2_description`, build only this workspace path or the needed packages.

## Run

Start the simulation/controller first:

```bash
ros2 launch quadruped_controller mujoco_control.launch.py
```

Then open another terminal and run:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

Keep this terminal focused, because the node reads directly from stdin.

## Key Map

Mode keys:

* `1`: PASSIVE
* `2`: FIXEDSTAND
* `3`: TROTTING
* `4`: FREESTAND

FreeStand pose keys:

* `A` / `D`: roll left/right (`lx`)
* `W` / `S`: pitch forward/back (`ly`)
* `J` / `L`: yaw left/right (`rx`)
* `I` / `K`: body height up/down (`ry`)
* `Space`: reset `lx`, `ly`, `rx`, and `ry` to zero

The pose inputs are normalized to `[-1, 1]`. Each key press changes the
corresponding value by `0.05`.

## Current Notes

* This package is a convenience input tool, not a finished operator interface.
* FreeStand roll and height inputs have been used for quick IK checks.
* Long-duration TROTTING behavior still needs separate simulation validation.
* Keys `5` to `0` are reserved by the node but are not part of the current
  stable controller workflow.
