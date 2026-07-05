# quadruped_pkg

`quadruped_pkg` is a ROS 2 Humble quadruped-control project for the Unitree Go2 simulation. It combines a Go2 URDF/MuJoCo description, a MuJoCo-backed `ros2_control` hardware interface, a finite-state quadruped controller, Pinocchio-based kinematics/dynamics, and a centroidal-MPC trotting pipeline.

The current workflow is focused on simulation validation: bring the robot up in MuJoCo, switch between standing/free-stand/MPC trotting modes, and inspect the model in RViz and the embedded MuJoCo viewer.

<table>
  <tr>
    <td align="center"><strong>RViz</strong></td>
    <td align="center"><strong>MuJoCo Simulation</strong></td>
  </tr>
  <tr>
    <td><img src=".images/rviz.png" alt="Go2 model in RViz" width="420"/></td>
    <td><img src=".images/mujoco.png" alt="Go2 simulation in MuJoCo" width="420"/></td>
  </tr>
</table>

<p align="center">
  <strong>MPC State, Force, and Torque Logs</strong><br/>
  <img src=".images/mpc_metrics_flat_demo/mpc_state_force_torque_summary.png" alt="MPC state, contact force, and joint torque logs" width="860"/>
</p>

<p align="center">
  <strong>MPC Contact Forces</strong><br/>
  <img src=".images/mpc_metrics_flat_demo/mpc_contact_forces.png" alt="MPC contact force logs" width="860"/>
</p>

## Packages

- `go2_description`: Go2 URDF/Xacro model, mesh assets, MuJoCo model assets, RViz config, and `ros2_control` hardware configuration.
- `quadruped_mujoco_hardware`: MuJoCo virtual hardware plugin for `ros2_control`, including joint states, actuator torques, IMU, foot-force interfaces, odometer state, and optional embedded viewer.
- `quadruped_controller`: Main controller plugin. It includes the FSM states, Go2 Pinocchio model wrapper, gait generation, COM trajectory generation, centroidal MPC, and leg torque control.
- `quadruped_controller_msgs`: Custom input message used by the keyboard command node.
- `keyboard_input`: Terminal keyboard node and scripted MPC command scheduler that publish `quadruped_controller_msgs/msg/Inputs` to `/control_input`.
- `third_party`: Local third-party solver dependencies used by the controller.

## Main Control States

- `PASSIVE`: Zero-torque safe state.
- `FIXEDSTAND`: Position-controlled standing state.
- `FREESTAND`: Body pose adjustment state for IK and posture checks.
- `MPC_TROTTING`: Integrated locomotion state using `ComTrajectory + CentroidalMPC + LegController`.

## Controller Overview

The motion-control stack currently includes:

- **Centroidal MPC (~48 Hz)**  
  Contact-force-based centroidal MPC implemented in C++ with OSQP/OsqpEigen. It solves a convex QP over one gait-cycle prediction horizon, divided into 16 time steps, and outputs optimized ground reaction forces for each foot.

- **Reference Trajectory Generator (~48 Hz)**  
  Generates the desired CoM position, velocity, attitude, angular velocity, and foot-lever references for the MPC horizon from user body-frame velocity and yaw-rate commands.

- **Swing/Stance Leg Controller (500 Hz)**  
  Swing legs use Cartesian-space impedance control with feedforward swing acceleration and Pinocchio-based Jacobian dynamics. Stance legs map the optimized MPC contact forces into joint torques through the foot Jacobian.

- **Gait Scheduler and Foot Trajectory Generator (500 Hz)**  
  Schedules swing/stance timing in `FL FR RL RR` leg order, computes touchdown positions using a Raibert-style foot-placement rule, and generates swing-foot trajectories with a quintic polynomial and adjustable apex height.

## Prerequisites

The project is developed for Ubuntu 22.04 and ROS 2 Humble.

Install common build tools and ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-matplotlib \
  libeigen3-dev libglfw3-dev libgl1-mesa-dev \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-imu-sensor-broadcaster \
  ros-humble-xacro ros-humble-rviz2 \
  ros-humble-pinocchio ros-humble-kdl-parser
```

External libraries:

- Install the MuJoCo C/C++ library. The default CMake path is `/opt/mujoco`, and this directory should contain `include/mujoco/mujoco.h` and `lib/libmujoco.so`.
- `OSQP` and `OsqpEigen` are already provided in `third_party/`, so no extra system installation is required for them.

## Build

Clone this repository into a clean ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/HongjianHu/quadruped_pkg.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Run this once if rosdep has not been initialized on your machine:
# sudo rosdep init
rosdep update
rosdep install --from-paths src/quadruped_pkg --ignore-src -r -y

colcon build --symlink-install --packages-up-to quadruped_controller keyboard_input
source install/setup.bash
```

If MuJoCo is not installed at `/opt/mujoco`, build with:

```bash
colcon build --symlink-install \
  --packages-up-to quadruped_controller keyboard_input \
  --cmake-args -DMUJOCO_ROOT=/path/to/mujoco
```

## Run Simulation

Start the controller, MuJoCo hardware simulation, robot state publisher, broadcasters, and RViz:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch quadruped_controller mujoco_control.launch.py use_rviz:=true
```

Useful launch arguments:

- `use_rviz:=true`: Start RViz with the Go2 model.
- `use_embedded_mujoco_viewer:=true`: Start the embedded MuJoCo viewer inside the hardware interface.
- `use_mujoco_viewer:=true`: Start the standalone MuJoCo `simulate` viewer for model inspection.
- `mujoco_scene:=scene.xml`: Select the MuJoCo scene file. Relative paths are resolved under `go2_description/mujoco/go2_unitree/`.

Example without RViz:

```bash
ros2 launch quadruped_controller mujoco_control.launch.py use_rviz:=false
```

Example on a flat ground scene:

```bash
ros2 launch quadruped_controller mujoco_control.launch.py \
  use_rviz:=false \
  use_embedded_mujoco_viewer:=true \
  mujoco_scene:=scene_flat.xml
```

## MPC Demo Recording and Plots

The MPC plots shown above are generated from a scripted flat-ground MuJoCo run. Start the flat scene first:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch quadruped_controller mujoco_control.launch.py \
  use_rviz:=false \
  use_embedded_mujoco_viewer:=true \
  mujoco_scene:=scene_flat.xml
```

In a second terminal, record the debug topics to CSV:

```bash
cd ~/ros2_ws/src/quadruped_pkg
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 scripts/record_mpc_debug_csv.py \
  --output-dir /tmp/quadruped_plot_data/scheduled_flat_demo \
  --duration 26.0 \
  --no-command
```

In a third terminal, publish the scripted MPC command sequence:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_input mpc_command_scheduler --ros-args \
  -p fixedstand_duration:=5.0 \
  -p speed_scale:=1.0
```

Then generate the figures:

```bash
cd ~/ros2_ws/src/quadruped_pkg
python3 scripts/plot_mpc_metrics.py \
  --csv /tmp/quadruped_plot_data/scheduled_flat_demo/estimator_debug.csv \
  --joint-csv /tmp/quadruped_plot_data/scheduled_flat_demo/joint_states.csv \
  --out-dir .images/mpc_metrics_flat_demo \
  --start 4.5 \
  --end 23.0 \
  --ref-height 0.27
```

The scripted demo includes forward walking, lateral motion, yaw rotation, combined forward turning, and a faster forward segment. With the default MPC scaling, the maximum commanded forward velocity is about `0.50 m/s`.

## Keyboard Control

Run the keyboard node in another terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

Keep this terminal focused, because it reads directly from stdin.

<table>
  <tr>
    <th>Mode keys</th>
    <th>FreeStand keys</th>
    <th>MPC keys</th>
  </tr>
  <tr>
    <td>
      <code>1</code>: <code>PASSIVE</code><br/>
      <code>2</code>: <code>FIXEDSTAND</code><br/>
      <code>4</code>: <code>FREESTAND</code><br/>
      <code>6</code>: <code>MPC_TROTTING</code>
    </td>
    <td>
      <code>A</code> / <code>D</code>: roll<br/>
      <code>W</code> / <code>S</code>: pitch<br/>
      <code>J</code> / <code>L</code>: yaw<br/>
      <code>I</code> / <code>K</code>: body height<br/>
      <code>Space</code>: reset command axes
    </td>
    <td>
      <code>A</code> / <code>D</code>: desired body-frame <code>lx</code><br/>
      <code>W</code> / <code>S</code>: desired body-frame <code>ly</code><br/>
      <code>J</code> / <code>L</code>: desired yaw rate<br/>
      <code>I</code> / <code>K</code>: desired body height<br/>
      <code>Space</code>: reset command axes
    </td>
  </tr>
</table>

## What's Next

- **More Robot Models**: extend the current description and MuJoCo hardware pipeline to support additional quadruped models.
- **Try more controllers**: build upon the current MPC framework by integrating a Whole-Body Control (WBC) layer as the lower-level torque allocation module, forming a hierarchical MPC+WBC architecture for enhanced disturbance rejection. Meanwhile, explore model-free alternatives such as Reinforcement Learning to develop adaptive locomotion policies for challenging unstructured terrains.
