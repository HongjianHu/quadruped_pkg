#!/usr/bin/env python3
"""Record quadruped MPC simulation debug topics to CSV.

Run this while `mujoco_control.launch.py` is active. The script publishes a
small command sequence by default: FIXEDSTAND, MPC_TROTTING, then zero-speed
MPC hold. It records `/estimator_debug` and `/joint_states`.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    from quadruped_controller_msgs.msg import Inputs
except Exception:  # pragma: no cover - lets the recorder run without command publishing.
    Inputs = None


LEG_NAMES = ("FL", "FR", "RL", "RR")
AXES = ("x", "y", "z")


def estimator_columns() -> List[str]:
    columns = ["t"]
    for prefix in ("est_pos", "est_vel"):
        columns.extend(f"{prefix}_{axis}" for axis in AXES)
    columns.extend(("rpy_roll", "rpy_pitch", "rpy_yaw"))
    columns.extend(f"ang_vel_world_{axis}" for axis in AXES)
    for prefix in ("acc_body", "u_world", "odom_pos", "odom_vel"):
        columns.extend(f"{prefix}_{axis}" for axis in AXES)
    for leg in LEG_NAMES:
        columns.extend(f"feet_pos_world_{leg}_{axis}" for axis in AXES)
    columns.extend(f"foot_force_{leg}" for leg in LEG_NAMES)
    columns.extend(f"force_contact_{leg}" for leg in LEG_NAMES)
    columns.extend(f"slip_detected_{leg}" for leg in LEG_NAMES)
    columns.extend(f"est_contact_{leg}" for leg in LEG_NAMES)
    columns.extend(f"wave_contact_{leg}" for leg in LEG_NAMES)
    columns.extend(f"phase_{leg}" for leg in LEG_NAMES)
    columns.extend(f"pos_err_{axis}" for axis in AXES)
    columns.extend(f"vel_err_{axis}" for axis in AXES)
    for leg in LEG_NAMES:
        columns.extend(f"odom_contact_pos_world_{leg}_{axis}" for axis in AXES)
    columns.extend(f"odom_contact_clearance_{leg}" for leg in LEG_NAMES)
    return columns


class MpcDebugRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mpc_debug_csv_recorder")
        self.args = args
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.estimator_columns = estimator_columns()
        self.estimator_file = (self.output_dir / "estimator_debug.csv").open("w", newline="")
        self.estimator_writer = csv.writer(self.estimator_file)
        self.estimator_writer.writerow(self.estimator_columns)

        self.joint_file = (self.output_dir / "joint_states.csv").open("w", newline="")
        self.joint_writer = csv.writer(self.joint_file)
        self.joint_header_written = False
        self.joint_names: List[str] = []

        if not args.no_command:
            if Inputs is None:
                raise RuntimeError(
                    "quadruped_controller_msgs could not be imported. "
                    "Use --no-command and publish /control_input from keyboard_input instead."
                )
            self.command_pub = self.create_publisher(Inputs, "/control_input", 10)
        else:
            self.command_pub = None
        self.create_subscription(Float64MultiArray, "/estimator_debug", self.estimator_callback, 50)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 50)

        self.start_time: Optional[float] = None
        self.estimator_count = 0
        self.joint_count = 0
        self.timer = self.create_timer(0.02, self.timer_callback)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def elapsed_s(self) -> float:
        now = self.now_s()
        if self.start_time is None:
            self.start_time = now
        return now - self.start_time

    def command_for_time(self, t: float) -> "Inputs":
        msg = Inputs()
        if t < self.args.fixedstand_time:
            msg.command = 2
        elif t < self.args.fixedstand_time + self.args.mpc_switch_time:
            msg.command = 6
        else:
            msg.command = 0
            msg.lx = self.args.lx
            msg.ly = self.args.ly
            msg.rx = self.args.yaw_rate
            msg.ry = self.args.height
        return msg

    def timer_callback(self) -> None:
        t = self.elapsed_s()
        if self.command_pub is not None:
            self.command_pub.publish(self.command_for_time(t))
        if t >= self.args.duration:
            self.get_logger().info(
                f"Recorded {self.estimator_count} estimator rows and {self.joint_count} joint rows to {self.output_dir}"
            )
            rclpy.shutdown()

    def estimator_callback(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < len(self.estimator_columns):
            data.extend([float("nan")] * (len(self.estimator_columns) - len(data)))
        self.estimator_writer.writerow(data[: len(self.estimator_columns)])
        self.estimator_count += 1

    def joint_state_callback(self, msg: JointState) -> None:
        if not self.joint_header_written:
            self.joint_names = list(msg.name)
            columns = ["t"]
            columns.extend(f"{name}_pos" for name in self.joint_names)
            columns.extend(f"{name}_vel" for name in self.joint_names)
            columns.extend(f"{name}_effort" for name in self.joint_names)
            self.joint_writer.writerow(columns)
            self.joint_header_written = True

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        row = [t]
        row.extend(msg.position[: len(self.joint_names)])
        row.extend(msg.velocity[: len(self.joint_names)])
        row.extend(msg.effort[: len(self.joint_names)])
        self.joint_writer.writerow(row)
        self.joint_count += 1

    def destroy_node(self) -> bool:
        self.estimator_file.close()
        self.joint_file.close()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="logs/mpc_metrics/latest", help="Directory for output CSV files.")
    parser.add_argument("--duration", type=float, default=10.0, help="Recording duration in seconds.")
    parser.add_argument("--fixedstand-time", type=float, default=2.0, help="Seconds spent publishing FIXEDSTAND.")
    parser.add_argument("--mpc-switch-time", type=float, default=1.0, help="Seconds spent publishing MPC_TROTTING switch.")
    parser.add_argument("--lx", type=float, default=0.0, help="Normalized MPC forward command [-1, 1].")
    parser.add_argument("--ly", type=float, default=0.0, help="Normalized MPC lateral command [-1, 1].")
    parser.add_argument("--yaw-rate", type=float, default=0.0, help="Normalized MPC yaw-rate command [-1, 1].")
    parser.add_argument("--height", type=float, default=0.0, help="Normalized MPC height-rate command [-1, 1].")
    parser.add_argument("--no-command", action="store_true", help="Only record topics; do not publish /control_input.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = MpcDebugRecorder(args)
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
