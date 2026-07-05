#!/usr/bin/env python3
"""Plot quadruped MPC simulation metrics from recorded CSV files."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Iterable, Optional

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
import numpy as np


LEG_NAMES = ("FL", "FR", "RL", "RR")
LEG_COLORS = {
    "FL": "#1f77b4",
    "FR": "#ff7f0e",
    "RL": "#2ca02c",
    "RR": "#d62728",
}
AXIS_COLORS = {
    "x": "#1f77b4",
    "y": "#ff7f0e",
    "z": "#2ca02c",
}
JOINT_COLORS = {
    "hip": "#1f77b4",
    "thigh": "#ff7f0e",
    "calf": "#2ca02c",
}


class Table:
    def __init__(self, columns: dict[str, np.ndarray]) -> None:
        self.columns = columns

    def __contains__(self, key: str) -> bool:
        return key in self.columns

    def __getitem__(self, key: str) -> np.ndarray:
        return self.columns[key]


def load_csv(path: Path) -> Table:
    with path.open("r", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    if "t" not in fieldnames:
        raise ValueError(f"{path} does not contain a 't' column")

    columns: dict[str, np.ndarray] = {}
    for name in fieldnames:
        values = []
        for row in rows:
            try:
                values.append(float(row.get(name, "nan")))
            except ValueError:
                values.append(float("nan"))
        columns[name] = np.asarray(values, dtype=float)

    valid = np.isfinite(columns["t"])
    columns = {name: values[valid] for name, values in columns.items()}
    if columns["t"].size == 0:
        raise ValueError(f"{path} does not contain finite timestamps")
    columns["time"] = columns["t"] - columns["t"][0]
    return Table(columns)


def crop_time(table: Table, start: float, end: Optional[float]) -> Table:
    time = table["time"]
    mask = time >= start
    if end is not None:
        mask &= time <= end
    if not np.any(mask):
        return Table({name: values.copy() for name, values in table.columns.items()})
    columns = {name: values[mask].copy() for name, values in table.columns.items()}
    columns["time"] = columns["time"] - columns["time"][0]
    return Table(columns)


def style_axes(axes: Iterable[plt.Axes]) -> None:
    for ax in axes:
        ax.grid(True, color="#b7b7b7", linewidth=0.65, alpha=0.9)
        ax.tick_params(axis="both", labelsize=8)
        for spine in ax.spines.values():
            spine.set_linewidth(0.8)


def first_existing(df: Table, names: Iterable[str]) -> Optional[str]:
    for name in names:
        if name in df:
            return name
    return None


def setup_plot_style() -> None:
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.titlesize": 11,
            "axes.labelsize": 9,
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "figure.dpi": 100,
            "savefig.dpi": 160,
            "lines.linewidth": 1.35,
        }
    )


def save_base_tracking(df: Table, out: Path, ref_height: float, ref_vx: float, ref_vy: float) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(10.0, 7.0), sharex=True, constrained_layout=True)
    fig.suptitle("Base Tracking in MuJoCo Simulation", fontsize=13)

    series = [
        ("odom_pos_z", "base height z [m]", ref_height),
        ("odom_vel_x", "forward velocity vx [m/s]", ref_vx),
        ("odom_vel_y", "lateral velocity vy [m/s]", ref_vy),
        (first_existing(df, ("rpy_yaw", "rpy_z")), "yaw angle [rad]", 0.0),
    ]
    for ax, (column, ylabel, ref) in zip(axes, series):
        if column is not None and column in df:
            ax.plot(df["time"], df[column], color="#2f5f98", linewidth=1.6, label="actual")
        ax.axhline(ref, color="#c23b22", linestyle="--", linewidth=1.2, label="reference")
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right", frameon=False, fontsize=8)

    axes[-1].set_xlabel("time [s]")
    style_axes(axes)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def save_contact_forces(df: Table, out: Path, contact_threshold: float) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10.0, 5.5), sharex=True, constrained_layout=True)
    fig.suptitle("Contact Schedule and Vertical Ground Reaction Forces", fontsize=13)

    for leg in LEG_NAMES:
        force_col = f"foot_force_{leg}"
        if force_col in df:
            axes[0].plot(df["time"], df[force_col], color=LEG_COLORS[leg], linewidth=1.4, label=leg)
    axes[0].set_ylabel("measured Fz [N]")
    axes[0].legend(ncol=4, frameon=False, fontsize=8)

    offsets = np.arange(len(LEG_NAMES))[::-1] * 1.25
    for offset, leg in zip(offsets, LEG_NAMES):
        force_col = f"foot_force_{leg}"
        if force_col in df:
            contact = (df[force_col] > contact_threshold).astype(float)
            axes[1].step(df["time"], contact + offset, where="post", color=LEG_COLORS[leg], linewidth=1.4)
            axes[1].text(df["time"][0], offset + 0.35, leg, color=LEG_COLORS[leg], fontsize=9, va="center")
    axes[1].set_ylabel(f"measured contact > {contact_threshold:g} N")
    axes[1].set_yticks([])
    axes[1].set_xlabel("time [s]")

    style_axes(axes)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def save_foot_height(df: Table, out: Path) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(10.0, 7.0), sharex=True, constrained_layout=True)
    fig.suptitle("Foot Height Trajectories", fontsize=13)
    for ax, leg in zip(axes, LEG_NAMES):
        column = f"feet_pos_world_{leg}_z"
        if column in df:
            ax.plot(df["time"], df[column], color=LEG_COLORS[leg], linewidth=1.5)
        ax.set_ylabel(f"{leg} z [m]")
    axes[-1].set_xlabel("time [s]")
    style_axes(axes)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def save_joint_effort(joint_df: Table, out: Path) -> None:
    effort_columns = [col for col in joint_df.columns if col.endswith("_effort")]
    if not effort_columns:
        return

    fig, ax = plt.subplots(1, 1, figsize=(10.0, 4.6), constrained_layout=True)
    fig.suptitle("Joint Effort Envelope", fontsize=13)

    effort = np.vstack([np.abs(joint_df[col]) for col in effort_columns])
    ax.plot(joint_df["time"], np.nanmax(effort, axis=0), color="#2f5f98", linewidth=1.6, label="max |effort|")
    ax.axhline(23.0, color="#db7c26", linestyle="--", linewidth=1.2, label="hip/thigh limit 23 Nm")
    ax.axhline(35.0, color="#c23b22", linestyle="--", linewidth=1.2, label="calf limit 35 Nm")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("effort [Nm]")
    ax.legend(frameon=False, fontsize=8)
    style_axes([ax])
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_xyz(ax: plt.Axes, df: Table, prefix: str, title: str, ylabel: str, labels: tuple[str, str, str]) -> None:
    plotted = False
    for axis, label in zip(("x", "y", "z"), labels):
        col = f"{prefix}_{axis}"
        if col in df:
            ax.plot(df["time"], df[col], color=AXIS_COLORS[axis], label=label)
            plotted = True
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    if plotted:
        ax.legend(loc="best", framealpha=0.85)


def save_state_force_torque_summary(
    df: Table,
    joint_df: Optional[Table],
    out: Path,
    ref_height: float,
    contact_threshold: float,
) -> None:
    fig, axes = plt.subplots(4, 3, figsize=(16.0, 9.5), sharex="col")
    fig.suptitle("MPC State, Force, and Torque Logs", fontsize=14)

    for row, leg in enumerate(LEG_NAMES):
        ax = axes[row, 0]
        force_col = f"foot_force_{leg}"
        if force_col in df:
            ax.plot(df["time"], df[force_col], color=LEG_COLORS[leg], label="measured $f_z$")
            ax.axhline(contact_threshold, color="#777777", linestyle="--", linewidth=1.0, label="contact threshold")
        ax.set_title(f"{leg} Foot Vertical Contact Force (N)")
        ax.set_ylabel("force [N]")
        ax.legend(loc="best", framealpha=0.85)

        ax = axes[row, 1]
        if joint_df is not None:
            for joint in ("hip", "thigh", "calf"):
                col = f"{leg}_{joint}_joint_effort"
                if col in joint_df:
                    ax.plot(joint_df["time"], joint_df[col], color=JOINT_COLORS[joint], label=f"{joint}_tau")
            ax.axhline(23.0, color="#888888", linestyle="--", linewidth=0.9)
            ax.axhline(-23.0, color="#888888", linestyle="--", linewidth=0.9)
        ax.set_title(f"{leg} Leg Joint Torque (Nm)")
        ax.set_ylabel("torque [Nm]")
        ax.legend(loc="best", framealpha=0.85)

    plot_xyz(
        axes[0, 2],
        df,
        "odom_pos",
        "CoM Position in World Frame (m)",
        "position [m]",
        ("x-position", "y-position", "z-position"),
    )
    axes[0, 2].axhline(ref_height, color="#555555", linestyle="--", linewidth=0.8, label="z reference")
    axes[0, 2].legend(loc="best", framealpha=0.85)

    rpy_cols = {
        "roll": first_existing(df, ("rpy_roll", "rpy_x")),
        "pitch": first_existing(df, ("rpy_pitch", "rpy_y")),
        "yaw": first_existing(df, ("rpy_yaw", "rpy_z")),
    }
    for name, col in rpy_cols.items():
        if col is not None:
            axes[1, 2].plot(df["time"], df[col], label=name)
    axes[1, 2].set_title("ZYX Euler (rad)")
    axes[1, 2].set_ylabel("angle [rad]")
    axes[1, 2].legend(loc="best", framealpha=0.85)

    plot_xyz(axes[2, 2], df, "odom_vel", "CoM Velocity in World Frame (m/s)", "velocity [m/s]", ("x-velocity", "y-velocity", "z-velocity"))
    axes[2, 2].axhline(0.0, color="#555555", linestyle="--", linewidth=0.8)

    plot_xyz(axes[3, 2], df, "ang_vel_world", "Angular Velocity in World Frame (rad/s)", "angular velocity [rad/s]", ("roll rate", "pitch rate", "yaw rate"))
    axes[3, 2].axhline(0.0, color="#555555", linestyle="--", linewidth=0.8)

    for ax in axes[-1, :]:
        ax.set_xlabel("time [s]")
    style_axes(axes.ravel())
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(out)
    plt.close(fig)


def save_runtime_stats_from_csv(runtime_csv: Optional[Path], out: Path, real_time_hz: float) -> bool:
    if runtime_csv is None or not runtime_csv.exists():
        return False

    runtime = load_csv(runtime_csv)
    model_col = first_existing(runtime, ("model_update_ms", "update_time_ms", "mpc_update_time_ms"))
    solve_col = first_existing(runtime, ("qp_solve_ms", "solve_time_ms", "mpc_solve_time_ms"))
    if model_col is None or solve_col is None:
        return False

    model_time = runtime[model_col]
    solve_time = runtime[solve_col]
    total_time = model_time + solve_time
    steps = np.arange(model_time.size)
    budget_ms = 1000.0 / real_time_hz

    fig, ax = plt.subplots(1, 1, figsize=(10.0, 5.6))
    ax.set_title("MPC Iteration Stats")
    ax.bar(steps, model_time, width=1.0, label="Model Update Time (ms)")
    ax.bar(steps, solve_time, width=1.0, bottom=model_time, label="QP Solve Time (ms)")
    ax.axhline(budget_ms, color="#1f77b4", linestyle="--", linewidth=1.5, label=f"Real-Time Budget {real_time_hz:.1f} Hz ({budget_ms:.1f} ms)")
    ax.text(
        0.02,
        0.70,
        f"Avg Model Update Time: {np.nanmean(model_time):.2f} ms\n"
        f"Avg QP solve time:  {np.nanmean(solve_time):.2f} ms\n"
        f"Avg MPC Cycle Time:  {np.nanmean(total_time):.2f} ms",
        transform=ax.transAxes,
        bbox={"boxstyle": "round", "facecolor": "#cfe5f5", "edgecolor": "#8aa9bd", "alpha": 0.95},
    )
    ax.set_xlabel("MPC Step")
    ax.set_ylabel("Time (ms)")
    ax.legend(loc="center right", framealpha=0.9)
    style_axes([ax])
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    return True


def metric_summary(df: Table, joint_df: Optional[Table], out: Path, ref_height: float) -> None:
    def rmse(column: str, ref: float = 0.0) -> float:
        if column not in df:
            return float("nan")
        values = df[column].astype(float)
        return float(np.sqrt(np.nanmean((values - ref) ** 2)))

    lines = [
        "# MPC Simulation Metrics",
        "",
        f"- duration_s: {df['time'][-1]:.3f}",
        f"- base_height_rmse_m: {rmse('odom_pos_z', ref_height):.5f}",
        f"- vx_rmse_mps: {rmse('odom_vel_x'):.5f}",
        f"- vy_rmse_mps: {rmse('odom_vel_y'):.5f}",
    ]
    yaw_col = first_existing(df, ("rpy_yaw", "rpy_z"))
    lines.append(
        f"- yaw_abs_max_rad: {float(np.nanmax(np.abs(df[yaw_col]))):.5f}" if yaw_col is not None else "- yaw_abs_max_rad: nan"
    )

    force_cols = [f"foot_force_{leg}" for leg in LEG_NAMES if f"foot_force_{leg}" in df]
    if force_cols:
        total_fz = np.vstack([df[col] for col in force_cols]).sum(axis=0)
        lines.extend(
            [
                f"- total_fz_mean_N: {float(np.nanmean(total_fz)):.3f}",
                f"- total_fz_peak_N: {float(np.nanmax(total_fz)):.3f}",
            ]
        )

    if joint_df is not None:
        effort_cols = [col for col in joint_df.columns if col.endswith("_effort")]
        if effort_cols:
            max_effort = np.nanmax(np.vstack([np.abs(joint_df[col]) for col in effort_cols]), axis=0)
            lines.extend(
                [
                    f"- joint_effort_peak_Nm: {float(np.nanmax(max_effort)):.3f}",
                    f"- joint_effort_over_23Nm_ratio: {float(np.nanmean(max_effort > 23.0)):.5f}",
                    f"- joint_effort_over_35Nm_ratio: {float(np.nanmean(max_effort > 35.0)):.5f}",
                ]
            )

    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path, help="Path to estimator_debug.csv")
    parser.add_argument("--joint-csv", type=Path, help="Optional path to joint_states.csv")
    parser.add_argument("--out-dir", default=".images/mpc_metrics", type=Path, help="Directory for output images")
    parser.add_argument("--start", type=float, default=0.0, help="Crop start time in seconds")
    parser.add_argument("--end", type=float, help="Crop end time in seconds")
    parser.add_argument("--ref-height", type=float, default=0.27)
    parser.add_argument("--ref-vx", type=float, default=0.0)
    parser.add_argument("--ref-vy", type=float, default=0.0)
    parser.add_argument("--contact-threshold", type=float, default=5.0)
    parser.add_argument("--runtime-csv", type=Path, help="Optional CSV with model_update_ms and qp_solve_ms columns")
    parser.add_argument("--mpc-rate", type=float, default=48.0, help="Real-time MPC budget line for runtime plot")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    setup_plot_style()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    df = crop_time(load_csv(args.csv), args.start, args.end)
    joint_df = None
    if args.joint_csv and args.joint_csv.exists():
        joint_df = crop_time(load_csv(args.joint_csv), args.start, args.end)

    save_base_tracking(df, args.out_dir / "mpc_base_tracking.png", args.ref_height, args.ref_vx, args.ref_vy)
    save_contact_forces(df, args.out_dir / "mpc_contact_forces.png", args.contact_threshold)
    save_foot_height(df, args.out_dir / "mpc_foot_height.png")
    if joint_df is not None:
        save_joint_effort(joint_df, args.out_dir / "mpc_joint_effort.png")
    save_state_force_torque_summary(
        df,
        joint_df,
        args.out_dir / "mpc_state_force_torque_summary.png",
        args.ref_height,
        args.contact_threshold,
    )
    save_runtime_stats_from_csv(args.runtime_csv, args.out_dir / "mpc_runtime_stats.png", args.mpc_rate)
    metric_summary(df, joint_df, args.out_dir / "mpc_metrics_summary.md", args.ref_height)

    print(f"Saved plots and metric summary to {args.out_dir}")


if __name__ == "__main__":
    main()
