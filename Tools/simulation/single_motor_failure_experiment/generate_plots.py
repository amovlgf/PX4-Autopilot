#!/usr/bin/env python3
"""Generate forum-ready figures from the extracted experiment results."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch
from pyulog import ULog


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
HEIGHTS = [2.5, 10.0, 20.0]
DELAYS = [0.0, 5.0]
MOTORS = [1, 2, 3, 4]


def _dataset(ulog, name):
    for dataset in ulog.data_list:
        if dataset.name == name and dataset.multi_id == 0:
            return dataset.data

    raise RuntimeError(f"missing {name}")


def _quaternion_to_roll_pitch_deg(attitude):
    w = attitude["q[0]"]
    x = attitude["q[1]"]
    y = attitude["q[2]"]
    z = attitude["q[3]"]
    roll = np.arctan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return np.degrees(roll), np.degrees(pitch)


def _condition_match(result, motor, height, delay):
    return (
        result.get("status") == "ok"
        and result.get("motor_number") == motor
        and abs(float(result.get("mis_takeoff_alt", -1)) - height) < 0.1
        and abs(float(result.get("com_fail_act_t", -1)) - delay) < 0.1
    )


def _save(fig, output_path):
    fig.savefig(output_path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def plot_matrix(results, output_dir):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
    pass_color = np.array([0.25, 0.68, 0.40])
    fail_color = np.array([0.86, 0.30, 0.28])
    missing_color = np.array([0.75, 0.75, 0.75])

    for axis, delay in zip(axes, DELAYS):
        image = np.empty((len(HEIGHTS), len(MOTORS), 3))

        for row, height in enumerate(HEIGHTS):
            for column, motor in enumerate(MOTORS):
                matches = [
                    result for result in results
                    if _condition_match(result, motor, height, delay)
                ]

                if not matches:
                    image[row, column] = missing_color

                elif matches[0].get("safe_threshold_pass"):
                    image[row, column] = pass_color

                else:
                    image[row, column] = fail_color

                label = "PASS" if matches and matches[0].get("safe_threshold_pass") else "FAIL"

                if not matches:
                    label = "N/A"

                axis.text(
                    column,
                    row,
                    label,
                    ha="center",
                    va="center",
                    color="white" if label != "N/A" else "black",
                    weight="bold",
                    fontsize=10,
                )

        axis.imshow(image, aspect="auto")
        axis.set_xticks(range(len(MOTORS)), [f"Motor {motor}" for motor in MOTORS])
        axis.set_yticks(range(len(HEIGHTS)), [f"{height:g} m" for height in HEIGHTS])
        axis.set_title(f"COM_FAIL_ACT_T = {delay:g} s")
        axis.set_xlabel("Injected motor failure")
        axis.set_ylabel("Takeoff altitude")

    fig.suptitle("Single-Motor-Failure SITL Safety Threshold Matrix", fontsize=14)
    _save(fig, output_dir / "matrix_pass_heatmap.png")


def plot_metrics(results, output_dir):
    metrics = [
        ("max_abs_roll_deg", "Maximum |roll|", "deg", 60.0),
        ("max_abs_pitch_deg", "Maximum |pitch|", "deg", 60.0),
        ("max_down_speed_m_s", "Maximum down speed", "m/s", 3.0),
        ("max_horizontal_drift_m", "Maximum horizontal drift", "m", 5.0),
        ("max_abs_yaw_rate_rad_s", "Maximum |yaw rate|", "rad/s", None),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.5), constrained_layout=True)
    flattened = axes.ravel()
    colors = {0.0: "#2878B5", 5.0: "#F28E2B"}

    for axis, (field, title, unit, threshold) in zip(flattened, metrics):
        for delay in DELAYS:
            for height_index, height in enumerate(HEIGHTS):
                values = [
                    result[field] for result in results
                    if result.get("status") == "ok"
                    and abs(float(result.get("mis_takeoff_alt", -1)) - height) < 0.1
                    and abs(float(result.get("com_fail_act_t", -1)) - delay) < 0.1
                    and result.get(field) is not None
                ]
                offsets = np.linspace(-0.08, 0.08, len(values)) if values else []
                base = height_index + (-0.15 if delay == 0.0 else 0.15)
                axis.scatter(
                    np.asarray(offsets) + base,
                    values,
                    color=colors[delay],
                    label=f"delay {delay:g} s" if height_index == 0 else None,
                    s=30,
                    alpha=0.85,
                )

        if threshold is not None:
            axis.axhline(
                threshold,
                color="#C33",
                linestyle="--",
                linewidth=1,
                label="safety threshold",
            )

        axis.set_xticks(range(len(HEIGHTS)), [f"{height:g} m" for height in HEIGHTS])
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.grid(alpha=0.25)

    flattened[-1].axis("off")
    handles, labels = flattened[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower right", bbox_to_anchor=(0.97, 0.06))
    fig.suptitle("Post-Failure Dynamic Response by Test Height", fontsize=14)
    _save(fig, output_dir / "metrics_by_height.png")


def plot_response_delays(results, output_dir):
    fig, axis = plt.subplots(figsize=(10, 4.8), constrained_layout=True)
    ordered = sorted(
        [result for result in results if result.get("status") == "ok"],
        key=lambda result: (
            result.get("com_fail_act_t"),
            result.get("mis_takeoff_alt"),
            result.get("motor_number"),
        ),
    )
    x = np.arange(len(ordered))
    handled = [result.get("handled_delay_s") for result in ordered]
    land = [result.get("land_delay_s") for result in ordered]
    labels = [
        f"D{result['com_fail_act_t']:g}/H{result['mis_takeoff_alt']:g}/M{result['motor_number']}"
        for result in ordered
    ]
    axis.scatter(x, handled, label="allocator handled", marker="o", color="#2878B5")
    axis.scatter(x, land, label="Land mode active", marker="x", color="#D94F4F")
    axis.axhline(0.5, color="#555", linestyle="--", linewidth=1, label="0.5 s criterion")
    axis.set_xticks(x, labels, rotation=65, ha="right", fontsize=8)
    axis.set_ylabel("Seconds after motor stop mask")
    axis.set_title("Failure Handling and Land-Mode Response")
    axis.grid(axis="y", alpha=0.25)
    axis.legend()
    _save(fig, output_dir / "response_delays.png")


def _draw_box(axis, x, y, width, height, text, color):
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.02",
        linewidth=1.5,
        edgecolor=color,
        facecolor="white",
    )
    axis.add_patch(patch)
    axis.text(x + width / 2, y + height / 2, text, ha="center", va="center", fontsize=9)


def plot_data_flow(output_dir):
    fig, axis = plt.subplots(figsize=(13, 4.2), constrained_layout=True)
    axis.set_xlim(0, 13.3)
    axis.set_ylim(0, 4)
    axis.axis("off")
    boxes = [
        (0.2, 2.4, "Failure\ninjection", "#666"),
        (2.0, 2.4, "Failure detector\nstatus", "#2878B5"),
        (4.0, 2.4, "Commander /\nFailsafe", "#2878B5"),
        (6.0, 2.4, "Auto Land /\nsetpoints", "#2878B5"),
        (8.0, 2.4, "Rate\ncontroller", "#2878B5"),
        (10.0, 2.4, "Control\nAllocator", "#2878B5"),
        (11.8, 2.4, "Actuator\nmotors", "#2878B5"),
        (10.0, 0.5, "jMAVSim\nvehicle", "#F28E2B"),
        (7.6, 0.5, "Sensors /\nEKF", "#F28E2B"),
        (5.2, 0.5, "Vehicle\nstate", "#F28E2B"),
    ]
    width = 1.2
    height = 0.8

    for x, y, text, color in boxes:
        _draw_box(axis, x, y, width, height, text, color)

    arrows = [
        ((1.4, 2.8), (2.0, 2.8)),
        ((3.2, 2.8), (4.0, 2.8)),
        ((5.2, 2.8), (6.0, 2.8)),
        ((7.2, 2.8), (8.0, 2.8)),
        ((9.2, 2.8), (10.0, 2.8)),
        ((11.2, 2.8), (11.8, 2.8)),
        ((12.4, 2.4), (10.8, 1.3)),
        ((10.0, 0.9), (8.8, 0.9)),
        ((7.6, 0.9), (6.4, 0.9)),
        ((5.8, 1.3), (6.6, 2.4)),
        ((3.2, 2.55), (10.0, 1.3)),
    ]

    for start, end in arrows:
        axis.annotate(
            "",
            xy=end,
            xytext=start,
            arrowprops={"arrowstyle": "->", "color": "#444", "linewidth": 1.4},
        )

    axis.text(
        0.2,
        0.2,
        "Blue: PX4 failure/action/control path    Orange: simulated plant and feedback",
        fontsize=9,
        color="#444",
    )
    axis.set_title("PX4 Single-Motor-Failure Experiment Data Flow", fontsize=14)
    _save(fig, output_dir / "data_flow.png")


def _relative_time(data, reference_us):
    return (data["timestamp"].astype(np.int64) - int(reference_us)) / 1e6


def _load_representative(result):
    path = Path(result["path"])

    if not path.is_absolute():
        path = REPOSITORY_ROOT / path

    ulog = ULog(
        str(path),
        message_name_filter_list=[
            "failure_detector_status",
            "vehicle_angular_velocity",
            "vehicle_attitude",
            "vehicle_local_position",
        ],
    )
    failure = _dataset(ulog, "failure_detector_status")
    injection_indices = np.flatnonzero(failure["motor_stop_mask"] != 0)

    if not injection_indices.size:
        raise RuntimeError(f"no motor failure in {path}")

    return ulog, int(failure["timestamp"][int(injection_indices[0])])


def plot_representatives(results, output_dir):
    selected = []

    for height in [2.5, 10.0]:
        matches = [
            result for result in results
            if _condition_match(result, motor=1, height=height, delay=5.0)
        ]

        if not matches:
            return

        selected.append(matches[0])

    fig, axes = plt.subplots(4, 2, figsize=(13, 10), sharex="col", constrained_layout=True)

    for column, result in enumerate(selected):
        ulog, injection_time_us = _load_representative(result)
        attitude = _dataset(ulog, "vehicle_attitude")
        angular_velocity = _dataset(ulog, "vehicle_angular_velocity")
        local_position = _dataset(ulog, "vehicle_local_position")
        roll, pitch = _quaternion_to_roll_pitch_deg(attitude)
        attitude_time = _relative_time(attitude, injection_time_us)
        angular_time = _relative_time(angular_velocity, injection_time_us)
        position_time = _relative_time(local_position, injection_time_us)
        reference_index = int(np.argmin(np.abs(position_time)))
        drift = np.hypot(
            local_position["x"] - local_position["x"][reference_index],
            local_position["y"] - local_position["y"][reference_index],
        )

        axes[0, column].plot(attitude_time, roll, label="roll")
        axes[0, column].plot(attitude_time, pitch, label="pitch")
        axes[0, column].axhline(60, color="#C33", linestyle="--", linewidth=1)
        axes[0, column].axhline(-60, color="#C33", linestyle="--", linewidth=1)
        axes[0, column].set_ylabel("attitude (deg)")
        axes[0, column].legend()

        axes[1, column].plot(
            angular_time,
            np.abs(angular_velocity["xyz[2]"]),
            color="#8E5EA2",
        )
        axes[1, column].set_ylabel("|yaw rate| (rad/s)")

        axes[2, column].plot(position_time, local_position["vz"], color="#F28E2B")
        axes[2, column].axhline(3, color="#C33", linestyle="--", linewidth=1)
        axes[2, column].set_ylabel("down speed (m/s)")

        axes[3, column].plot(position_time, drift, color="#59A14F")
        axes[3, column].axhline(5, color="#C33", linestyle="--", linewidth=1)
        axes[3, column].set_ylabel("horizontal drift (m)")
        axes[3, column].set_xlabel("seconds after motor stop mask")

        for row in range(4):
            axes[row, column].axvline(0, color="#222", linewidth=1)
            axes[row, column].grid(alpha=0.2)
            axes[row, column].set_xlim(left=-1)

        verdict = "PASS" if result["safe_threshold_pass"] else "FAIL"
        axes[0, column].set_title(
            f"Motor 1, {result['mis_takeoff_alt']:g} m, delay 5 s — {verdict}"
        )

    fig.suptitle("Representative Low- and Mid-Altitude Responses", fontsize=14)
    _save(fig, output_dir / "representative_comparison.png")


def _handled_mask(path):
    if not path.is_absolute():
        path = REPOSITORY_ROOT / path

    ulog = ULog(
        str(path),
        message_name_filter_list=[
            "control_allocator_status",
            "failure_detector_status",
        ],
    )
    failure = _dataset(ulog, "failure_detector_status")
    allocator = _dataset(ulog, "control_allocator_status")
    injection_indices = np.flatnonzero(failure["motor_stop_mask"] != 0)

    if not injection_indices.size:
        return None

    injection_time_us = int(failure["timestamp"][int(injection_indices[0])])
    allocator_indices = np.flatnonzero(
        (allocator["timestamp"] >= injection_time_us)
        & (allocator["handled_motor_failure_mask"] != 0)
    )

    if not allocator_indices.size:
        return 0

    return int(allocator["handled_motor_failure_mask"][int(allocator_indices[0])])


def plot_baseline_comparison(results, baseline_results, output_dir):
    baseline_candidates = [
        result for result in baseline_results if result.get("status") == "ok"
    ]
    improved_candidates = [
        result for result in results
        if _condition_match(result, motor=1, height=2.5, delay=5.0)
    ]

    if not baseline_candidates or not improved_candidates:
        return

    baseline = baseline_candidates[0]
    improved = improved_candidates[0]
    metric_definitions = [
        ("land_delay_s", "Land response", "s", 0.5),
        ("max_abs_roll_deg", "Maximum |roll|", "deg", 60.0),
        ("max_abs_pitch_deg", "Maximum |pitch|", "deg", 60.0),
        ("max_down_speed_m_s", "Maximum down speed", "m/s", 3.0),
        ("max_horizontal_drift_m", "Horizontal drift", "m", 5.0),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(12, 7.2), constrained_layout=True)

    for axis, (field, title, unit, threshold) in zip(axes.ravel(), metric_definitions):
        values = [baseline.get(field), improved.get(field)]
        axis.bar(["pre-fix\nbaseline", "degraded\nlanding"], values, color=["#777", "#2878B5"])
        axis.axhline(threshold, color="#C33", linestyle="--", linewidth=1)
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.grid(axis="y", alpha=0.2)

        for index, value in enumerate(values):
            axis.text(index, value, f"{value:.2f}", ha="center", va="bottom", fontsize=9)

    mask_axis = axes.ravel()[-1]
    baseline_handled = _handled_mask(Path(baseline["path"]))
    improved_handled = _handled_mask(Path(improved["path"]))
    mask_axis.axis("off")
    mask_axis.text(
        0.5,
        0.7,
        "Failure-mask mapping",
        ha="center",
        fontsize=12,
        weight="bold",
    )
    mask_axis.text(
        0.5,
        0.48,
        f"Pre-fix: stopped 0x{baseline['motor_stop_mask']:X} → handled 0x{baseline_handled:X}",
        ha="center",
        fontsize=11,
        color="#A33",
    )
    mask_axis.text(
        0.5,
        0.30,
        f"Improved: stopped 0x{improved['motor_stop_mask']:X} → handled 0x{improved_handled:X}",
        ha="center",
        fontsize=11,
        color="#2878B5",
    )
    fig.suptitle("Pre-Fix Baseline vs. Low-Altitude Degraded Landing", fontsize=14)
    _save(fig, output_dir / "baseline_comparison.png")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--baseline-results", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    with args.results.open(encoding="utf-8") as stream:
        document = json.load(stream)

    results = document["results"]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plot_matrix(results, args.output_dir)
    plot_metrics(results, args.output_dir)
    plot_response_delays(results, args.output_dir)
    plot_data_flow(args.output_dir)
    plot_representatives(results, args.output_dir)

    if args.baseline_results:
        with args.baseline_results.open(encoding="utf-8") as stream:
            baseline_document = json.load(stream)

        plot_baseline_comparison(
            results,
            baseline_document["results"],
            args.output_dir,
        )

    for path in sorted(args.output_dir.glob("*.png")):
        print(path)


if __name__ == "__main__":
    main()
