#!/usr/bin/env python3
"""
Extract reproducible single-motor-failure metrics from PX4 ULog files.

The script intentionally derives the test condition from each ULog:

* motor number: first non-zero failure_detector_status.motor_stop_mask
* commanded height: MIS_TAKEOFF_ALT
* failsafe delay: COM_FAIL_ACT_T

This avoids relying on filenames or a transient test-runner summary.
"""

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]

TOPICS = [
    "actuator_motors",
    "control_allocator_status",
    "failsafe_flags",
    "failure_detector_status",
    "vehicle_angular_velocity",
    "vehicle_attitude",
    "vehicle_land_detected",
    "vehicle_local_position",
    "vehicle_status",
]

PARAMETERS = [
    "CA_FAILURE_MODE",
    "CA_ROTOR_COUNT",
    "COM_ACT_FAIL_ACT",
    "COM_FAIL_ACT_T",
    "FD_ACT_EN",
    "MC_AIRMODE",
    "MC_YAWRATE_MAX",
    "MIS_TAKEOFF_ALT",
    "MPC_LAND_ALT1",
    "MPC_LAND_ALT2",
    "MPC_LAND_SPEED",
    "MPC_Z_V_AUTO_DN",
    "SYS_FAILURE_EN",
]

NAVIGATION_STATE_AUTO_LAND = 18
ARMING_STATE_DISARMED = 1
ARM_DISARM_REASON_LANDING = 6

MAX_LAND_RESPONSE_S = 0.5
MAX_HANDLE_RESPONSE_S = 0.5
MAX_TILT_DEG = 60.0
MAX_DOWN_SPEED_M_S = 3.0
MAX_HORIZONTAL_DRIFT_M = 5.0


def _dataset(ulog, name):
    for dataset in ulog.data_list:
        if dataset.name == name and dataset.multi_id == 0:
            return dataset.data

    return None


def _first_timestamp(data, condition, after_us):
    if data is None:
        return None

    indices = np.flatnonzero((data["timestamp"] >= after_us) & condition)
    return int(data["timestamp"][indices[0]]) if indices.size else None


def _seconds_after(timestamp_us, reference_us):
    if timestamp_us is None:
        return None

    return (timestamp_us - reference_us) / 1e6


def _single_bit_index(mask):
    value = int(mask)

    if value == 0 or value & (value - 1):
        return None

    return value.bit_length()


def _nearest_index(timestamps, timestamp_us):
    return int(np.argmin(np.abs(timestamps.astype(np.int64) - int(timestamp_us))))


def _quaternion_to_roll_pitch_deg(attitude):
    w = attitude["q[0]"]
    x = attitude["q[1]"]
    y = attitude["q[2]"]
    z = attitude["q[3]"]

    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)

    return np.degrees(roll), np.degrees(pitch)


def _finite_float(value):
    if value is None:
        return None

    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def _json_value(value):
    if isinstance(value, (np.bool_, bool)):
        return bool(value)

    if isinstance(value, (np.integer,)):
        return int(value)

    if isinstance(value, (np.floating, float)):
        return _finite_float(value)

    return value


def _parameter_at(ulog, name, timestamp_us):
    value = ulog.initial_parameters.get(name)

    for changed_timestamp, changed_name, changed_value in ulog.changed_parameters:
        if changed_timestamp > timestamp_us:
            break

        if changed_name == name:
            value = changed_value

    return _json_value(value)


def _sha256(path):
    digest = hashlib.sha256()

    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)

    return digest.hexdigest()


def analyze_log(path, calculate_hash):
    ulog = ULog(str(path), message_name_filter_list=TOPICS)

    try:
        display_path = str(path.resolve().relative_to(REPOSITORY_ROOT))

    except ValueError:
        display_path = str(path)

    result = {
        "file": path.name,
        "path": display_path,
        "sha256": _sha256(path) if calculate_hash else None,
        "size_bytes": path.stat().st_size,
        "status": "ok",
        "software_sha": ulog.msg_info_dict.get("ver_sw"),
        "software_branch": ulog.msg_info_dict.get("ver_sw_branch", "").strip(),
        "hardware": ulog.msg_info_dict.get("ver_hw"),
        "log_duration_s": (ulog.last_timestamp - ulog.start_timestamp) / 1e6,
    }

    failure = _dataset(ulog, "failure_detector_status")

    if failure is None:
        result["status"] = "missing_failure_detector_status"
        return result

    injected = np.flatnonzero(failure["motor_stop_mask"] != 0)

    if not injected.size:
        result["status"] = "no_motor_stop_injection"
        return result

    injection_index = int(injected[0])
    injection_time_us = int(failure["timestamp"][injection_index])
    motor_stop_mask = int(failure["motor_stop_mask"][injection_index])
    motor_number = _single_bit_index(motor_stop_mask)
    result.update({
        "injection_time_us": injection_time_us,
        "motor_stop_mask": motor_stop_mask,
        "motor_number": motor_number,
    })

    for parameter in PARAMETERS:
        result[parameter.lower()] = _parameter_at(ulog, parameter, injection_time_us)

    vehicle_status = _dataset(ulog, "vehicle_status")
    allocator = _dataset(ulog, "control_allocator_status")
    failsafe = _dataset(ulog, "failsafe_flags")
    land_detected = _dataset(ulog, "vehicle_land_detected")

    handled_time_us = None

    if allocator is not None:
        handled_time_us = _first_timestamp(
            allocator,
            (allocator["handled_motor_failure_mask"] & motor_stop_mask) == motor_stop_mask,
            injection_time_us,
        )

    motor_failsafe_time_us = None

    if failsafe is not None:
        motor_failsafe_time_us = _first_timestamp(
            failsafe,
            failsafe["fd_motor_failure"].astype(bool),
            injection_time_us,
        )

    land_time_us = None
    disarm_time_us = None
    disarm_reason = None

    if vehicle_status is not None:
        land_time_us = _first_timestamp(
            vehicle_status,
            vehicle_status["nav_state"] == NAVIGATION_STATE_AUTO_LAND,
            injection_time_us,
        )
        disarm_indices = np.flatnonzero(
            (vehicle_status["timestamp"] >= injection_time_us)
            & (vehicle_status["arming_state"] == ARMING_STATE_DISARMED)
        )

        if disarm_indices.size:
            disarm_index = int(disarm_indices[0])
            disarm_time_us = int(vehicle_status["timestamp"][disarm_index])
            disarm_reason = int(vehicle_status["latest_disarming_reason"][disarm_index])

    landed_time_us = None

    if land_detected is not None:
        landed_time_us = _first_timestamp(
            land_detected,
            land_detected["landed"].astype(bool),
            injection_time_us,
        )

    result.update({
        "motor_failsafe_delay_s": _seconds_after(motor_failsafe_time_us, injection_time_us),
        "handled_delay_s": _seconds_after(handled_time_us, injection_time_us),
        "land_delay_s": _seconds_after(land_time_us, injection_time_us),
        "landed_delay_s": _seconds_after(landed_time_us, injection_time_us),
        "disarm_delay_s": _seconds_after(disarm_time_us, injection_time_us),
        "disarm_reason": disarm_reason,
        "natural_disarm": disarm_reason == ARM_DISARM_REASON_LANDING,
        "landed": landed_time_us is not None,
    })

    evaluation_end_us = disarm_time_us if disarm_time_us is not None else ulog.last_timestamp

    local_position = _dataset(ulog, "vehicle_local_position")

    if local_position is not None:
        position_mask = (
            (local_position["timestamp"] >= injection_time_us)
            & (local_position["timestamp"] <= evaluation_end_us)
        )
        position_indices = np.flatnonzero(position_mask)

        if position_indices.size:
            reference_index = _nearest_index(local_position["timestamp"], injection_time_us)
            x0 = float(local_position["x"][reference_index])
            y0 = float(local_position["y"][reference_index])
            z0 = float(local_position["z"][reference_index])
            x = local_position["x"][position_indices]
            y = local_position["y"][position_indices]
            vz = local_position["vz"][position_indices]
            result["failure_altitude_m"] = max(0.0, -z0)
            result["max_horizontal_drift_m"] = float(np.nanmax(np.hypot(x - x0, y - y0)))
            result["max_down_speed_m_s"] = float(np.nanmax(np.maximum(vz, 0.0)))

    attitude = _dataset(ulog, "vehicle_attitude")

    if attitude is not None:
        attitude_mask = (
            (attitude["timestamp"] >= injection_time_us)
            & (attitude["timestamp"] <= evaluation_end_us)
        )
        attitude_indices = np.flatnonzero(attitude_mask)

        if attitude_indices.size:
            roll_deg, pitch_deg = _quaternion_to_roll_pitch_deg(attitude)
            result["max_abs_roll_deg"] = float(np.nanmax(np.abs(roll_deg[attitude_indices])))
            result["max_abs_pitch_deg"] = float(np.nanmax(np.abs(pitch_deg[attitude_indices])))

    angular_velocity = _dataset(ulog, "vehicle_angular_velocity")

    if angular_velocity is not None:
        angular_velocity_mask = (
            (angular_velocity["timestamp"] >= injection_time_us)
            & (angular_velocity["timestamp"] <= evaluation_end_us)
        )
        angular_velocity_indices = np.flatnonzero(angular_velocity_mask)

        if angular_velocity_indices.size:
            result["max_abs_yaw_rate_rad_s"] = float(
                np.nanmax(np.abs(angular_velocity["xyz[2]"][angular_velocity_indices]))
            )

    allocator_finite = False
    handled_mask_exact = False
    torque_achieved_fraction = None
    thrust_achieved_fraction = None

    if allocator is not None:
        allocator_mask = (
            (allocator["timestamp"] >= (handled_time_us or injection_time_us))
            & (allocator["timestamp"] <= evaluation_end_us)
        )
        allocator_indices = np.flatnonzero(allocator_mask)

        if allocator_indices.size:
            fields = [
                "unallocated_torque[0]",
                "unallocated_torque[1]",
                "unallocated_torque[2]",
                "unallocated_thrust[0]",
                "unallocated_thrust[1]",
                "unallocated_thrust[2]",
            ]
            allocator_finite = all(
                np.all(np.isfinite(allocator[field][allocator_indices])) for field in fields
            )
            handled_values = allocator["handled_motor_failure_mask"][allocator_indices]
            handled_mask_exact = bool(np.all(handled_values == motor_stop_mask))
            torque_achieved_fraction = float(
                np.mean(allocator["torque_setpoint_achieved"][allocator_indices])
            )
            thrust_achieved_fraction = float(
                np.mean(allocator["thrust_setpoint_achieved"][allocator_indices])
            )

    result.update({
        "allocator_finite": allocator_finite,
        "handled_mask_exact": handled_mask_exact,
        "torque_achieved_fraction": torque_achieved_fraction,
        "thrust_achieved_fraction": thrust_achieved_fraction,
    })

    actuator_motors = _dataset(ulog, "actuator_motors")
    stopped_output_valid = False
    stopped_output_valid_fraction = None
    remaining_outputs_finite_fraction = None

    if actuator_motors is not None and motor_number is not None:
        actuator_mask = (
            (actuator_motors["timestamp"] >= injection_time_us)
            & (actuator_motors["timestamp"] <= evaluation_end_us)
        )
        actuator_indices = np.flatnonzero(actuator_mask)

        if actuator_indices.size:
            failed_index = motor_number - 1
            failed_output = actuator_motors[f"control[{failed_index}]"][actuator_indices]
            stopped_samples = (~np.isfinite(failed_output)) | (failed_output <= 1e-3)
            stopped_output_valid_fraction = float(np.mean(stopped_samples))
            stopped_output_valid = stopped_output_valid_fraction >= 0.95

            remaining_outputs = []

            for motor_index in range(4):
                if motor_index != failed_index:
                    remaining_outputs.append(
                        actuator_motors[f"control[{motor_index}]"][actuator_indices]
                    )

            remaining_outputs_finite_fraction = float(
                np.mean(np.isfinite(np.concatenate(remaining_outputs)))
            )

    result.update({
        "stopped_output_valid": stopped_output_valid,
        "stopped_output_valid_fraction": stopped_output_valid_fraction,
        "remaining_outputs_finite_fraction": remaining_outputs_finite_fraction,
    })

    required_metrics = [
        "max_abs_roll_deg",
        "max_abs_pitch_deg",
        "max_down_speed_m_s",
        "max_horizontal_drift_m",
    ]
    metrics_present = all(result.get(name) is not None for name in required_metrics)
    response_valid = (
        result["handled_delay_s"] is not None
        and result["handled_delay_s"] <= MAX_HANDLE_RESPONSE_S
        and result["land_delay_s"] is not None
        and result["land_delay_s"] <= MAX_LAND_RESPONSE_S
    )
    dynamics_valid = (
        metrics_present
        and result["max_abs_roll_deg"] < MAX_TILT_DEG
        and result["max_abs_pitch_deg"] < MAX_TILT_DEG
        and result["max_down_speed_m_s"] < MAX_DOWN_SPEED_M_S
        and result["max_horizontal_drift_m"] < MAX_HORIZONTAL_DRIFT_M
    )
    result["safe_threshold_pass"] = bool(
        response_valid
        and dynamics_valid
        and result["landed"]
        and allocator_finite
        and handled_mask_exact
        and stopped_output_valid
    )

    return result


def _collect_paths(arguments):
    collected = []

    for raw_path in arguments:
        path = Path(raw_path)

        if path.is_dir():
            collected.extend(sorted(path.rglob("*.ulg")))

        elif path.is_file() and path.suffix == ".ulg":
            collected.append(path)

        else:
            print(f"warning: ignoring missing or unsupported path: {path}", file=sys.stderr)

    return sorted(set(path.resolve() for path in collected))


def _write_csv(path, results):
    fields = []

    for result in results:
        for key in result:
            if key not in fields:
                fields.append(key)

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)


def _summary(results):
    valid = [result for result in results if result.get("status") == "ok"]
    safe = [result for result in valid if result.get("safe_threshold_pass")]
    natural = [result for result in valid if result.get("natural_disarm")]
    return {
        "logs_total": len(results),
        "logs_with_motor_injection": len(valid),
        "safe_threshold_pass_count": len(safe),
        "natural_disarm_count": len(natural),
        "criteria": {
            "max_handle_response_s": MAX_HANDLE_RESPONSE_S,
            "max_land_response_s": MAX_LAND_RESPONSE_S,
            "max_tilt_deg": MAX_TILT_DEG,
            "max_down_speed_m_s": MAX_DOWN_SPEED_M_S,
            "max_horizontal_drift_m": MAX_HORIZONTAL_DRIFT_M,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", help="ULog files or directories")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--label", default="unclassified")
    parser.add_argument("--skip-hash", action="store_true")
    args = parser.parse_args()

    log_paths = _collect_paths(args.logs)

    if not log_paths:
        parser.error("no ULog files found")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = []

    for index, log_path in enumerate(log_paths, start=1):
        print(f"[{index}/{len(log_paths)}] {log_path}")
        result = analyze_log(log_path, not args.skip_hash)
        result["group"] = args.label
        results.append({key: _json_value(value) for key, value in result.items()})

    document = {
        "summary": _summary(results),
        "results": results,
    }
    json_path = args.output_dir / "results.json"
    csv_path = args.output_dir / "results.csv"
    manifest_path = args.output_dir / "logs_manifest.csv"

    with json_path.open("w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")

    _write_csv(csv_path, results)
    manifest_fields = [
        "file",
        "sha256",
        "size_bytes",
        "software_sha",
        "software_branch",
        "motor_number",
        "mis_takeoff_alt",
        "com_fail_act_t",
        "group",
    ]

    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=manifest_fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(results)

    print(json.dumps(document["summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
