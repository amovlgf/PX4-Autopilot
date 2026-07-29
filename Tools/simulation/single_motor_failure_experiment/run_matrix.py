#!/usr/bin/env python3
"""
Run the quadrotor single-motor-failure SITL matrix through MAVLink.

The script expects an already-running visible PX4 jMAVSim session. QGC may
remain connected for observation. Vehicle commands and parameter changes use
the regular MAVLink command and parameter protocols.
"""

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
BENCH_TOOLS = REPOSITORY_ROOT / "Tools" / "bench_test"
sys.path.insert(0, str(BENCH_TOOLS))

from px4bench import connect, send_heartbeat  # noqa: E402
from px4bench.params import (  # noqa: E402
    drain_param_values,
    int32_to_param_float,
    param_float_to_int32,
    param_id_str,
)
from pymavlink import mavutil  # noqa: E402


DEFAULT_CONNECTION = "udpin:0.0.0.0:14540"
DEFAULT_LOG_ROOT = REPOSITORY_ROOT / "build" / "px4_sitl_default" / "rootfs" / "log"
DEFAULT_LOG_LIST = Path(__file__).resolve().parent / "matrix_logs.txt"
DEFAULT_RUN_SUMMARY = (
    Path(__file__).resolve().parent / "generated" / "matrix" / "run_summary.json"
)

STATIC_PARAMETERS = {
    "SYS_FAILURE_EN": 1,
    "FD_ACT_EN": 0,
    "MC_AIRMODE": 1,
    "CA_FAILURE_MODE": 1,
    "COM_ACT_FAIL_ACT": 2,
}

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_INJECT_FAILURE = 420
FAILURE_UNIT_SYSTEM_MOTOR = 101
FAILURE_TYPE_OK = 0
FAILURE_TYPE_OFF = 1
MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
MAVLINK_MSG_ID_EXTENDED_SYS_STATE = 245
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_LANDED_STATE_ON_GROUND = 1
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5


class MatrixError(RuntimeError):
    pass


def connect_vehicle(connection, timeout):
    mav = connect(connection, timeout=timeout)
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        heartbeat = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if (
            heartbeat is not None
            and heartbeat.autopilot
            != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            mav.target_system = heartbeat.get_srcSystem()
            mav.target_component = heartbeat.get_srcComponent()
            return mav

    mav.close()
    raise MatrixError("no PX4 autopilot heartbeat received on {}".format(connection))


def parse_csv_numbers(raw, cast):
    values = []

    for item in raw.split(","):
        item = item.strip()

        if item:
            values.append(cast(item))

    if not values:
        raise argparse.ArgumentTypeError("at least one value is required")

    return values


def set_parameter(mav, name, value, integer):
    expected = float(value)
    parameter_type = (
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
        if integer
        else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    wire_value = int32_to_param_float(int(value)) if integer else expected
    drain_param_values(mav)
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        name.encode("ascii"),
        wire_value,
        parameter_type,
    )
    deadline = time.monotonic() + 5
    observed = []

    while time.monotonic() < deadline:
        message = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)

        if message is None or param_id_str(message.param_id) != name:
            continue

        current = (
            float(param_float_to_int32(message.param_value))
            if integer
            else float(message.param_value)
        )
        observed.append(current)
        tolerance = max(1e-4, abs(expected) * 1e-4)

        if math.isclose(current, expected, abs_tol=tolerance):
            return

    raise MatrixError(
        "parameter verification failed for {}: expected {}, observed {}".format(
            name, expected, observed
        )
    )


def request_stream(mav, message_id, rate_hz):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        1e6 / rate_hz,
        0,
        0,
        0,
        0,
        0,
    )


def send_command(mav, command, params=None, timeout=10, accepted_results=None):
    params = list(params or [])
    params.extend([float("nan")] * (7 - len(params)))
    accepted_results = accepted_results or (
        MAV_RESULT_ACCEPTED,
        MAV_RESULT_IN_PROGRESS,
    )

    while mav.recv_match(type="COMMAND_ACK", blocking=False) is not None:
        pass

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        command,
        0,
        *params[:7],
    )
    deadline = time.monotonic() + timeout
    observed = []

    while time.monotonic() < deadline:
        message = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)

        if message is None or message.command != command:
            continue

        observed.append(int(message.result))

        if message.result in accepted_results:
            return int(message.result)

        raise MatrixError(
            "MAVLink command {} rejected with result {}".format(
                command, message.result
            )
        )

    raise MatrixError(
        "MAVLink command {} was not acknowledged; observed {}".format(
            command, observed
        )
    )


def inject_motor_failure(mav, motor, failure_type):
    send_command(
        mav,
        MAV_CMD_INJECT_FAILURE,
        [FAILURE_UNIT_SYSTEM_MOTOR, failure_type, motor, 0, 0, 0, 0],
    )


def arm_vehicle(mav, timeout, force):
    deadline = time.monotonic() + timeout
    arm_magic = 21196 if force else 0

    while time.monotonic() < deadline:
        result = send_command(
            mav,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1, arm_magic, 0, 0, 0, 0, 0],
            timeout=min(5, max(1, deadline - time.monotonic())),
            accepted_results=(
                MAV_RESULT_ACCEPTED,
                MAV_RESULT_IN_PROGRESS,
                1,  # MAV_RESULT_TEMPORARILY_REJECTED
            ),
        )

        if result != 1 and wait_for_armed_state(mav, True, 3):
            return True

        time.sleep(1)

    return False


def heartbeat_armed(message):
    return bool(message.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)


def wait_for_armed_state(mav, armed, timeout):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        message = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if message is None or message.get_srcSystem() != mav.target_system:
            send_heartbeat(mav)
            continue

        if heartbeat_armed(message) == armed:
            return True

    return False


def wait_for_stable_hover(mav, altitude_m, timeout, stable_time_s):
    deadline = time.monotonic() + timeout
    stable_since = None
    last_status = None

    while time.monotonic() < deadline:
        message = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)

        if message is None:
            send_heartbeat(mav)
            continue

        height = max(0.0, -float(message.z))
        speed = math.sqrt(message.vx ** 2 + message.vy ** 2 + message.vz ** 2)
        last_status = {"height_m": height, "speed_m_s": speed}
        within_altitude = abs(height - altitude_m) <= 0.6
        nearly_stationary = speed <= 1.0

        if within_altitude and nearly_stationary:
            stable_since = stable_since or time.monotonic()

            if time.monotonic() - stable_since >= stable_time_s:
                return last_status

        else:
            stable_since = None

    raise MatrixError(
        "hover did not stabilize at {} m; last telemetry: {}".format(
            altitude_m, last_status
        )
    )


def wait_for_touchdown_and_disarm(mav, timeout):
    start = time.monotonic()
    deadline = start + timeout
    touchdown_s = None
    disarm_s = None

    while time.monotonic() < deadline:
        message = mav.recv_match(
            type=["EXTENDED_SYS_STATE", "HEARTBEAT"],
            blocking=True,
            timeout=1,
        )

        if message is None:
            send_heartbeat(mav)
            continue

        if message.get_type() == "EXTENDED_SYS_STATE":
            if (
                message.landed_state == MAV_LANDED_STATE_ON_GROUND
                and touchdown_s is None
            ):
                touchdown_s = time.monotonic() - start

        elif (
            message.get_srcSystem() == mav.target_system
            and not heartbeat_armed(message)
        ):
            disarm_s = time.monotonic() - start
            break

    return touchdown_s, disarm_s


def all_logs(log_root):
    return set(log_root.rglob("*.ulg")) if log_root.exists() else set()


def log_snapshot(log_root):
    return {
        path: (path.stat().st_size, path.stat().st_mtime_ns)
        for path in all_logs(log_root)
    }


def wait_for_case_log(log_root, previous_snapshot, timeout=8):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        current_snapshot = log_snapshot(log_root)
        candidates = [
            path
            for path, metadata in current_snapshot.items()
            if path not in previous_snapshot
            or metadata != previous_snapshot[path]
        ]

        if candidates:
            return max(candidates, key=lambda path: path.stat().st_mtime_ns)

        time.sleep(0.25)

    return None


def relative_log_path(path):
    try:
        return str(path.resolve().relative_to(REPOSITORY_ROOT))
    except ValueError:
        return str(path.resolve())


def write_progress(results, summary_path, log_list_path):
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "results": results,
    }
    summary_path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    log_paths = [
        result["log_path"]
        for result in results
        if result.get("log_path")
    ]
    log_list_path.write_text(
        "# Generated by run_matrix.py; paths are relative to the repository root.\n"
        + "\n".join(log_paths)
        + ("\n" if log_paths else ""),
        encoding="utf-8",
    )


def run_case(mav, case, args):
    result = dict(case)
    result["started_at"] = datetime.now(timezone.utc).isoformat()
    before_logs = log_snapshot(args.log_root)
    motor = case["motor"]

    set_parameter(mav, "MIS_TAKEOFF_ALT", case["height_m"], integer=False)
    set_parameter(mav, "COM_FAIL_ACT_T", case["fail_delay_s"], integer=False)
    inject_motor_failure(mav, motor, FAILURE_TYPE_OK)
    send_command(mav, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    if not arm_vehicle(mav, args.arm_timeout, args.force_arm):
        raise MatrixError("vehicle did not arm")

    hover = wait_for_stable_hover(
        mav,
        case["height_m"],
        args.takeoff_timeout,
        args.stable_time,
    )
    result["injection_hover"] = hover
    result["injected_at"] = datetime.now(timezone.utc).isoformat()
    inject_motor_failure(mav, motor, FAILURE_TYPE_OFF)
    touchdown_s, disarm_s = wait_for_touchdown_and_disarm(
        mav, args.landing_timeout
    )
    result["touchdown_observed_s"] = touchdown_s
    result["natural_disarm_observed_s"] = disarm_s

    if disarm_s is None:
        result["forced_disarm"] = True
        send_command(
            mav,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0, 21196, 0, 0, 0, 0, 0],
        )
        wait_for_armed_state(mav, False, args.disarm_timeout)
    else:
        result["forced_disarm"] = False

    inject_motor_failure(mav, motor, FAILURE_TYPE_OK)
    time.sleep(args.reset_wait)
    log_path = wait_for_case_log(args.log_root, before_logs)
    result["log_path"] = relative_log_path(log_path) if log_path else None
    result["status"] = "complete" if log_path else "missing_log"
    result["finished_at"] = datetime.now(timezone.utc).isoformat()
    return result


def build_cases(args):
    return [
        {
            "motor": motor,
            "height_m": height,
            "fail_delay_s": delay,
        }
        for delay in args.delays
        for height in args.heights
        for motor in args.motors
    ]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument(
        "--motors",
        type=lambda value: parse_csv_numbers(value, int),
        default=[1, 2, 3, 4],
        help="comma-separated motor instances",
    )
    parser.add_argument(
        "--heights",
        type=lambda value: parse_csv_numbers(value, float),
        default=[2.5, 10.0, 20.0],
        help="comma-separated takeoff heights in metres",
    )
    parser.add_argument(
        "--delays",
        type=lambda value: parse_csv_numbers(value, float),
        default=[0.0, 5.0],
        help="comma-separated COM_FAIL_ACT_T values",
    )
    parser.add_argument("--log-root", type=Path, default=DEFAULT_LOG_ROOT)
    parser.add_argument("--log-list", type=Path, default=DEFAULT_LOG_LIST)
    parser.add_argument("--run-summary", type=Path, default=DEFAULT_RUN_SUMMARY)
    parser.add_argument("--arm-timeout", type=float, default=15)
    parser.add_argument("--takeoff-timeout", type=float, default=60)
    parser.add_argument("--landing-timeout", type=float, default=90)
    parser.add_argument("--disarm-timeout", type=float, default=15)
    parser.add_argument("--stable-time", type=float, default=2)
    parser.add_argument("--reset-wait", type=float, default=2)
    parser.add_argument(
        "--force-arm",
        action="store_true",
        help="use PX4's SITL force-arm magic after switching to Takeoff",
    )
    parser.add_argument("--fail-fast", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    cases = build_cases(args)

    if any(motor not in (1, 2, 3, 4) for motor in args.motors):
        raise SystemExit("motor instances must be in the range 1..4")

    print(json.dumps({"connection": args.connection, "cases": cases}, indent=2))

    if args.dry_run:
        return 0

    mav = connect_vehicle(args.connection, timeout=30)
    request_stream(mav, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
    request_stream(mav, MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5)

    results = []

    try:
        for name, value in STATIC_PARAMETERS.items():
            set_parameter(mav, name, value, integer=True)

        set_parameter(mav, "CA_ROTOR_COUNT", 4, integer=True)

        for index, case in enumerate(cases, start=1):
            print(
                "[{}/{}] motor={} height={}m delay={}s".format(
                    index,
                    len(cases),
                    case["motor"],
                    case["height_m"],
                    case["fail_delay_s"],
                ),
                flush=True,
            )

            try:
                result = run_case(mav, case, args)
            except Exception as error:
                result = dict(case)
                result.update(
                    {
                        "status": "runner_error",
                        "error": str(error),
                        "finished_at": datetime.now(timezone.utc).isoformat(),
                    }
                )

                try:
                    send_command(
                        mav,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        [0, 21196, 0, 0, 0, 0, 0],
                    )
                    inject_motor_failure(mav, case["motor"], FAILURE_TYPE_OK)
                except Exception as cleanup_error:
                    result["cleanup_error"] = str(cleanup_error)

            results.append(result)
            write_progress(results, args.run_summary, args.log_list)
            print(json.dumps(result, indent=2, sort_keys=True), flush=True)

            if result["status"] != "complete" and args.fail_fast:
                break

    finally:
        mav.close()

    complete = sum(result.get("status") == "complete" for result in results)
    print("matrix logs recorded: {}/{}".format(complete, len(cases)))
    return 0 if complete == len(cases) else 1


if __name__ == "__main__":
    sys.exit(main())
