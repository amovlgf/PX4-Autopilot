#!/usr/bin/env python3
"""
Run every single-motor-failure case in a fresh visible jMAVSim session.

Three-motor emergency landings can leave the simulated vehicle tilted on the
ground. Reusing that world makes later arming checks dependent on the previous
impact. This wrapper keeps QGroundControl running, but restarts PX4 and the
visible jMAVSim GUI before each case and delegates the flight to run_matrix.py.
"""

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_DIR = Path(__file__).resolve().parent
RUN_MATRIX = SCRIPT_DIR / "run_matrix.py"
PX4_BINARY = (
    REPOSITORY_ROOT / "build" / "px4_sitl_default" / "bin" / "px4"
).resolve()
PX4_COMMANDER = PX4_BINARY.with_name("px4-commander")
DEFAULT_LOG_LIST = SCRIPT_DIR / "matrix_logs.txt"
DEFAULT_RUN_SUMMARY = SCRIPT_DIR / "generated" / "matrix" / "run_summary.json"
DESKTOP_ENVIRONMENT_KEYS = (
    "DISPLAY",
    "WAYLAND_DISPLAY",
    "XAUTHORITY",
    "DBUS_SESSION_BUS_ADDRESS",
    "XDG_RUNTIME_DIR",
)


class IsolatedMatrixError(RuntimeError):
    pass


def parse_csv_numbers(raw, cast):
    values = []

    for item in raw.split(","):
        item = item.strip()

        if item:
            values.append(cast(item))

    if not values:
        raise argparse.ArgumentTypeError("at least one value is required")

    return values


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


def read_proc_path(pid, entry):
    try:
        return (Path("/proc") / str(pid) / entry).resolve()
    except (FileNotFoundError, PermissionError, OSError):
        return None


def read_proc_cmdline(pid):
    try:
        data = (Path("/proc") / str(pid) / "cmdline").read_bytes()
        return [part.decode(errors="replace") for part in data.split(b"\0") if part]
    except (FileNotFoundError, PermissionError, OSError):
        return []


def read_proc_environment(pid):
    try:
        data = (Path("/proc") / str(pid) / "environ").read_bytes()
        entries = [
            part.decode(errors="replace") for part in data.split(b"\0") if part
        ]
        return dict(entry.split("=", 1) for entry in entries if "=" in entry)
    except (FileNotFoundError, PermissionError, OSError):
        return {}


def desktop_environment():
    environment = os.environ.copy()

    if environment.get("DISPLAY") or environment.get("WAYLAND_DISPLAY"):
        return environment

    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue

        command = read_proc_cmdline(int(entry.name))

        if not command or not command[0].endswith("/QGroundControl"):
            continue

        process_environment = read_proc_environment(int(entry.name))

        for key in DESKTOP_ENVIRONMENT_KEYS:
            if process_environment.get(key):
                environment[key] = process_environment[key]

        break

    if not environment.get("DISPLAY") and not environment.get("WAYLAND_DISPLAY"):
        raise IsolatedMatrixError(
            "no desktop display environment found; start QGroundControl first"
        )

    return environment


def belongs_to_repository(pid):
    cwd = read_proc_path(pid, "cwd")

    if cwd is None:
        return False

    try:
        cwd.relative_to(REPOSITORY_ROOT)
        return True
    except ValueError:
        return False


def simulation_processes():
    processes = {"px4": [], "jmavsim": []}

    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue

        pid = int(entry.name)

        if not belongs_to_repository(pid):
            continue

        executable = read_proc_path(pid, "exe")
        command = read_proc_cmdline(pid)

        if executable == PX4_BINARY:
            processes["px4"].append(pid)

        elif any(part.endswith("jmavsim_run.jar") for part in command):
            processes["jmavsim"].append(pid)

    return processes


def signal_processes(processes, sig):
    for process_type in ("px4", "jmavsim"):
        for pid in processes[process_type]:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass


def stop_simulation(timeout):
    processes = simulation_processes()

    if not processes["px4"] and not processes["jmavsim"]:
        return

    print(
        "stopping PX4 {} and jMAVSim {}".format(
            processes["px4"], processes["jmavsim"]
        ),
        flush=True,
    )
    signal_processes(processes, signal.SIGINT)
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        remaining = simulation_processes()

        if not remaining["px4"] and not remaining["jmavsim"]:
            return

        time.sleep(0.25)

    remaining = simulation_processes()
    signal_processes(remaining, signal.SIGTERM)
    deadline = time.monotonic() + min(5, timeout)

    while time.monotonic() < deadline:
        remaining = simulation_processes()

        if not remaining["px4"] and not remaining["jmavsim"]:
            return

        time.sleep(0.25)

    raise IsolatedMatrixError(
        "simulation processes did not stop: {}".format(simulation_processes())
    )


def terminal_command(title):
    make_command = "cd {} && unset HEADLESS && exec make px4_sitl_default jmavsim".format(
        shlex.quote(str(REPOSITORY_ROOT))
    )
    return [
        "gnome-terminal",
        "--title={}".format(title),
        "--",
        "bash",
        "--noprofile",
        "--norc",
        "-lc",
        make_command,
    ]


def start_simulation(case_index, timeout):
    title = "PX4 Matrix Case {:02d}".format(case_index)
    command = terminal_command(title)
    print("launching visible session: {}".format(shlex.join(command)), flush=True)
    result = subprocess.run(command, check=False, env=desktop_environment())

    if result.returncode != 0:
        raise IsolatedMatrixError(
            "gnome-terminal launch failed with status {}".format(result.returncode)
        )

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        processes = simulation_processes()

        if processes["px4"] and processes["jmavsim"]:
            print(
                "started PX4 {} and jMAVSim {}".format(
                    processes["px4"], processes["jmavsim"]
                ),
                flush=True,
            )
            return

        time.sleep(0.5)

    raise IsolatedMatrixError("PX4 and visible jMAVSim did not start")


def wait_for_preflight(timeout):
    deadline = time.monotonic() + timeout
    last_output = ""

    while time.monotonic() < deadline:
        if PX4_COMMANDER.exists():
            result = subprocess.run(
                [str(PX4_COMMANDER), "check"],
                check=False,
                capture_output=True,
                text=True,
            )
            last_output = result.stdout + result.stderr

            if "Preflight check: OK" in last_output:
                print("PX4 preflight check: OK", flush=True)
                return

        time.sleep(1)

    raise IsolatedMatrixError(
        "PX4 preflight did not become ready; last output: {}".format(
            last_output.strip()
        )
    )


def child_command(args, case, log_list, run_summary):
    command = [
        sys.executable,
        str(RUN_MATRIX),
        "--connection",
        args.connection,
        "--motors",
        str(case["motor"]),
        "--heights",
        str(case["height_m"]),
        "--delays",
        str(case["fail_delay_s"]),
        "--log-list",
        str(log_list),
        "--run-summary",
        str(run_summary),
        "--arm-timeout",
        str(args.arm_timeout),
        "--takeoff-timeout",
        str(args.takeoff_timeout),
        "--landing-timeout",
        str(args.landing_timeout),
        "--disarm-timeout",
        str(args.disarm_timeout),
        "--stable-time",
        str(args.stable_time),
        "--reset-wait",
        str(args.reset_wait),
        "--fail-fast",
    ]

    if args.force_arm:
        command.append("--force-arm")

    return command


def load_child_result(summary_path):
    try:
        document = json.loads(summary_path.read_text(encoding="utf-8"))
        results = document["results"]

        if len(results) != 1:
            raise ValueError("expected one result, found {}".format(len(results)))

        return results[0]
    except (FileNotFoundError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise IsolatedMatrixError(
            "unable to read single-case summary {}: {}".format(summary_path, error)
        ) from error


def write_progress(results, summary_path, log_list_path):
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "session_strategy": "fresh-visible-jmavsim-per-case",
        "results": results,
    }
    summary_path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    log_paths = [
        result["log_path"] for result in results if result.get("log_path")
    ]
    log_list_path.write_text(
        "# Generated by run_isolated_matrix.py; paths are relative to the repository root.\n"
        + "\n".join(log_paths)
        + ("\n" if log_paths else ""),
        encoding="utf-8",
    )


def load_resume_results(summary_path, cases):
    if not summary_path.exists():
        return []

    try:
        document = json.loads(summary_path.read_text(encoding="utf-8"))

        if document.get("session_strategy") != "fresh-visible-jmavsim-per-case":
            raise ValueError("summary was not generated by the isolated runner")

        previous_results = document["results"]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise IsolatedMatrixError(
            "unable to resume from {}: {}".format(summary_path, error)
        ) from error

    completed_results = []

    for index, result in enumerate(previous_results):
        if index >= len(cases):
            raise IsolatedMatrixError(
                "resume summary has more cases than the requested matrix"
            )

        expected = cases[index]
        observed = {
            "motor": result.get("motor"),
            "height_m": result.get("height_m"),
            "fail_delay_s": result.get("fail_delay_s"),
        }

        if observed != expected:
            raise IsolatedMatrixError(
                "resume case {} does not match: expected {}, observed {}".format(
                    index + 1, expected, observed
                )
            )

        if result.get("status") != "complete":
            break

        completed_results.append(result)

    return completed_results


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udpin:0.0.0.0:14540")
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
    parser.add_argument("--log-list", type=Path, default=DEFAULT_LOG_LIST)
    parser.add_argument("--run-summary", type=Path, default=DEFAULT_RUN_SUMMARY)
    parser.add_argument("--startup-timeout", type=float, default=120)
    parser.add_argument("--preflight-timeout", type=float, default=60)
    parser.add_argument("--stop-timeout", type=float, default=10)
    parser.add_argument("--arm-timeout", type=float, default=15)
    parser.add_argument("--takeoff-timeout", type=float, default=60)
    parser.add_argument("--landing-timeout", type=float, default=90)
    parser.add_argument("--disarm-timeout", type=float, default=15)
    parser.add_argument("--stable-time", type=float, default=2)
    parser.add_argument("--reset-wait", type=float, default=2)
    parser.add_argument("--force-arm", action="store_true")
    parser.add_argument("--keep-last-session", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--fail-fast", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    cases = build_cases(args)

    if any(motor not in (1, 2, 3, 4) for motor in args.motors):
        raise SystemExit("motor instances must be in the range 1..4")

    preview = {
        "cases": cases,
        "session_strategy": "fresh-visible-jmavsim-per-case",
        "terminal_command": terminal_command("PX4 Matrix Case 01"),
    }
    print(json.dumps(preview, indent=2), flush=True)

    if args.dry_run:
        return 0

    results = (
        load_resume_results(args.run_summary, cases) if args.resume else []
    )

    if results:
        print("resuming after {} completed cases".format(len(results)), flush=True)

    with tempfile.TemporaryDirectory(prefix="px4-three-motor-matrix-") as temp_dir:
        temp_path = Path(temp_dir)

        for index, case in enumerate(cases, start=1):
            if index <= len(results):
                continue

            print(
                "[{}/{}] isolated motor={} height={}m delay={}s".format(
                    index,
                    len(cases),
                    case["motor"],
                    case["height_m"],
                    case["fail_delay_s"],
                ),
                flush=True,
            )
            result = dict(case)

            try:
                stop_simulation(args.stop_timeout)
                start_simulation(index, args.startup_timeout)
                wait_for_preflight(args.preflight_timeout)
                case_log_list = temp_path / "case_{:02d}_logs.txt".format(index)
                case_summary = temp_path / "case_{:02d}_summary.json".format(index)
                command = child_command(
                    args, case, case_log_list, case_summary
                )
                print("running: {}".format(shlex.join(command)), flush=True)
                completed = subprocess.run(command, check=False)
                result = load_child_result(case_summary)
                result["isolated_session"] = True
                result["case_index"] = index

                if completed.returncode != 0 and result.get("status") == "complete":
                    result["status"] = "runner_error"
                    result["error"] = (
                        "single-case runner exited with status {}".format(
                            completed.returncode
                        )
                    )

            except Exception as error:
                result.update(
                    {
                        "status": "runner_error",
                        "error": str(error),
                        "finished_at": datetime.now(timezone.utc).isoformat(),
                        "isolated_session": True,
                        "case_index": index,
                    }
                )

            finally:
                if not (args.keep_last_session and index == len(cases)):
                    try:
                        stop_simulation(args.stop_timeout)
                    except Exception as cleanup_error:
                        result["cleanup_error"] = str(cleanup_error)

            results.append(result)
            write_progress(results, args.run_summary, args.log_list)
            print(json.dumps(result, indent=2, sort_keys=True), flush=True)

            if result.get("status") != "complete" and args.fail_fast:
                break

    complete = sum(result.get("status") == "complete" for result in results)
    print("isolated matrix logs recorded: {}/{}".format(complete, len(cases)))
    return 0 if complete == len(cases) else 1


if __name__ == "__main__":
    sys.exit(main())
