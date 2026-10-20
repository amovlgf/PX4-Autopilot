#!/usr/bin/env python3
"""Validate a jMAVSim source profile across an externally managed restart.

Requires pymavlink and a disarmed airframe-10017 SITL session. Configure:
    python3 Tools/simulation/jmavsim/tests/test_sitl_flight.py configure --profile rtk
Stop and restart the same PX4/jMAVSim instance, retaining its parameter file:
    python3 Tools/simulation/jmavsim/tests/test_sitl_flight.py fly --profile rtk
Repeat both steps with --profile vision. The fly action arms SITL, takes off
using local offboard position setpoints, hovers, and lands. It checks actual
EKF aid-source fusion as well as control flags. It does not start/stop SITL.
The check action performs the restart and fusion checks without arming, and
also supports mixed, baro_mag and vision_range profiles.
The configure action intentionally disables vision and one IMU as restart
sentinels; the normal jMAVSim startup must restore both input masks.
"""

import argparse
import json
import math
from pathlib import Path
import re
import time

from pymavlink import mavutil

from test_sitl_sensors import Console, expect


COMMON_PARAMETERS = {
    "EKF2_BARO_CTRL": 0,
    "EKF2_RNG_CTRL": 0,
    "EKF2_MAG_TYPE": 5,
}
PROFILES = {
    "rtk": {
        "parameters": {"EKF2_GPS_CTRL": 15, "EKF2_EV_CTRL": 0, "EKF2_HGT_REF": 1},
        "flags": ("cs_gnss_pos", "cs_gnss_vel", "cs_gps_hgt", "cs_gnss_yaw"),
        "aids": ("gnss_pos", "gnss_vel", "gnss_hgt", "gnss_yaw"),
        "faults": ("cs_gnss_fault", "cs_gnss_hgt_fault", "cs_gnss_yaw_fault"),
    },
    "vision": {
        "parameters": {"EKF2_GPS_CTRL": 0, "EKF2_EV_CTRL": 15, "EKF2_HGT_REF": 3},
        "flags": ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt", "cs_ev_yaw"),
        "aids": ("ev_pos", "ev_vel", "ev_hgt", "ev_yaw"),
        "faults": ("cs_ev_yaw_fault",),
    },
    "mixed": {
        "parameters": {"EKF2_GPS_CTRL": 15, "EKF2_EV_CTRL": 5, "EKF2_HGT_REF": 1},
        "flags": ("cs_gnss_pos", "cs_gnss_vel", "cs_gps_hgt", "cs_gnss_yaw",
                  "cs_ev_pos", "cs_ev_vel"),
        "aids": ("gnss_pos", "gnss_vel", "gnss_hgt", "gnss_yaw", "ev_pos", "ev_vel"),
        "faults": ("cs_gnss_fault", "cs_gnss_hgt_fault", "cs_gnss_yaw_fault"),
    },
    "baro_mag": {
        "parameters": {"EKF2_GPS_CTRL": 5, "EKF2_EV_CTRL": 0, "EKF2_HGT_REF": 0,
                       "EKF2_BARO_CTRL": 1, "EKF2_MAG_TYPE": 0},
        "flags": ("cs_gnss_pos", "cs_gnss_vel", "cs_baro_hgt"),
        "any_flags": (("cs_mag_hdg", "cs_mag_3d"),),
        # Automatic magnetometer fusion uses heading mode on the ground.
        # Both modes report accepted samples through the shared mag aid topic.
        "aids": ("gnss_pos", "gnss_vel", "baro_hgt", "mag"),
        "faults": ("cs_gnss_fault", "cs_baro_fault", "cs_mag_fault"),
    },
    "vision_range": {
        "parameters": {"EKF2_GPS_CTRL": 0, "EKF2_EV_CTRL": 13, "EKF2_HGT_REF": 2,
                       "EKF2_RNG_CTRL": 2},
        "flags": ("cs_ev_pos", "cs_ev_vel", "cs_ev_yaw", "cs_rng_hgt"),
        "aids": ("ev_pos", "ev_vel", "ev_yaw", "rng_hgt"),
        "faults": ("cs_ev_yaw_fault", "cs_rng_fault"),
    },
}
FUSION_FLAGS = ("cs_gnss_pos", "cs_gnss_vel", "cs_gps_hgt", "cs_gnss_yaw",
                "cs_ev_pos", "cs_ev_vel", "cs_ev_hgt", "cs_ev_yaw",
                "cs_baro_hgt", "cs_rng_hgt", "cs_mag_hdg", "cs_mag_3d")


def fields(output):
    return dict(re.findall(r"^\s*(\w+):\s*(\S+)", output, re.MULTILINE))


def flag(values, name):
    if name not in values:
        raise AssertionError(f"Missing {name} in topic output")
    return values[name].lower() in ("true", "1")


class FlightConsole(Console):
    """Maintain offboard publication while the inherited console waits for replies."""

    def __init__(self, endpoint):
        self.target = None
        self.next_setpoint = 0
        self.started = time.monotonic()
        super().__init__(endpoint)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            now = time.monotonic()
            if self.target is not None and now >= self.next_setpoint:
                x, y, z, yaw = self.target
                mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
                self.link.mav.set_position_target_local_ned_send(
                    int((now - self.started) * 1000) & 0xffffffff,
                    self.heartbeat.get_srcSystem(), self.heartbeat.get_srcComponent(),
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                    x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0)
                self.next_setpoint = now + 0.05
            super().pump(min(0.02, max(0, end - time.monotonic())))

    def topic(self, name):
        return fields(self.sample(name)[1])


def verify_ground(console):
    state = console.topic("vehicle_status")
    if int(state["arming_state"]) == 2:
        raise RuntimeError("The test must start disarmed")
    if not flag(console.topic("vehicle_land_detected"), "landed"):
        raise RuntimeError("The test must start on the ground")


def profile_parameters(profile):
    return dict(COMMON_PARAMETERS, **PROFILES[profile]["parameters"])


def configure(console, args):
    verify_ground(console)
    parameters = profile_parameters(args.profile)
    for name, value in parameters.items():
        console.command(f"param set {name} {value}")
        expect(console.param(name) == value, f"saved {name}={value}")
    console.command("param set SIM_MAV_IMU 1")
    console.command("param set SIM_MAV_SENS 31")
    console.pump(1.2)
    expect(console.param("SIM_MAV_IMU") == 1, "restart sentinel: one IMU disabled")
    expect(console.param("SIM_MAV_SENS") == 31, "restart sentinel: vision input disabled")
    console.command("param save")
    args.state_file.write_text(json.dumps({
        "profile": args.profile,
        "parameters": parameters,
        "endpoint": args.endpoint,
        "configured_at": time.time(),
        "phase": "awaiting_external_restart",
    }, indent=2) + "\n", encoding="utf-8")
    print("Configuration saved. Stop and restart this PX4/jMAVSim instance, then run check or fly.",
          flush=True)


def check_fusion(console, profile, previous=None):
    specification = PROFILES[profile]
    values = console.topic("estimator_status_flags")
    required = specification["flags"] + ("cs_tilt_align", "cs_yaw_align")
    for name in required:
        if not flag(values, name):
            raise AssertionError(f"EKF flag {name} is false")
    allowed = set(specification["flags"])
    for alternatives in specification.get("any_flags", ()):
        if not any(flag(values, name) for name in alternatives):
            raise AssertionError(f"None of the expected EKF modes is active: {alternatives}")
        allowed.update(alternatives)
    forbidden = tuple(name for name in FUSION_FLAGS if name not in allowed)
    forbidden += specification["faults"] + (
        "cs_inertial_dead_reckoning", "cs_fake_pos", "cs_fake_hgt")
    for name in forbidden:
        if flag(values, name):
            raise AssertionError(f"Unexpected EKF flag {name}")
    samples = {}
    for name in specification["aids"]:
        aid = console.topic("estimator_aid_src_" + name)
        if not flag(aid, "fused") or flag(aid, "innovation_rejected"):
            raise AssertionError(f"{name}: newest observation was not accepted and fused")
        samples[name] = int(aid["timestamp_sample"])
        if samples[name] == 0 or (previous is not None and samples[name] <= previous[name]):
            raise AssertionError(f"{name}: fused sample timestamp did not advance")
    local = console.topic("vehicle_local_position")
    for name in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid"):
        if not flag(local, name):
            raise AssertionError(f"Local position flag {name} is false")
    return samples


def wait_until(console, predicate, timeout, description):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            result = predicate()
            if result:
                return result
        except (AssertionError, KeyError) as error:
            last_error = error
        console.pump(0.2)
    raise TimeoutError(f"Timed out waiting for {description}: {last_error}")


def land(console):
    """Use the normal landing mode; never force-disarm an airborne vehicle."""
    console.command("commander land")
    wait_until(console, lambda: flag(console.topic("vehicle_land_detected"), "landed"),
               45, "landing")
    console.target = None
    console.command("commander disarm")
    wait_until(console, lambda: int(console.topic("vehicle_status")["arming_state"]) != 2,
               8, "disarming on the ground")


def check_restart(console, args):
    """Shared disarmed validation; does not issue any arming or flight commands."""
    verify_ground(console)
    state = json.loads(args.state_file.read_text(encoding="utf-8"))
    expect(state.get("phase") in ("awaiting_external_restart", "ground_checked"),
           "configuration phase recorded")
    expect(state["profile"] == args.profile and state["endpoint"] == args.endpoint,
           "configuration belongs to this profile and endpoint")
    expect(state["parameters"] == profile_parameters(args.profile), "profile definition unchanged")
    for name, value in state["parameters"].items():
        expect(console.param(name) == value, f"restart preserved {name}={value}")
    expect(console.param("SIM_MAV_IMU") == 3, "restart restored both IMUs")
    expect(console.param("SIM_MAV_SENS") == 63, "restart restored all auxiliary inputs")

    previous = wait_until(console, lambda: check_fusion(console, args.profile),
                          90, "selected estimator sources")
    console.pump(0.5)
    previous = check_fusion(console, args.profile, previous)
    print("PASS: selected sources are accepting fresh observations", flush=True)
    verify_ground(console)
    return state, previous


def check(console, args):
    state, _ = check_restart(console, args)
    state.update(phase="ground_checked", ground_checked_at=time.time(),
                 fusion_sources=PROFILES[args.profile]["aids"])
    args.state_file.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    print(f"PASS: {args.profile} restart and ground fusion checks; vehicle remains disarmed",
          flush=True)
    if args.profile == "vision_range":
        print("NOTE: ground range fusion may use PX4's minimum-range fallback; "
              "this ground check does not validate airborne range geometry.", flush=True)


def fly(console, args):
    state, previous = check_restart(console, args)
    initial = console.topic("vehicle_local_position")
    x, y, z, yaw = (float(initial[name]) for name in ("x", "y", "z", "heading"))
    if not all(math.isfinite(value) for value in (x, y, z, yaw)):
        raise AssertionError("Initial local position or heading is not finite")

    # Local offboard takeoff also works for vision without a global position.
    console.target = (x, y, z - args.altitude, yaw)
    console.pump(2)
    console.command("commander mode offboard")
    wait_until(console, lambda: int(console.topic("vehicle_status")["nav_state"]) == 14,
               8, "offboard mode")
    attempted_arm = False
    try:
        attempted_arm = True
        console.command("commander arm")
        wait_until(console, lambda: int(console.topic("vehicle_status")["arming_state"]) == 2,
                   8, "normal arming")

        def at_height():
            position = console.topic("vehicle_local_position")
            return (abs(float(position["z"]) - console.target[2]) < 0.35
                    and not flag(console.topic("vehicle_land_detected"), "landed"))

        wait_until(console, at_height, 35, "takeoff altitude")
        hover_end = time.monotonic() + args.hover_seconds
        peak_horizontal_error = 0.0
        peak_altitude_error = 0.0
        while time.monotonic() < hover_end:
            previous = check_fusion(console, args.profile, previous)
            position = console.topic("vehicle_local_position")
            horizontal_error = math.hypot(float(position["x"]) - x, float(position["y"]) - y)
            altitude_error = abs(float(position["z"]) - console.target[2])
            if not math.isfinite(horizontal_error + altitude_error):
                raise AssertionError("Non-finite position during hover")
            if horizontal_error > 1.0 or altitude_error > 0.75:
                raise AssertionError(f"Hover error too large: horizontal={horizontal_error:.3f}m, "
                                     f"vertical={altitude_error:.3f}m")
            vehicle = console.topic("vehicle_status")
            if int(vehicle["arming_state"]) != 2 or int(vehicle["nav_state"]) != 14:
                raise AssertionError("Vehicle left armed offboard flight during hover")
            peak_horizontal_error = max(peak_horizontal_error, horizontal_error)
            peak_altitude_error = max(peak_altitude_error, altitude_error)
            console.pump(0.3)
        land(console)
        attempted_arm = False
        state.update(phase="completed", completed_at=time.time(),
                     peak_horizontal_error_m=peak_horizontal_error,
                     peak_altitude_error_m=peak_altitude_error)
        args.state_file.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
        print(f"PASS: {args.profile} takeoff, hover and landing; peak estimated-position "
              f"errors horizontal={peak_horizontal_error:.3f}m, vertical={peak_altitude_error:.3f}m",
              flush=True)
    finally:
        if attempted_arm:
            land(console)
        console.target = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("configure", "check", "fly"))
    parser.add_argument("--profile", choices=tuple(PROFILES), required=True)
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14540")
    parser.add_argument("--state-file", type=Path, default=Path("/tmp/px4-jmavsim-flight.json"))
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--hover-seconds", type=float, default=8.0)
    args = parser.parse_args()
    if not 1.0 <= args.altitude <= 10.0 or not 3.0 <= args.hover_seconds <= 60.0:
        parser.error("altitude must be 1–10 m and hover-seconds must be 3–60 s")
    if args.action == "fly" and args.profile not in ("rtk", "vision"):
        parser.error("fly supports rtk and vision; use check for the other profiles")
    console = FlightConsole(args.endpoint)
    try:
        if args.action == "configure":
            configure(console, args)
        elif args.action == "check":
            check(console, args)
        else:
            fly(console, args)
    finally:
        console.close()


if __name__ == "__main__":
    main()
