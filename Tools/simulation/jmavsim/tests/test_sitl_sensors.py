#!/usr/bin/env python3
"""Exercise jMAVSim input controls through the QGC MAVLink-console protocol.

Start a disarmed `make px4_sitl_default jmavsim` session first, then run:
    python3 Tools/simulation/jmavsim/tests/test_sitl_sensors.py

Requires pymavlink. The test restores the model input masks before exiting.
It does not arm the vehicle or change EKF source-selection parameters.
"""

import argparse
import math
import re
import time

from pymavlink import mavutil


class Console:
    def __init__(self, endpoint):
        self.link = mavutil.mavlink_connection(endpoint, source_system=245,
                                               source_component=190)
        self.next_heartbeat = 0
        self.heartbeat = None
        self.output = ""
        deadline = time.monotonic() + 20
        while self.heartbeat is None and time.monotonic() < deadline:
            self.pump(0.1)
        if self.heartbeat is None:
            raise TimeoutError("No PX4 heartbeat")
        if self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError("Disarm SITL before testing sensor controls")
        self.command("")
        if not re.search(r"SYS_AUTOSTART[^\n]*\b10017\b", self.command("param show SYS_AUTOSTART")):
            raise RuntimeError("This test requires jMAVSim airframe 10017")

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= self.next_heartbeat:
                self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                             mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                             0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
                self.next_heartbeat = now + 1
            msg = self.link.recv_match(blocking=True, timeout=min(0.05, max(0, end - now)))
            if msg is None:
                continue
            if msg.get_type() == "HEARTBEAT" and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self.heartbeat = msg
            elif msg.get_type() == "SERIAL_CONTROL" and msg.device == 10:
                self.output += bytes(msg.data[:msg.count]).decode("utf-8", errors="replace")

    def command(self, text, timeout=8):
        self.pump(0.1)
        self.output = ""
        data = (text + "\n").encode()
        for offset in range(0, len(data), 70):
            part = data[offset:offset + 70]
            self.link.mav.serial_control_send(
                10, mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0, 0, len(part), list(part) + [0] * (70 - len(part)))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(0.1)
            cleaned = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", self.output)
            # Background module diagnostics can arrive after the completed
            # command's prompt (notably during an intentional IMU outage).
            if re.search(r"(?:^|[\r\n])pxh> ", cleaned):
                return cleaned
        raise TimeoutError(f"Console timeout: {text}\n{self.output}")

    def sample(self, topic, instance=0):
        output = self.command(f"listener {topic} -i {instance} -n 1")
        match = re.search(r"\btimestamp:\s*(\d+)", output)
        if match is None:
            raise AssertionError(f"No sample for {topic}[{instance}]\n{output}")
        return int(match.group(1)), output

    def param(self, name):
        output = self.command(f"param show {name}")
        match = re.search(re.escape(name) + r"[^\n]*:\s*([-+0-9.eE]+)\s*$", output, re.MULTILINE)
        if match is None:
            raise AssertionError(output)
        return float(match.group(1))

    def close(self):
        self.link.mav.serial_control_send(10, 0, 0, 0, 0, [0] * 70)
        self.link.close()


def expect(condition, message):
    if not condition:
        raise AssertionError(message)
    print("PASS:", message, flush=True)


def check_toggle(console, name, topic, instance=0, other=None):
    console.command(f"simulator_mavlink sensor {name} off")
    console.pump(1.2)
    first, _ = console.sample(topic, instance)
    clock_first, _ = console.sample("sensor_accel", 1 if name == "imu1" else 0)
    console.pump(0.4)
    second, _ = console.sample(topic, instance)
    clock_second, _ = console.sample("sensor_accel", 1 if name == "imu1" else 0)
    expect(first == second, f"{name}: publication stops")
    expect(clock_second > clock_first, f"{name}: simulator clock and console remain live")
    if other is not None:
        before, _ = console.sample(other)
        console.pump(0.3)
        after, _ = console.sample(other)
        expect(after > before, f"{name}: {other} continues independently")
    console.command(f"simulator_mavlink sensor {name} on")
    console.pump(1.2)
    restored, _ = console.sample(topic, instance)
    expect(restored > second, f"{name}: publication resumes")


def check_metadata(console):
    """Check the actual PX4 inputs, including absent pre-advertised instances."""
    expect(console.param("SIM_MAV_IMU") == 3 and console.param("SIM_MAV_SENS") == 63,
           "metadata check starts with all model inputs enabled")
    for topic, instance in (("sensor_accel", 2), ("sensor_gyro", 2),
                            ("sensor_baro", 1), ("sensor_mag", 1),
                            ("sensor_gnss", 1), ("sensor_gnss_relative", 1),
                            ("distance_sensor", 1), ("vehicle_visual_odometry", 1)):
        output = console.command(f"listener {topic} -i {instance} -n 1")
        timestamp = re.search(r"\btimestamp:\s*(\d+)", output)
        expect(timestamp is None or int(timestamp.group(1)) == 0,
               f"{topic}[{instance}]: no extra sensor output")

    def scalar(output, name):
        match = re.search(r"\b" + re.escape(name) + r":\s*([-+0-9.eE]+)", output)
        if match is None:
            raise AssertionError(f"Missing {name}\n{output}")
        return float(match.group(1))

    _, gps = console.sample("sensor_gnss")
    for name, expected in (("eph", 0.02), ("epv", 0.03), ("speed_accuracy", 0.05)):
        expect(math.isclose(scalar(gps, name), expected, abs_tol=0.0001),
               f"GNSS {name} matches model uncertainty")
    for topic in ("vehicle_gnss", "vehicle_gnss_heading"):
        _, output = console.sample(topic)
        delay = scalar(output, "timestamp") - scalar(output, "timestamp_sample")
        expect(180000 <= delay <= 220000, f"{topic}: receiver delay applied once ({delay:.0f} us)")

    _, odometry = console.sample("vehicle_visual_odometry")
    expect(scalar(odometry, "pose_frame") == 1 and scalar(odometry, "velocity_frame") == 1,
           "visual position and velocity use NED")
    expect(scalar(odometry, "quality") == 100, "visual MAVLink 2 quality extension received")
    for name in ("position_variance", "orientation_variance", "velocity_variance"):
        match = re.search(r"\b" + name + r":\s*\[([^]]+)\]", odometry)
        values = [] if match is None else [float(value) for value in match.group(1).split(",")]
        expect(len(values) == 3 and all(math.isfinite(value) and value > 0 for value in values),
               f"visual {name}: finite and nonzero")
    expect(console.param("EKF2_EV_DELAY") == 40, "visual delay compensation is 40 ms")

    _, distance = console.sample("distance_sensor")
    expect(math.isclose(scalar(distance, "min_distance"), 0.1, abs_tol=0.001)
           and math.isclose(scalar(distance, "max_distance"), 30, abs_tol=0.001)
           and scalar(distance, "orientation") == 25, "range limits and body-down orientation received")


def run(console):
    console.command("simulator_mavlink sensor reset")
    console.pump(2)
    print(console.command("simulator_mavlink status"), flush=True)
    _, gps = console.sample("sensor_gnss")
    expect(bool(re.search(r"fix_type:\s*6\b", gps)), "single GNSS reports RTK fixed")
    for topic, count in (("sensor_accel", 2), ("sensor_gyro", 2),
                         ("sensor_baro", 1), ("sensor_mag", 1),
                         ("sensor_gnss", 1), ("sensor_gnss_relative", 1),
                         ("distance_sensor", 1), ("vehicle_visual_odometry", 1)):
        for instance in range(count):
            first, _ = console.sample(topic, instance)
            console.pump(0.2)
            second, _ = console.sample(topic, instance)
            expect(second > first, f"{topic}[{instance}]: updates")

    check_toggle(console, "imu1", "sensor_accel", 0)
    check_toggle(console, "imu2", "sensor_gyro", 1)
    check_toggle(console, "baro", "sensor_baro")
    check_toggle(console, "mag", "sensor_mag")
    check_toggle(console, "gnss", "sensor_gnss", other="sensor_gnss_relative")
    check_toggle(console, "gnss_yaw", "sensor_gnss_relative", other="sensor_gnss")
    check_toggle(console, "range", "distance_sensor")
    check_toggle(console, "vision", "vehicle_visual_odometry")

    console.command("simulator_mavlink sensor imu1 off")
    console.pump(1.2)
    console.command("simulator_mavlink sensor imu2 off")
    expect(console.param("SIM_MAV_IMU") == 2, "CLI retains the last available IMU")
    for invalid in (0, 4, 8):
        console.command(f"param set SIM_MAV_IMU {invalid}")
        console.pump(1.2)
        expect(console.param("SIM_MAV_IMU") == 2,
               f"invalid direct IMU mask {invalid} is rolled back")
    console.command("simulator_mavlink sensor reset")

    console.command("param set SIM_MAV_SENS 0")
    console.pump(1.2)
    first, _ = console.sample("sensor_accel")
    console.pump(0.4)
    second, _ = console.sample("sensor_accel")
    expect(second > first, "all non-IMU inputs can be disabled together")
    console.command("simulator_mavlink sensor reset")
    console.pump(1.2)
    expect(console.param("SIM_MAV_IMU") == 3, "reset restores both model IMUs")
    expect(console.param("SIM_MAV_SENS") == 63, "reset restores all other model inputs")
    print(console.command("simulator_mavlink status"), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14540")
    parser.add_argument("--metadata-only", action="store_true",
                        help="Check sensor counts, frames, uncertainty and delay without toggling inputs")
    args = parser.parse_args()
    connection = Console(args.endpoint)
    try:
        if args.metadata_only:
            check_metadata(connection)
        else:
            run(connection)
    finally:
        try:
            if not args.metadata_only:
                connection.command("simulator_mavlink sensor reset")
        finally:
            connection.close()
