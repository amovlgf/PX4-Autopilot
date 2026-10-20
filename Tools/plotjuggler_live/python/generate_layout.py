#!/usr/bin/env python3
"""Generate a native PlotJuggler layout from the staged PX4 message manifest.

The XML structure follows the PX4 official sample and PlotJuggler 3.17.2;
the streamer configuration follows plotjuggler-ros-plugins 2.3.1. Generation
checks source fields, but does not constitute a GUI/live-data acceptance test.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

from monitor_common import load_manifest


CORE_VERSION = "3.17.2"
ROS_PLUGIN_VERSION = "2.3.1"
STREAMER = "ROS2 Topic Subscriber"
COLORS = ("#1f77b4", "#d62728", "#2ca02c", "#ff7f0e")
SOURCES = {
    "native_xml_sample": "https://github.com/PX4/PX4-Autopilot/blob/"
    "7293640e6a/docs/assets/flight_log_analysis/plot_juggler/plotjuggler_sample_view.xml",
    "humble_package_index": "http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages.gz",
    "cli_3_17_2": "https://github.com/PlotJuggler/PlotJuggler/blob/3.17.2/plotjuggler_app/main.cpp",
    "cli_3_9_2": "https://github.com/PlotJuggler/PlotJuggler/blob/3.9.2/plotjuggler_app/main.cpp",
    "layout_loader": "https://github.com/PlotJuggler/PlotJuggler/blob/3.17.2/"
    "plotjuggler_app/mainwindow.cpp",
    "ros_array_paths": "https://github.com/PlotJuggler/PlotJuggler/blob/3.17.2/"
    "plotjuggler_plugins/ParserROS/rosx_introspection/src/stringtree_leaf.cpp",
    "streamer_options": "https://github.com/PlotJuggler/plotjuggler-ros-plugins/blob/"
    "2.3.1/src/parser_configuration.cpp",
    "streamer_start": "https://github.com/PlotJuggler/plotjuggler-ros-plugins/blob/"
    "2.3.1/src/DataStreamROS2/datastream_ros2.cpp",
}
WARNINGS = [
    "Source-checked layout; GUI loading and live curves require separate acceptance.",
    "Stock PlotJuggler may ask Start Streaming, then ask to confirm the preselected ROS topics.",
    "Controller curves can remain empty while PX4 is disarmed; no flight commands are sent.",
    "Motor curves show raw normalized thrust; NaN means disarmed and is not replaced with zero.",
    "The time axis uses ROS receive time, not synchronized PX4 sample time.",
]


def installed_versions():
    """Record installed ROS package versions without importing or starting ROS."""
    found = {}
    prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    prefixes.append("/opt/ros/humble")
    for name in ("plotjuggler", "plotjuggler_ros"):
        for prefix in filter(None, prefixes):
            path = Path(prefix) / "share" / name / "package.xml"
            if path.is_file():
                found[name] = ET.parse(str(path)).getroot().findtext("version")
                break
    return found


class CurveNames:
    """Resolve actual topic names and reject stale/nonexistent message fields."""

    def __init__(self, manifest, workspace):
        self.manifest = manifest
        self.workspace = Path(workspace)
        self.fields = {}

    def px4(self, base, field, index=None):
        spec = self.manifest["topics"].get(base, {})
        if not spec.get("exported") or not spec.get("definition_available"):
            raise ValueError("layout requires exported message definition: " + base)
        topic = spec["topic"]
        if not topic.startswith("/") or "/fmu/in/" in topic:
            raise ValueError("layout only accepts absolute PX4 output topics")
        message = spec["type"].split("/")[-1]
        if message not in self.fields:
            relative = "msg/" + message + ".msg"
            definition = self.workspace / "src/px4_msgs" / relative
            data = definition.read_bytes()
            expected = self.manifest["interfaces"][relative]["sha256"]
            if hashlib.sha256(data).hexdigest() != expected:
                raise ValueError("staged message was modified: " + relative)
            fields = {}
            for line in data.decode().splitlines():
                line = line.partition("#")[0].strip()
                if not line or "=" in line:
                    continue
                match = re.fullmatch(r"\S+?(?:\[(\d+)\])?\s+([A-Za-z_]\w*)", line)
                if match:
                    fields[match.group(2)] = int(match.group(1)) if match.group(1) else None
            self.fields[message] = fields
        fields = self.fields[message]
        if field not in fields:
            raise ValueError("message field is missing: " + message + "/" + field)
        size = fields[field]
        if index is not None and (size is None or not 0 <= index < size):
            raise ValueError("invalid array index: " + message + "/" + field)
        if index is None and size is not None:
            raise ValueError("array field requires an index: " + message + "/" + field)
        # ROS ParserROS/rosx_introspection 3.17.2 uses brackets, unlike ULog .00.
        return topic + "/" + field + ("[%d]" % index if index is not None else "")


def plot(parent, title, curves, lower, upper):
    area = ET.SubElement(parent, "DockArea", name=title)
    element = ET.SubElement(area, "plot", style="Lines", mode="TimeSeries",
                            flip_y="false", flip_x="false")
    ET.SubElement(element, "range", bottom=str(lower), top=str(upper), left="0", right="30")
    ET.SubElement(element, "limitY")
    for index, curve in enumerate(curves):
        ET.SubElement(element, "curve", name=curve, color=COLORS[index % len(COLORS)])


def build_layout(manifest, workspace):
    names = CurveNames(manifest, workspace)
    rate = lambda field: names.px4("vehicle_rates_setpoint", field)
    angular = lambda index: names.px4("vehicle_angular_velocity", "xyz", index)
    position = lambda field: names.px4("vehicle_local_position", field)
    integral = lambda field: names.px4("rate_ctrl_status", field)
    torque = lambda index: names.px4("vehicle_torque_setpoint", "xyz", index)
    motor = lambda index: names.px4("actuator_motors", "control", index)
    attitude = "/px4_monitor/attitude_deg/vector/"
    tabs = [
        ("Attitude", [
            ("Roll [deg]", [attitude + "x"], -180, 180),
            ("Pitch [deg]", [attitude + "y"], -90, 90),
            ("Yaw [deg]", [attitude + "z"], -180, 180),
            ("Yaw unwrapped [deg]", ["/px4_monitor/yaw_unwrapped_deg/data"], -180, 180),
        ]),
        ("Rate Control", [
            ("Roll rate: measured / setpoint [rad/s]", [angular(0), rate("roll")], -3, 3),
            ("Pitch rate: measured / setpoint [rad/s]", [angular(1), rate("pitch")], -3, 3),
            ("Yaw rate: measured / setpoint [rad/s]", [angular(2), rate("yaw")], -3, 3),
            ("Body gyro x / y / z [rad/s]", [names.px4("sensor_combined", "gyro_rad", i)
                                                   for i in range(3)], -3, 3),
        ]),
        ("Position", [
            ("North position [m]", [position("x")], -10, 10),
            ("East position [m]", [position("y")], -10, 10),
            ("Down position [m] (NED, positive down)", [position("z")], -10, 10),
            ("NED velocity x / y / z [m/s]", [position(field) for field in ("vx", "vy", "vz")],
             -5, 5),
        ]),
        ("PID Analysis", [
            ("Rate integrator roll / pitch / yaw [normalized torque]",
             [integral(field) for field in ("rollspeed_integ", "pitchspeed_integ", "yawspeed_integ")], -1, 1),
            ("Torque setpoint x / y / z [normalized]", [torque(i) for i in range(3)], -1, 1),
            ("Motor thrust 1 / 2 / 3 / 4 [normalized; NaN = disarmed]",
             [motor(i) for i in range(4)], -1, 1),
        ]),
    ]
    root = ET.Element("root")
    root.append(ET.Comment("Generated from official source formats; see the adjacent .sources.json for provenance."))
    widget = ET.SubElement(root, "tabbed_widget", parent="main_window", name="Main Window")
    for title, plots in tabs:
        tab = ET.SubElement(widget, "Tab", containers="1", tab_name=title)
        container = ET.SubElement(tab, "Container")
        # A single splitter is also recognized by readAllCurvesFromXML when
        # previouslyLoaded_Streamer creates placeholders before live data arrive.
        splitter = ET.SubElement(container, "DockSplitter", count=str(len(plots)),
                                 orientation="-", sizes=";".join([format(1 / len(plots), ".6g")] * len(plots)))
        for title, curves, lower, upper in plots:
            plot(splitter, title, curves, lower, upper)
    ET.SubElement(widget, "currentTabIndex", index="0")
    ET.SubElement(root, "use_relative_time_offset", enabled="1")
    ET.SubElement(root, "streaming_buffer_size", value="60")
    plugins = ET.SubElement(root, "Plugins")
    plugin = ET.SubElement(plugins, "plugin", ID=STREAMER)
    selected = sorted({spec["topic"] for spec in manifest["topics"].values()
                       if spec.get("exported") and spec.get("definition_available")}
                      | {"/px4_monitor/attitude_deg", "/px4_monitor/yaw_unwrapped_deg"})
    for name, value in (("use_header_stamp", "false"), ("discard_large_arrays", "false"),
                        ("max_array_size", "100"), ("boolean_strings_to_number", "false"),
                        ("remove_suffix_from_strings", "false"),
                        ("selected_topics", ";".join(selected))):
        ET.SubElement(plugin, name, value=value)
    ET.SubElement(root, "previouslyLoaded_Streamer", name=STREAMER)
    ET.SubElement(root, "customMathEquations")
    ET.SubElement(root, "snippets")
    return root


def generate(manifest_path, output):
    manifest_path, output = Path(manifest_path), Path(output)
    manifest = load_manifest(manifest_path)
    runtime = installed_versions()
    if runtime.get("plotjuggler") not in (None, CORE_VERSION):
        raise ValueError("layout array paths were checked for PlotJuggler " + CORE_VERSION
                         + "; installed version is " + runtime["plotjuggler"])
    if runtime.get("plotjuggler_ros") not in (None, ROS_PLUGIN_VERSION):
        raise ValueError("layout streamer options were checked for plotjuggler_ros " + ROS_PLUGIN_VERSION
                         + "; installed version is " + runtime["plotjuggler_ros"])
    root = build_layout(manifest, manifest_path.parent)
    # minidom is part of Python 3.8 too (ElementTree.indent requires 3.9).
    from xml.dom import minidom
    data = minidom.parseString(ET.tostring(root)).toprettyxml(indent=" ", encoding="UTF-8")
    provenance_path = output.with_suffix(".sources.json")
    provenance = {
        "owner": "px4_plotjuggler_live", "schema_version": 1,
        "manifest_fingerprint": manifest["fingerprint"],
        "px4_revision": manifest.get("px4_revision"),
        "source_versions": {"plotjuggler": CORE_VERSION, "plotjuggler_ros": ROS_PLUGIN_VERSION},
        "installed_versions": runtime, "sources": SOURCES,
        "official_humble_index_observed": {
            "date": "2026-10-19",
            "ros-humble-plotjuggler": "3.17.2-3jammy.20260908.091409",
            "ros-humble-plotjuggler-ros": "2.3.1-2jammy.20260915.205807",
        },
        "startup": {
            "arguments": ["--layout", str(output)],
            "confirmation_dialogs": ["Start Streaming? (Yes)", "Confirm preselected ROS topics"],
            "maximum_expected_confirmations": 2,
            "streaming_buffer_seconds": 60,
            "buffer_cli_semantics": "--buffer_size sets the spinbox maximum; reaching that value means infinite buffering",
            "finite_buffer_requirement": "pass a CLI maximum greater than 60 and set MainWindow.streamingBufferValue=60 in the dedicated QSettings file",
            "settings_file": "/work/home/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini",
            "settings_section": "General",
            "settings_key": "MainWindow.streamingBufferValue",
            "start_streamer_cli_3_17_2": "bare flag, do not combine with a layout that restores the streamer",
            "start_streamer_cli_3_9_2": "requires plugin-name argument; older layout/array format is not supported here",
            "reason_for_layout_only": "previouslyLoaded_Streamer loads native streamer state and creates curve placeholders",
        },
        "layout_sha256": hashlib.sha256(data).hexdigest(),
        "gui_verified": False, "warnings": WARNINGS,
        "curves": [curve.attrib["name"] for curve in root.iter("curve")],
    }
    provenance_data = (json.dumps(provenance, indent=2, sort_keys=True) + "\n").encode()
    # Preserve both user-saved layouts and their metadata, including runtime
    # acceptance evidence. Repeated setup is allowed only for identical XML.
    if output.exists():
        if output.read_bytes() != data:
            raise ValueError("layout already exists with different content; preserve it and choose --output: "
                             + str(output))
        return {"layout": str(output), "provenance": str(provenance_path), "changed": False,
                "gui_verified": False, "warnings": WARNINGS}
    if provenance_path.exists():
        raise ValueError("layout provenance already exists; choose a new --output: " + str(provenance_path))
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation cannot replace an existing user file or a racing writer.
    with provenance_path.open("xb") as stream:
        stream.write(provenance_data)
    with output.open("xb") as stream:
        stream.write(data)
    return {"layout": str(output), "provenance": str(provenance_path), "changed": True,
            "gui_verified": False, "warnings": WARNINGS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path("/work/current/manifest.json"))
    parser.add_argument("--output", type=Path, default=Path("/work/layout-candidate.xml"))
    args = parser.parse_args()
    try:
        print(json.dumps(generate(args.manifest, args.output), sort_keys=True))
    except (OSError, ValueError, KeyError, ET.ParseError) as error:
        print("layout generation failed: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
