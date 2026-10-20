#!/usr/bin/env python3
"""Reject stale PX4 topic names before opening a selected PlotJuggler layout."""

import argparse
from pathlib import Path
import re
import shlex
import sys
import xml.etree.ElementTree as ET

from monitor_common import load_manifest

PX4_OUTPUT = re.compile(r"^(/(?:[^/]+/)*fmu/out/([^/]+))(?:/|$)")


def validate_layout_topics(manifest, layout):
    """Check known PX4 outputs; preserve unrelated custom topics and curves."""
    layout = Path(layout)
    if not layout.is_file():
        return {"skipped": True, "checked": 0, "unverified_px4_topics": []}
    root = ET.parse(str(layout)).getroot()
    names = {curve.get("name", "") for curve in root.iter("curve")}
    for plugin in root.iter("plugin"):
        if plugin.get("ID") == "ROS2 Topic Subscriber":
            for selection in plugin.iter("selected_topics"):
                names.update(name.strip() for name in selection.get("value", "").split(";"))
    checked = set()
    unverified = set()
    problems = set()
    for name in names:
        match = PX4_OUTPUT.match(name)
        if not match:
            continue
        topic, leaf = match.groups()
        base = re.sub(r"_v\d+$", "", leaf)
        spec = manifest["topics"].get(base)
        if spec is None:
            # The monitoring manifest covers its configured monitoring channels,
            # not every possible PX4 extension. Do not invent their versions.
            unverified.add(topic)
            continue
        checked.add(topic)
        if not spec.get("exported"):
            problems.add(topic + " -> not exported by the current DDS configuration")
        elif topic != spec["topic"]:
            problems.add(topic + " -> expected " + spec["topic"])
    if problems:
        raise ValueError("stale or unavailable PX4 layout topics:\n  " + "\n  ".join(sorted(problems)))
    return {"skipped": False, "checked": len(checked), "unverified_px4_topics": sorted(unverified)}


def repair_instructions(manifest_path, manifest):
    identity = manifest.get("fingerprint", "current")[:12]
    candidate = "/work/layouts/local/candidate-" + identity + ".xml"
    command = " ".join(shlex.quote(part) for part in (
        "python3", "/opt/px4-monitor/python/generate_layout.py", "--manifest", str(manifest_path),
        "--output", candidate))
    return ("Regenerate a candidate in a one-off monitoring container (host command in README):\n  " + command +
            "\nThen select the corrected layout with PJ_LAYOUT=" + shlex.quote(candidate) +
            ". Back up and move any higher-priority /work/current/layouts/live.xml first, "
            "or update that layout manually. Existing layouts are not overwritten; "
            "the candidate still requires GUI verification.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--layout", required=True)
    args = parser.parse_args(argv)
    manifest = None
    try:
        manifest = load_manifest(args.manifest)
        result = validate_layout_topics(manifest, args.layout)
    except (OSError, ValueError, KeyError, ET.ParseError) as error:
        print("layout validation failed: " + str(error), file=sys.stderr)
        if manifest is not None:
            print(repair_instructions(args.manifest, manifest), file=sys.stderr)
        return 1
    if result["skipped"]:
        print("Layout file absent; keeping manual PlotJuggler selection.")
    else:
        print("Layout topic mapping checked against the current manifest: %d PX4 topics." % result["checked"])
        if result["unverified_px4_topics"]:
            print("Additional PX4 topics outside this monitoring manifest require manual validation: " +
                  ", ".join(result["unverified_px4_topics"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
