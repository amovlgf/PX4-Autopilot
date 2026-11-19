#!/usr/bin/env python3
"""Generate MAVLink 2 bindings for Flycore-Q incoming inspection."""

import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--definitions", required=True, type=Path, help="Upstream MAVLink XML directory")
    parser.add_argument("--pymavlink-root", type=Path, help="Directory containing the pinned pymavlink package")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--language", default="C", choices=("C", "Python3"))
    args = parser.parse_args()
    if args.pymavlink_root:
        sys.path.insert(0, str(args.pymavlink_root.resolve()))
    from pymavlink.generator import mavgen

    dialect = Path(__file__).with_name("flycore_test.xml")
    definitions = args.definitions.resolve()
    staging = args.output.resolve().parent / "flycore-test-definitions"
    if staging == definitions:
        parser.error("output must stage definitions outside the upstream directory")
    if not (definitions / "common.xml").is_file():
        parser.error(f"common.xml not found in {definitions}")

    messages = ET.parse(dialect).findall("messages/message")
    ids = {message.get("id") for message in messages}
    names = {message.get("name") for message in messages}
    staging.mkdir(parents=True, exist_ok=True)

    for upstream in sorted(definitions.glob("*.xml")):
        for message in ET.parse(upstream).findall("messages/message"):
            if message.get("id") in ids or message.get("name") in names:
                parser.error(f"message collision at {upstream}: {message.attrib}")
        shutil.copyfile(upstream, staging / upstream.name)

    staged_dialect = staging / dialect.name
    shutil.copyfile(dialect, staged_dialect)
    options = mavgen.Opts(str(args.output), wire_protocol="2.0", language=args.language)
    return 0 if mavgen.mavgen(options, [str(staged_dialect)]) else 1


if __name__ == "__main__":
    sys.exit(main())
