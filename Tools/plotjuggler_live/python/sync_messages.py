#!/usr/bin/env python3
"""Stage an immutable, local PX4 ROS interface workspace; never switch current."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

SCHEMA_VERSION = 1
GENERATOR_VERSION = 1
MONITORED_TYPES = {
    "vehicle_attitude": "VehicleAttitude",
    "vehicle_local_position": "VehicleLocalPosition",
    "vehicle_angular_velocity": "VehicleAngularVelocity",
    "sensor_combined": "SensorCombined",
    "vehicle_rates_setpoint": "VehicleRatesSetpoint",
    "rate_ctrl_status": "RateCtrlStatus",
    "vehicle_status": "VehicleStatus",
    "vehicle_odometry": "VehicleOdometry",
    "vehicle_torque_setpoint": "VehicleTorqueSetpoint",
    "estimator_status_flags": "EstimatorStatusFlags",
    "actuator_motors": "ActuatorMotors",
}
PACKAGE_XML = '''<?xml version="1.0"?>
<package format="3">
  <name>px4_msgs</name>
  <version>0.0.0</version>
  <description>Interfaces copied exactly from the monitored local PX4 checkout.</description>
  <maintainer email="maintainers@px4.io">PX4</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
'''


def message_version(content):
    matches = re.findall(r"^\s*uint32\s+MESSAGE_VERSION\s*=\s*(\d+)\s*(?:#.*)?$", content, re.MULTILINE)
    if len(matches) > 1:
        raise ValueError("duplicate MESSAGE_VERSION constants")
    return int(matches[0]) if matches else 0


def normalize_namespace(namespace):
    namespace = namespace.strip("/")
    if namespace and not all(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", part) for part in namespace.split("/")):
        raise ValueError("namespace must consist of valid ROS name segments")
    return "/" + namespace if namespace else ""


def topic_name(base, version=0, namespace="", instance=None):
    suffix = str(instance) if instance is not None else ""
    suffix += "_v%d" % version if version else ""
    return normalize_namespace(namespace) + "/fmu/out/" + base + suffix


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def collect_interfaces(px4_root):
    """Flatten current versioned definitions, excluding archived messages."""
    result = {}
    for folder, pattern, target in (("msg", "*.msg", "msg"), ("msg/versioned", "*.msg", "msg"),
                                    ("srv", "*.srv", "srv"), ("srv/versioned", "*.srv", "srv")):
        for source in sorted((px4_root / folder).glob(pattern)):
            destination = target + "/" + source.name
            if destination in result:
                raise ValueError("duplicate current interface: " + destination)
            result[destination] = {"source": source.relative_to(px4_root).as_posix(), "data": source.read_bytes()}
    if not any(name.startswith("msg/") for name in result):
        raise ValueError("no local PX4 message definitions found")
    return result


def read_publications(path):
    import yaml
    document = yaml.safe_load(path.read_text())
    entries = document.get("publications", [])
    if not isinstance(entries, list):
        raise ValueError("DDS publications must be a list")
    return entries


def build_topic_manifest(interfaces, publications, namespace):
    topics = {}
    for base, class_name in MONITORED_TYPES.items():
        definition = interfaces.get("msg/" + class_name + ".msg")
        version = message_version(definition["data"].decode()) if definition else 0
        matches = [entry for entry in publications if entry.get("topic", "").split("/")[-1] == base
                   and entry.get("instance") in (None, 0)]
        if len(matches) > 1:
            raise ValueError("ambiguous DDS publication for " + base)
        publication = matches[0] if matches else None
        ros_type = "px4_msgs/msg/" + class_name
        if publication and publication.get("type", "").replace("::", "/") != ros_type:
            raise ValueError("DDS type mismatch for " + base)
        # The literal configured publication path matters if the user customized it.
        actual = topic_name(base, version, namespace)
        if publication:
            actual = normalize_namespace(namespace) + "/" + publication["topic"].strip("/")
            if publication.get("instance") is not None:
                actual += str(publication["instance"])
            if version:
                actual += "_v%d" % version
        topics[base] = {
            "topic": actual, "type": ros_type, "message_version": version,
            "definition_available": definition is not None, "exported": publication is not None,
            "rate_limit": float(publication.get("rate_limit", 100.0)) if publication else None,
        }
    return topics


def git_value(root, *arguments):
    result = subprocess.run(["git", "-c", "safe.directory=" + str(root), "-C", str(root), *arguments],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def verify_workspace(workspace, fingerprint):
    """Refuse to reuse or overwrite a user-modified generated package."""
    try:
        manifest = json.loads((workspace / "manifest.json").read_text())
    except (OSError, ValueError) as error:
        raise ValueError("existing workspace has no valid ownership manifest: " + str(workspace)) from error
    if manifest.get("owner") != "px4_plotjuggler_live" or manifest.get("fingerprint") != fingerprint:
        raise ValueError("refusing to overwrite an unrecognized workspace: " + str(workspace))
    package = workspace / "src/px4_msgs"
    expected = manifest["generated_files"]
    actual = {path.relative_to(package).as_posix() for path in package.rglob("*") if path.is_file()}
    if actual != set(expected):
        raise ValueError("generated workspace file set was modified: " + str(workspace))
    for name, digest in expected.items():
        if sha256((package / name).read_bytes()) != digest:
            raise ValueError("generated workspace was modified: " + str(package / name))


def stage_workspace(px4_root, work_root, namespace=""):
    px4_root = Path(px4_root).resolve()
    work_root = Path(work_root).resolve()
    namespace = normalize_namespace(namespace)
    interfaces = collect_interfaces(px4_root)
    dds_path = px4_root / "src/modules/uxrce_dds_client/dds_topics.yaml"
    dds_bytes = dds_path.read_bytes()
    topics = build_topic_manifest(interfaces, read_publications(dds_path), namespace)
    generated = {name: item["data"] for name, item in interfaces.items()}
    generated["package.xml"] = PACKAGE_XML.encode()
    cmake = ('cmake_minimum_required(VERSION 3.8)\nproject(px4_msgs)\n'
             'find_package(ament_cmake REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)\n'
             'rosidl_generate_interfaces(${PROJECT_NAME}\n')
    cmake += "".join('  "%s"\n' % name for name in sorted(interfaces))
    cmake += ')\nament_export_dependencies(rosidl_default_runtime)\nament_package()\n'
    generated["CMakeLists.txt"] = cmake.encode()
    generated_hashes = {name: sha256(data) for name, data in sorted(generated.items())}
    # Runtime source validation uses this absolute path. Host and container
    # mount paths must not reuse a manifest pointing outside their filesystem.
    identity = {"generator": GENERATOR_VERSION, "files": generated_hashes,
                "source_root": str(px4_root), "dds_sha256": sha256(dds_bytes),
                "namespace": namespace, "topics": topics}
    fingerprint = sha256(json.dumps(identity, sort_keys=True).encode())
    parent = work_root / "workspaces"
    parent.mkdir(parents=True, exist_ok=True)
    workspace = parent / fingerprint
    if workspace.exists():
        verify_workspace(workspace, fingerprint)
        return {"workspace": str(workspace), "changed": False, "fingerprint": fingerprint}
    manifest = {
        "schema_version": SCHEMA_VERSION, "owner": "px4_plotjuggler_live", "fingerprint": fingerprint,
        "px4_revision": git_value(px4_root, "rev-parse", "HEAD"),
        "px4_version": git_value(px4_root, "describe", "--always", "--dirty", "--tags"),
        "source_root": str(px4_root), "namespace": namespace, "dds_sha256": sha256(dds_bytes),
        "interfaces": {name: {"source": item["source"], "sha256": sha256(item["data"])}
                       for name, item in sorted(interfaces.items())},
        "generated_files": generated_hashes, "topics": topics,
    }
    staging = Path(tempfile.mkdtemp(prefix=".staging-", dir=str(parent)))
    try:
        for name, data in generated.items():
            path = staging / "src/px4_msgs" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        (staging / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        # Rename only a complete workspace; setup owns the separate current symlink.
        try:
            os.rename(staging, workspace)
        except OSError:
            if not workspace.exists():
                raise
            verify_workspace(workspace, fingerprint)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return {"workspace": str(workspace), "changed": True, "fingerprint": fingerprint}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-root", type=Path, required=True)
    parser.add_argument("--work-root", type=Path, required=True)
    parser.add_argument("--namespace", default="")
    args = parser.parse_args()
    try:
        print(json.dumps(stage_workspace(args.px4_root, args.work_root, args.namespace), sort_keys=True))
    except (OSError, ValueError, ImportError) as error:
        print("message staging failed: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
