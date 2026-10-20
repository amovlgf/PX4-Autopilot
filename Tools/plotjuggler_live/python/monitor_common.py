"""Manifest validation shared by the read-only ROS monitoring tools."""

import importlib
import json
from pathlib import Path

from sync_messages import collect_interfaces, sha256


def verify_source_manifest(manifest, px4_root):
    """Reject stale source definitions without rebuilding or touching the workspace."""
    px4_root = Path(px4_root)
    try:
        dds_bytes = (px4_root / "src/modules/uxrce_dds_client/dds_topics.yaml").read_bytes()
        interfaces = collect_interfaces(px4_root)
    except (OSError, ValueError) as error:
        raise ValueError("cannot validate the PX4 source for this workspace: " + str(error) +
                         "; check the selected checkout and run setup.sh") from error
    expected = manifest.get("interfaces")
    if not isinstance(expected, dict):
        raise ValueError("workspace manifest has no source interface hashes; run setup.sh")
    changed = []
    if sha256(dds_bytes) != manifest.get("dds_sha256"):
        changed.append("DDS export configuration")
    if set(interfaces) != set(expected):
        changed.append("message/service file set")
    for name in sorted(set(interfaces) & set(expected)):
        source = interfaces[name]
        if (source["source"] != expected[name].get("source") or
                sha256(source["data"]) != expected[name].get("sha256")):
            changed.append(name)
    if changed:
        raise ValueError("PX4 source differs from the built message workspace (" + ", ".join(changed[:8]) +
                         (", ..." if len(changed) > 8 else "") +
                         "); run setup.sh before starting or diagnosing monitoring. No files were rebuilt.")


def load_manifest(path, source_root=None):
    manifest = json.loads(Path(path).read_text())
    if manifest.get("schema_version") != 1 or manifest.get("owner") != "px4_plotjuggler_live":
        raise ValueError("unsupported or unrecognized message manifest")
    if not isinstance(manifest.get("topics"), dict):
        raise ValueError("manifest has no topic mapping")
    source_root = source_root or manifest.get("source_root")
    if not source_root:
        raise ValueError("workspace manifest has no source checkout; run setup.sh")
    verify_source_manifest(manifest, source_root)
    return manifest


def message_class(spec):
    package, kind, name = spec["type"].split("/")
    if package != "px4_msgs" or kind != "msg":
        raise ValueError("monitor only accepts local px4_msgs message types")
    cls = getattr(importlib.import_module("px4_msgs.msg"), name)
    if getattr(cls, "MESSAGE_VERSION", 0) != spec["message_version"]:
        raise ValueError("sourced px4_msgs message version does not match manifest: " + name)
    return cls
