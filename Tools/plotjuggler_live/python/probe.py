#!/usr/bin/env python3
"""Receive real DDS samples and diagnose a PX4 monitoring link without commands."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

from attitude_math import quaternion_to_euler_deg
from monitor_common import load_manifest, message_class
from sync_messages import MONITORED_TYPES

CORE_TOPICS = {"vehicle_attitude", "vehicle_local_position", "sensor_combined"}
MONITORING_TOPICS = set(MONITORED_TYPES)
CONTROLLER_TOPICS = {"vehicle_rates_setpoint", "rate_ctrl_status", "vehicle_torque_setpoint", "actuator_motors"}
STALE_SECONDS = 2.0


def validity_fields(base, message):
    if base == "vehicle_local_position":
        return {name: bool(getattr(message, name)) for name in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")}
    return {}


def finite_measurement(base, message):
    """Check measured fields, not optional NaN-valued command fields."""
    if base == "vehicle_attitude":
        try:
            quaternion_to_euler_deg(message.q)
        except (ValueError, TypeError, OverflowError):
            return False
        return True
    fields = {
        "vehicle_local_position": ("x", "y", "z", "vx", "vy", "vz"),
        "vehicle_angular_velocity": ("xyz",),
        "sensor_combined": ("gyro_rad", "accelerometer_m_s2"),
    }.get(base, ())
    for field in fields:
        value = getattr(message, field)
        values = value if hasattr(value, "__iter__") else (value,)
        if not all(math.isfinite(float(item)) for item in values):
            return False
    return True


class TopicSamples:
    def __init__(self):
        self.count = 0
        self.first = None
        self.last = None
        self.last_valid = None
        self.invalid_count = 0
        self.last_validity = {}
        self.invalid_flags_count = 0
        self.last_timestamp = None
        self.timestamp_rollbacks = 0
        self.duplicate_timestamps = 0
        self.advancing_samples = 0
        self.incompatible_qos = 0
        self.arming_state = None

    def record(self, base, message, received_at):
        self.count += 1
        if self.first is None:
            self.first = received_at
        self.last = received_at
        valid = finite_measurement(base, message)
        if not valid:
            self.invalid_count += 1
        self.last_validity = validity_fields(base, message)
        if self.last_validity and not all(self.last_validity.values()):
            self.invalid_flags_count += 1
        stamp = getattr(message, "timestamp", None)
        if stamp is not None and stamp > 0 and stamp != self.last_timestamp:
            self.advancing_samples += 1
            if valid:
                self.last_valid = received_at
        if stamp is not None and self.last_timestamp is not None:
            self.timestamp_rollbacks += int(stamp < self.last_timestamp)
            self.duplicate_timestamps += int(stamp == self.last_timestamp)
        self.last_timestamp = stamp
        if base == "vehicle_status":
            self.arming_state = int(message.arming_state)

    def summary(self, now):
        elapsed = self.last - self.first if self.count > 1 else 0.0
        return {
            "count": self.count, "hz": (self.count - 1) / elapsed if elapsed > 0 else 0.0,
            "age_seconds": now - self.last if self.last is not None else None,
            "valid_age_seconds": now - self.last_valid if self.last_valid is not None else None,
            "invalid_samples": self.invalid_count, "validity": self.last_validity,
            "samples_with_invalid_flags": self.invalid_flags_count,
            "timestamp_rollbacks": self.timestamp_rollbacks, "duplicate_timestamps": self.duplicate_timestamps,
            "advancing_samples": self.advancing_samples,
            "incompatible_qos_events": self.incompatible_qos,
        }


def build_report(manifest, samples, discovered, publisher_qos, now, duration, errors=None):
    errors = errors or {}
    warnings = []
    topics = {}
    missing_monitoring_topics = []
    transport_ok = CORE_TOPICS.issubset(manifest["topics"])
    armed = samples.get("vehicle_status", TopicSamples()).arming_state
    qualifier = "disarmed" if armed == 1 else "arming_state_unknown" if armed is None else "armed"
    for base in sorted(MONITORING_TOPICS - set(manifest["topics"])):
        missing_monitoring_topics.append({"topic": base, "problems": ["missing_from_manifest"],
                                          "condition": qualifier if base in CONTROLLER_TOPICS else "telemetry"})
    for base, spec in manifest["topics"].items():
        observed = samples.get(base, TopicSamples()).summary(now)
        actual_types = discovered.get(spec["topic"], [])
        publishers = publisher_qos.get(spec["topic"], [])
        publication_types = sorted({entry["type"] for entry in publishers})
        problems = []
        # The ROS graph also includes our own subscriptions. Only publisher
        # endpoints establish that a producer currently exists.
        if not publishers:
            problems.append("no_publisher")
        if not spec["exported"]:
            problems.append("not_exported_by_dds_configuration")
        if base in errors:
            problems.append("message_type_unavailable: " + errors[base])
        if any(kind != spec["type"] for kind in publication_types):
            problems.append("incompatible_message_type")
        if not observed["count"]:
            problems.append("no_samples")
        elif observed["advancing_samples"] < 2:
            problems.append("insufficient_samples")
        if observed["count"] and (observed["valid_age_seconds"] is None or observed["valid_age_seconds"] > STALE_SECONDS):
            problems.append("invalid_or_stale_samples")
        if observed["incompatible_qos_events"]:
            problems.append("incompatible_qos")
        if base in CORE_TOPICS and problems:
            transport_ok = False
        elif problems:
            warnings.append({"topic": base, "reason": ",".join(problems),
                             "condition": qualifier if base in CONTROLLER_TOPICS else "telemetry"})
        complete_problems = list(problems)
        if observed["validity"] and not all(observed["validity"].values()):
            warnings.append({"topic": base, "reason": "estimator_validity_flags_false"})
            complete_problems.append("estimator_validity_flags_false")
        if base in MONITORING_TOPICS and complete_problems:
            missing_monitoring_topics.append({"topic": base, "problems": complete_problems,
                                              "condition": qualifier if base in CONTROLLER_TOPICS else "telemetry"})
        if observed["timestamp_rollbacks"]:
            warnings.append({"topic": base, "reason": "timestamp_rollback"})
        if observed["invalid_samples"]:
            warnings.append({"topic": base, "reason": "nonfinite_or_invalid_measurement"})
        topics[base] = dict(observed, topic=spec["topic"], expected_type=spec["type"],
                            discovered_types=actual_types, problems=problems,
                            publisher_types=publication_types, publisher_count=len(publishers),
                            publisher_qos=publisher_qos.get(spec["topic"], []),
                            similar_topics=[name for name in sorted(discovered)
                                            if name.rsplit("/", 1)[-1].startswith(base)
                                            and name != spec["topic"]])
    return {"schema_version": 1, "transport_ok": transport_ok,
            "monitoring_complete": transport_ok and not missing_monitoring_topics,
            "missing_monitoring_topics": sorted(missing_monitoring_topics, key=lambda entry: entry["topic"]),
            "duration_seconds": duration,
            "arming_state": armed, "warnings": warnings, "topics": topics,
            "time_basis": "host monotonic receipt time; no end-to-end latency claim",
            "fingerprint": manifest.get("fingerprint")}


def write_report(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".probe-", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w") as stream:
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--json-output")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--require-core", dest="require_core", action="store_true", default=True)
    group.add_argument("--allow-missing-core", dest="require_core", action="store_false")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 1:
        parser.error("duration must be a finite number of at least one second")
    manifest = load_manifest(args.manifest)
    import rclpy
    from rclpy.qos_event import SubscriptionEventCallbacks
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[])
    node = rclpy.create_node("px4_monitor_probe")
    samples = {base: TopicSamples() for base in manifest["topics"]}
    errors = {}
    subscriptions = []
    try:
        for base, spec in manifest["topics"].items():
            try:
                cls = message_class(spec)
                state = samples[base]
                callback = lambda msg, key=base, stat=state: stat.record(key, msg, time.monotonic())
                def incompatible(event, stat=state):
                    stat.incompatible_qos += event.total_count_change
                subscriptions.append(node.create_subscription(
                    cls, spec["topic"], callback, qos_profile_sensor_data,
                    event_callbacks=SubscriptionEventCallbacks(incompatible_qos=incompatible)))
            except (ImportError, AttributeError, ValueError, RuntimeError) as error:
                errors[base] = str(error)
        started = time.monotonic()
        deadline = started + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        discovered = dict(node.get_topic_names_and_types())
        offered = {}
        for spec in manifest["topics"].values():
            offered[spec["topic"]] = [
                {"node": endpoint.node_name, "type": endpoint.topic_type,
                 "reliability": str(endpoint.qos_profile.reliability),
                 "durability": str(endpoint.qos_profile.durability)}
                for endpoint in node.get_publishers_info_by_topic(spec["topic"])]
        report = build_report(manifest, samples, discovered, offered, time.monotonic(),
                              time.monotonic() - started, errors)
        if args.json_output:
            write_report(args.json_output, report)
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
        return 0 if report["transport_ok"] or not args.require_core else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, ImportError) as error:
        print("monitor probe failed: " + str(error), file=sys.stderr)
        sys.exit(2)
