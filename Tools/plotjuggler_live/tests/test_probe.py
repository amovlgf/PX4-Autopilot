from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from probe import CORE_TOPICS, CONTROLLER_TOPICS, MONITORING_TOPICS, TopicSamples, build_report


class ProbeTest(unittest.TestCase):
    def setUp(self):
        names = MONITORING_TOPICS
        self.manifest = {"topics": {name: {"topic": "/fmu/out/" + name, "type": "px4_msgs/msg/Type",
                                          "exported": True} for name in names}}
        self.samples = {name: TopicSamples() for name in names}
        self.message = SimpleNamespace(timestamp=1, q=[1, 0, 0, 0], x=0, y=0, z=-1, vx=0, vy=0, vz=0,
                                       xyz=[0, 0, 0], gyro_rad=[0, 0, 0], accelerometer_m_s2=[0, 0, -9.81],
                                       xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True, arming_state=1)

    def fill_core(self):
        for base in CORE_TOPICS | {"vehicle_status"}:
            for stamp, received in ((1, 10.0), (2, 10.02), (3, 10.04)):
                self.message.timestamp = stamp
                self.samples[base].record(base, self.message, received)

    def report(self, now=10.1, offered=None):
        if offered is None:
            offered = {spec["topic"]: [{"type": spec["type"]}] for spec in self.manifest["topics"].values()}
        return build_report(self.manifest, self.samples, {}, offered, now, 10.0)

    def test_no_data_is_failure_even_when_discovered(self):
        report = self.report()
        self.assertFalse(report["transport_ok"])
        self.assertIn("no_samples", report["topics"]["vehicle_attitude"]["problems"])

    def test_actual_rate_and_disarmed_controller_warning(self):
        self.fill_core()
        report = self.report()
        self.assertTrue(report["transport_ok"])
        self.assertFalse(report["monitoring_complete"])
        self.assertAlmostEqual(report["topics"]["vehicle_attitude"]["hz"], 50)
        self.assertTrue(any(warning.get("condition") == "disarmed" for warning in report["warnings"]))

    def test_old_px4_missing_angular_velocity_keeps_basic_attach(self):
        self.fill_core()
        offered = {self.manifest["topics"][base]["topic"]: [{"type": "px4_msgs/msg/Type"}]
                   for base in CORE_TOPICS | {"vehicle_status"}}
        report = self.report(offered=offered)
        self.assertTrue(report["transport_ok"])
        self.assertFalse(report["monitoring_complete"])
        angular = next(item for item in report["missing_monitoring_topics"]
                       if item["topic"] == "vehicle_angular_velocity")
        self.assertIn("no_publisher", angular["problems"])

    def test_own_subscription_is_not_a_publisher(self):
        discovered = {spec["topic"]: [spec["type"]] for spec in self.manifest["topics"].values()}
        report = build_report(self.manifest, self.samples, discovered, {}, 10.1, 10.0)
        self.assertFalse(report["transport_ok"])
        attitude = report["topics"]["vehicle_attitude"]
        self.assertEqual(attitude["publisher_count"], 0)
        self.assertIn("no_publisher", attitude["problems"])
        self.assertIn("no_samples", attitude["problems"])

    def test_present_publisher_without_samples_is_distinct(self):
        report = self.report()
        attitude = report["topics"]["vehicle_attitude"]
        self.assertEqual(attitude["publisher_count"], 1)
        self.assertNotIn("no_publisher", attitude["problems"])
        self.assertIn("no_samples", attitude["problems"])

    def test_all_expected_topics_must_receive_for_complete_monitoring(self):
        for base in MONITORING_TOPICS:
            for stamp, received in ((1, 10.0), (2, 10.02), (3, 10.04)):
                self.message.timestamp = stamp
                self.samples[base].record(base, self.message, received)
        report = self.report()
        self.assertTrue(report["transport_ok"])
        self.assertTrue(report["monitoring_complete"])
        self.assertEqual(report["missing_monitoring_topics"], [])

    def test_disarmed_torque_and_motors_have_explicit_condition(self):
        self.fill_core()
        report = self.report()
        for base in CONTROLLER_TOPICS:
            issue = next(item for item in report["missing_monitoring_topics"] if item["topic"] == base)
            self.assertEqual(issue["condition"], "disarmed")

    def test_stale_samples_fail(self):
        self.fill_core()
        self.assertFalse(self.report(now=13.0)["transport_ok"])

    def test_invalid_flags_are_explicit(self):
        self.fill_core()
        self.message.z_valid = False
        self.message.timestamp = 4
        self.samples["vehicle_local_position"].record("vehicle_local_position", self.message, 10.08)
        report = self.report()
        self.assertFalse(report["topics"]["vehicle_local_position"]["validity"]["z_valid"])
        self.assertIn("estimator_validity_flags_false", [warning["reason"] for warning in report["warnings"]])

    def test_incompatible_qos_and_type(self):
        self.fill_core()
        self.samples["vehicle_attitude"].incompatible_qos = 1
        offered = {"/fmu/out/vehicle_attitude": [{"type": "px4_msgs/msg/Wrong"}]}
        report = self.report(offered=offered)
        self.assertFalse(report["transport_ok"])
        self.assertIn("incompatible_message_type", report["topics"]["vehicle_attitude"]["problems"])
        self.assertIn("incompatible_qos", report["topics"]["vehicle_attitude"]["problems"])

    def test_duplicates_do_not_count_as_fresh_data(self):
        self.fill_core()
        self.samples["vehicle_attitude"] = TopicSamples()
        for received in (10.0, 10.02, 10.04):
            self.samples["vehicle_attitude"].record("vehicle_attitude", self.message, received)
        self.assertFalse(self.report()["transport_ok"])

    def test_nonfinite_quaternion_and_time_rollback(self):
        self.fill_core()
        self.message.q = [float("nan"), 0, 0, 0]
        self.message.timestamp = 1
        self.samples["vehicle_attitude"].record("vehicle_attitude", self.message, 10.08)
        result = self.report()["topics"]["vehicle_attitude"]
        self.assertEqual(result["invalid_samples"], 1)
        self.assertEqual(result["timestamp_rollbacks"], 1)


if __name__ == "__main__":
    unittest.main()
