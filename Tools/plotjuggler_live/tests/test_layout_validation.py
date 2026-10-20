import contextlib
import copy
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from validate_layout import main, validate_layout_topics


class LayoutValidationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.layout = Path(self.temporary.name) / "saved.xml"
        self.manifest = {
            "fingerprint": "1234567890abcdef", "topics": {
                "vehicle_local_position": {"topic": "/fmu/out/vehicle_local_position_v2", "exported": True},
                "vehicle_attitude": {"topic": "/fmu/out/vehicle_attitude", "exported": True},
            },
        }

    def write_layout(self, topics=(), curves=()):
        root = ET.Element("root")
        plugin = ET.SubElement(ET.SubElement(root, "Plugins"), "plugin", ID="ROS2 Topic Subscriber")
        ET.SubElement(plugin, "selected_topics", value=";".join(topics))
        for curve in curves:
            ET.SubElement(root, "curve", name=curve)
        ET.ElementTree(root).write(str(self.layout))

    def test_current_topics_and_custom_non_px4_curves_are_preserved(self):
        self.write_layout(["/fmu/out/vehicle_local_position_v2", "/camera/position"],
                          ["/fmu/out/vehicle_local_position_v2/x", "/px4_monitor/attitude_deg/vector/x",
                           "/camera/position/x"])
        original = self.layout.read_bytes()
        result = validate_layout_topics(self.manifest, self.layout)
        self.assertEqual(result["checked"], 1)
        self.assertFalse(result["unverified_px4_topics"])
        self.assertEqual(self.layout.read_bytes(), original)

    def test_old_subscription_version_fails_with_actual_expected_topic(self):
        self.write_layout(["/fmu/out/vehicle_local_position_v1"])
        with self.assertRaisesRegex(ValueError, "vehicle_local_position_v1 -> expected /fmu/out/vehicle_local_position_v2"):
            validate_layout_topics(self.manifest, self.layout)

    def test_stale_curve_fails_even_when_subscription_was_updated(self):
        self.write_layout(["/fmu/out/vehicle_local_position_v2"], ["/fmu/out/vehicle_local_position_v1/x"])
        with self.assertRaisesRegex(ValueError, "vehicle_local_position_v1"):
            validate_layout_topics(self.manifest, self.layout)

    def test_version_zero_to_nonzero_and_back_are_detected(self):
        self.write_layout(["/fmu/out/vehicle_local_position", "/fmu/out/vehicle_attitude_v1"])
        with self.assertRaises(ValueError) as error:
            validate_layout_topics(self.manifest, self.layout)
        self.assertIn("vehicle_local_position -> expected /fmu/out/vehicle_local_position_v2", str(error.exception))
        self.assertIn("vehicle_attitude_v1 -> expected /fmu/out/vehicle_attitude", str(error.exception))

    def test_namespace_change_is_detected(self):
        manifest = copy.deepcopy(self.manifest)
        manifest["topics"]["vehicle_attitude"]["topic"] = "/uav_1/fmu/out/vehicle_attitude"
        self.write_layout(["/fmu/out/vehicle_attitude"])
        with self.assertRaisesRegex(ValueError, "expected /uav_1/fmu/out/vehicle_attitude"):
            validate_layout_topics(manifest, self.layout)
        self.write_layout(["/uav_1/fmu/out/vehicle_attitude"], ["/uav_1/fmu/out/vehicle_attitude/q[0]"])
        self.assertEqual(validate_layout_topics(manifest, self.layout)["checked"], 1)

    def test_removed_export_is_reported(self):
        self.manifest["topics"]["vehicle_attitude"]["exported"] = False
        self.write_layout(["/fmu/out/vehicle_attitude"])
        with self.assertRaisesRegex(ValueError, "not exported"):
            validate_layout_topics(self.manifest, self.layout)

    def test_extra_px4_topics_are_explicitly_unverified_not_guessed(self):
        self.write_layout(["/fmu/out/custom_extension_v9"])
        result = validate_layout_topics(self.manifest, self.layout)
        self.assertEqual(result["unverified_px4_topics"], ["/fmu/out/custom_extension_v9"])

    def test_missing_layout_preserves_manual_selection(self):
        result = validate_layout_topics(self.manifest, self.layout)
        self.assertTrue(result["skipped"])

    def test_cli_failure_returns_nonzero_with_candidate_and_manual_repair(self):
        self.write_layout(["/fmu/out/vehicle_local_position_v1"])
        error = io.StringIO()
        with patch("validate_layout.load_manifest", return_value=self.manifest), contextlib.redirect_stderr(error):
            result = main(["--manifest", "/work/current/manifest.json", "--layout", str(self.layout)])
        self.assertEqual(result, 1)
        self.assertIn("generate_layout.py --manifest /work/current/manifest.json", error.getvalue())
        self.assertIn("candidate-1234567890ab.xml", error.getvalue())
        self.assertIn("PJ_LAYOUT=", error.getvalue())
        self.assertIn("Existing layouts are not overwritten", error.getvalue())


if __name__ == "__main__":
    unittest.main()
