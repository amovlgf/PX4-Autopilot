import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from generate_layout import build_layout, generate


class LayoutTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.workspace = Path(self.temporary.name)
        self.manifest = {"fingerprint": "fixture", "interfaces": {}, "topics": {}}
        messages = {
            "vehicle_local_position": ("VehicleLocalPosition", "float32 x\nfloat32 y\nfloat32 z\n"
                                       "float32 vx\nfloat32 vy\nfloat32 vz\n"),
            "vehicle_angular_velocity": ("VehicleAngularVelocity", "float32[3] xyz\n"),
            "vehicle_rates_setpoint": ("VehicleRatesSetpoint", "float32 roll\nfloat32 pitch\nfloat32 yaw\n"),
            "sensor_combined": ("SensorCombined", "float32[3] gyro_rad\n"),
            "rate_ctrl_status": ("RateCtrlStatus", "float32 rollspeed_integ\nfloat32 pitchspeed_integ\n"
                                 "float32 yawspeed_integ\n"),
            "vehicle_torque_setpoint": ("VehicleTorqueSetpoint", "float32[3] xyz\n"),
            "actuator_motors": ("ActuatorMotors", "float32[12] control\n"),
        }
        for base, (message, content) in messages.items():
            relative = "msg/" + message + ".msg"
            path = self.workspace / "src/px4_msgs" / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
            self.manifest["interfaces"][relative] = {"sha256": hashlib.sha256(content.encode()).hexdigest()}
            suffix = "_v1" if base == "vehicle_local_position" else ""
            self.manifest["topics"][base] = {
                "topic": "/uav_1/fmu/out/" + base + suffix,
                "type": "px4_msgs/msg/" + message,
                "exported": True, "definition_available": True,
            }
        self.manifest_path = self.workspace / "manifest.json"
        self.manifest_path.write_text(json.dumps(self.manifest))

    def tearDown(self):
        self.temporary.cleanup()

    def test_layout_preserves_actual_namespace_version_and_ros_array_paths(self):
        root = build_layout(self.manifest, self.workspace)
        self.assertEqual([tab.attrib["tab_name"] for tab in root.findall("tabbed_widget/Tab")],
                         ["Attitude", "Rate Control", "Position", "PID Analysis"])
        curves = {curve.attrib["name"] for curve in root.iter("curve")}
        self.assertIn("/uav_1/fmu/out/vehicle_local_position_v1/x", curves)
        self.assertIn("/uav_1/fmu/out/vehicle_angular_velocity/xyz[0]", curves)
        self.assertIn("/px4_monitor/attitude_deg/vector/x", curves)
        self.assertIn("/px4_monitor/yaw_unwrapped_deg/data", curves)
        self.assertEqual(len(root.findall("tabbed_widget/Tab/Container/DockSplitter/DockArea")), 15)
        self.assertEqual(root.find("Plugins/plugin/use_header_stamp").attrib["value"], "false")
        # This matches PlotJuggler 3.17.2's actual native GUI Save result.
        # The CLI maximum must separately be larger than 60 to avoid =inf.
        self.assertEqual(root.find("streaming_buffer_size").attrib, {"value": "60"})
        self.assertEqual(root.find("previouslyLoaded_Streamer").attrib["name"], "ROS2 Topic Subscriber")

    def test_pid_analysis_contains_integrators_torque_and_four_raw_motor_curves(self):
        root = build_layout(self.manifest, self.workspace)
        pid = root.find("tabbed_widget/Tab[@tab_name='PID Analysis']")
        areas = pid.findall("Container/DockSplitter/DockArea")
        self.assertEqual(len(areas), 3)
        integrators = [curve.attrib["name"] for curve in areas[0].iter("curve")]
        self.assertEqual(integrators, ["/uav_1/fmu/out/rate_ctrl_status/" + field
                                      for field in ("rollspeed_integ", "pitchspeed_integ", "yawspeed_integ")])
        self.assertEqual([curve.attrib["name"] for curve in areas[1].iter("curve")],
                         ["/uav_1/fmu/out/vehicle_torque_setpoint/xyz[%d]" % i for i in range(3)])
        self.assertEqual([curve.attrib["name"] for curve in areas[2].iter("curve")],
                         ["/uav_1/fmu/out/actuator_motors/control[%d]" % i for i in range(4)])
        self.assertIn("NaN = disarmed", areas[2].attrib["name"])
        self.assertIn("normalized torque", areas[0].attrib["name"])
        # Direct curves have no transformations that could fill NaN samples.
        self.assertFalse(list(root.find("customMathEquations")))
        self.assertEqual(len(list(root.iter("curve"))), 29)
        proportions = pid.find("Container/DockSplitter").attrib["sizes"].split(";")
        self.assertEqual(len(proportions), 3)
        self.assertAlmostEqual(sum(map(float, proportions)), 1, places=5)

    def test_missing_exports_or_changed_fields_are_not_silently_plotted(self):
        manifest = copy.deepcopy(self.manifest)
        manifest["topics"]["vehicle_angular_velocity"]["exported"] = False
        with self.assertRaisesRegex(ValueError, "exported message definition"):
            build_layout(manifest, self.workspace)
        path = self.workspace / "src/px4_msgs/msg/VehicleAngularVelocity.msg"
        path.write_text("float32[3] new_field\n")
        with self.assertRaisesRegex(ValueError, "staged message was modified"):
            build_layout(self.manifest, self.workspace)
        self.manifest["interfaces"]["msg/VehicleAngularVelocity.msg"]["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, "message field is missing"):
            build_layout(self.manifest, self.workspace)

    def test_generation_preserves_user_saved_layout_and_records_unverified_state(self):
        output = self.workspace / "layouts/candidate.xml"
        with patch("generate_layout.load_manifest", return_value=self.manifest), \
                patch("generate_layout.installed_versions", return_value={}):
            result = generate(self.manifest_path, output)
            self.assertTrue(result["changed"])
            self.assertFalse(result["gui_verified"])
            ET.parse(str(output))
            metadata = json.loads(output.with_suffix(".sources.json").read_text())
            self.assertEqual(metadata["source_versions"]["plotjuggler"], "3.17.2")
            self.assertEqual(metadata["startup"]["maximum_expected_confirmations"], 2)
            self.assertEqual(metadata["startup"]["arguments"], ["--layout", str(output)])
            self.assertFalse(generate(self.manifest_path, output)["changed"])
            output.write_text("user saved layout")
            with self.assertRaisesRegex(ValueError, "already exists with different content"):
                generate(self.manifest_path, output)
            self.assertEqual(output.read_text(), "user saved layout")

    def test_unsupported_installed_version_requires_format_review(self):
        with patch("generate_layout.load_manifest", return_value=self.manifest), \
                patch("generate_layout.installed_versions", return_value={"plotjuggler": "9.0.0"}):
            with self.assertRaisesRegex(ValueError, "installed version"):
                generate(self.manifest_path, self.workspace / "candidate.xml")

    def test_unsupported_ros_plugin_version_requires_format_review(self):
        with patch("generate_layout.load_manifest", return_value=self.manifest), \
                patch("generate_layout.installed_versions",
                      return_value={"plotjuggler": "3.17.2", "plotjuggler_ros": "9.0.0"}):
            with self.assertRaisesRegex(ValueError, "plotjuggler_ros 2.3.1; installed version is 9.0.0"):
                generate(self.manifest_path, self.workspace / "candidate.xml")


if __name__ == "__main__":
    unittest.main()
