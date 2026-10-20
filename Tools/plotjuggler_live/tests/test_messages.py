import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from sync_messages import collect_interfaces, message_version, normalize_namespace, stage_workspace, topic_name
from monitor_common import load_manifest


class MessageSyncTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.px4 = self.root / "px4"
        (self.px4 / "msg/versioned").mkdir(parents=True)
        (self.px4 / "msg/px4_msgs_old/msg").mkdir(parents=True)
        (self.px4 / "srv").mkdir()
        self.dds = self.px4 / "src/modules/uxrce_dds_client/dds_topics.yaml"
        self.dds.parent.mkdir(parents=True)
        self.dds.write_text("publications:\n  - topic: /fmu/out/vehicle_attitude\n"
                            "    type: px4_msgs::msg::VehicleAttitude\n    rate_limit: 50.\n")
        (self.px4 / "msg/versioned/VehicleAttitude.msg").write_text(
            "uint32 MESSAGE_VERSION = 2\nuint64 timestamp\nfloat32[4] q\n")
        (self.px4 / "msg/Example.msg").write_text("uint64 timestamp\n")
        (self.px4 / "msg/px4_msgs_old/msg/VehicleAttitudeV1.msg").write_text("ignored")
        (self.px4 / "srv/Example.srv").write_text("Example request\n---\nExample reply\n")
        self.work = self.root / "work"

    def tearDown(self):
        self.temporary.cleanup()

    def test_version_and_namespace(self):
        self.assertEqual(message_version("uint32 MESSAGE_VERSION = 4 # comment\n"), 4)
        self.assertEqual(message_version("# uint32 MESSAGE_VERSION = 9\n"), 0)
        self.assertEqual(topic_name("vehicle_attitude", 0), "/fmu/out/vehicle_attitude")
        self.assertEqual(topic_name("vehicle_attitude", 2, "/uav_1/"), "/uav_1/fmu/out/vehicle_attitude_v2")
        self.assertEqual(topic_name("vehicle_imu", 1, "", 0), "/fmu/out/vehicle_imu0_v1")
        with self.assertRaises(ValueError):
            normalize_namespace("uav 1")

    def test_exports_current_messages_and_services(self):
        result = stage_workspace(self.px4, self.work, "uav_1")
        workspace = Path(result["workspace"])
        manifest = json.loads((workspace / "manifest.json").read_text())
        self.assertEqual(manifest["topics"]["vehicle_attitude"]["topic"], "/uav_1/fmu/out/vehicle_attitude_v2")
        self.assertTrue(manifest["topics"]["vehicle_attitude"]["exported"])
        self.assertFalse(manifest["topics"]["vehicle_angular_velocity"]["exported"])
        self.assertTrue((workspace / "src/px4_msgs/srv/Example.srv").exists())
        self.assertFalse((workspace / "src/px4_msgs/msg/VehicleAttitudeV1.msg").exists())
        self.assertFalse((self.work / "current").exists())

    def test_idempotent_and_definition_changes_get_new_workspace(self):
        first = stage_workspace(self.px4, self.work)
        second = stage_workspace(self.px4, self.work)
        self.assertFalse(second["changed"])
        self.assertEqual(first["workspace"], second["workspace"])
        (self.px4 / "msg/Example.msg").write_text("uint64 timestamp\nfloat32 changed\n")
        third = stage_workspace(self.px4, self.work)
        self.assertNotEqual(first["workspace"], third["workspace"])
        self.assertTrue(Path(first["workspace"]).exists())

    def test_identical_sources_at_different_mount_paths_keep_separate_manifests(self):
        host = stage_workspace(self.px4, self.work)
        host_manifest = Path(host["workspace"]) / "manifest.json"
        original_manifest = host_manifest.read_bytes()
        mounted_px4 = self.root / "container_mount/px4"
        shutil.copytree(self.px4, mounted_px4)
        container = stage_workspace(mounted_px4, self.work)
        self.assertNotEqual(host["fingerprint"], container["fingerprint"])
        self.assertNotEqual(host["workspace"], container["workspace"])
        self.assertTrue(container["changed"])
        self.assertEqual(host_manifest.read_bytes(), original_manifest)
        self.assertEqual(load_manifest(host_manifest)["source_root"], str(self.px4))
        container_manifest = Path(container["workspace"]) / "manifest.json"
        self.assertEqual(load_manifest(container_manifest)["source_root"], str(mounted_px4))
        repeated = stage_workspace(mounted_px4, self.work)
        self.assertFalse(repeated["changed"])
        self.assertEqual(container["workspace"], repeated["workspace"])
        # A container cannot access the host path; its own manifest must still
        # validate without changing or weakening source hash verification.
        self.px4.rename(self.root / "hidden_host_checkout")
        self.assertEqual(load_manifest(container_manifest)["source_root"], str(mounted_px4))
        self.assertEqual(host_manifest.read_bytes(), original_manifest)

    def test_refuses_to_overwrite_user_modified_workspace(self):
        result = stage_workspace(self.px4, self.work)
        altered = Path(result["workspace"]) / "src/px4_msgs/msg/Example.msg"
        altered.write_text("user edit")
        with self.assertRaises(ValueError):
            stage_workspace(self.px4, self.work)
        self.assertEqual(altered.read_text(), "user edit")

    def test_runtime_validation_accepts_unchanged_source(self):
        staged = stage_workspace(self.px4, self.work)
        manifest = load_manifest(Path(staged["workspace"]) / "manifest.json")
        self.assertEqual(manifest["fingerprint"], staged["fingerprint"])

    def test_runtime_validation_rejects_dds_change_without_rebuilding(self):
        staged = stage_workspace(self.px4, self.work)
        manifest_path = Path(staged["workspace"]) / "manifest.json"
        original = manifest_path.read_bytes()
        self.dds.write_text(self.dds.read_text().replace("50.", "100."))
        with self.assertRaisesRegex(ValueError, "DDS export configuration.*setup.sh"):
            load_manifest(manifest_path)
        self.assertEqual(manifest_path.read_bytes(), original)
        self.assertEqual(len(list((self.work / "workspaces").iterdir())), 1)

    def test_runtime_validation_rejects_definition_change_without_version_bump(self):
        staged = stage_workspace(self.px4, self.work)
        (self.px4 / "msg/Example.msg").write_text("uint64 timestamp\nfloat32 different\n")
        with self.assertRaisesRegex(ValueError, "msg/Example.msg.*setup.sh"):
            load_manifest(Path(staged["workspace"]) / "manifest.json")

    def test_runtime_validation_rejects_added_or_removed_messages(self):
        staged = stage_workspace(self.px4, self.work)
        manifest_path = Path(staged["workspace"]) / "manifest.json"
        added = self.px4 / "msg/Added.msg"
        added.write_text("uint64 timestamp\n")
        with self.assertRaisesRegex(ValueError, "message/service file set"):
            load_manifest(manifest_path)
        added.unlink()
        (self.px4 / "msg/Example.msg").unlink()
        with self.assertRaisesRegex(ValueError, "message/service file set"):
            load_manifest(manifest_path)

    def test_duplicate_current_definition_rejected(self):
        (self.px4 / "msg/VehicleAttitude.msg").write_text("uint64 timestamp\n")
        with self.assertRaises(ValueError):
            collect_interfaces(self.px4)


if __name__ == "__main__":
    unittest.main()
