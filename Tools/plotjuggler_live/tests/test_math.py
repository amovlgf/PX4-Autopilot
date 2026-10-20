import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from attitude_math import quaternion_to_euler_deg, YawUnwrapper


class AttitudeMathTest(unittest.TestCase):
    def test_identity_and_normalization(self):
        self.assertEqual(quaternion_to_euler_deg([2, 0, 0, 0]), (0, 0, 0))

    def test_hamilton_axis_rotations(self):
        for axis in range(1, 4):
            q = [math.sqrt(0.5), 0, 0, 0]
            q[axis] = math.sqrt(0.5)
            self.assertAlmostEqual(quaternion_to_euler_deg(q)[axis - 1], 90, places=6)

    def test_quaternion_sign_does_not_change_attitude(self):
        q = [0.7, 0.2, 0.1, 0.3]
        self.assertEqual(quaternion_to_euler_deg(q), quaternion_to_euler_deg([-v for v in q]))

    def test_invalid_quaternions(self):
        for q in ([0, 0, 0, 0], [1, 0, 0], [math.nan, 0, 0, 1], [math.inf, 0, 0, 1]):
            with self.assertRaises(ValueError):
                quaternion_to_euler_deg(q)

    def test_yaw_crosses_positive_boundary(self):
        unwrap = YawUnwrapper()
        self.assertEqual(unwrap.update(179, 1, 0), 179)
        self.assertEqual(unwrap.update(-179, 2, 0), 181)
        self.assertEqual(unwrap.update(-170, 3, 0), 190)

    def test_yaw_crosses_negative_boundary(self):
        unwrap = YawUnwrapper()
        unwrap.update(-179, 1, 0)
        self.assertEqual(unwrap.update(179, 2, 0), -181)

    def test_estimator_reset_starts_new_segment(self):
        unwrap = YawUnwrapper()
        unwrap.update(179, 10, 255)
        self.assertEqual(unwrap.update(-179, 11, 0), -179)
        self.assertEqual(unwrap.reset_reason, "estimator_reset")

    def test_reception_gap_resets_even_when_px4_time_and_counter_do_not(self):
        unwrap = YawUnwrapper()
        unwrap.update(179, 1000000, 1, received_at=10.0)
        self.assertEqual(unwrap.update(-179, 1020000, 1, received_at=10.02), 181)
        self.assertEqual(unwrap.update(-170, 5000000, 1, received_at=13.0), -170)
        self.assertEqual(unwrap.reset_reason, "receipt_gap")
        self.assertAlmostEqual(unwrap.receipt_gap_seconds, 2.98)
        self.assertEqual(unwrap.update(-160, 5020000, 1, received_at=13.02), -160)
        self.assertIsNone(unwrap.reset_reason)

    def test_two_second_gap_does_not_reset(self):
        unwrap = YawUnwrapper()
        unwrap.update(179, 1000000, 0, received_at=10.0)
        self.assertEqual(unwrap.update(-179, 3000000, 0, received_at=12.0), 181)
        self.assertIsNone(unwrap.reset_reason)

    def test_rollback_and_duplicate(self):
        unwrap = YawUnwrapper()
        unwrap.update(179, 10, 0)
        self.assertEqual(unwrap.update(-179, 10, 0), 179)
        self.assertEqual(unwrap.update(-170, 9, 0), -170)
        self.assertEqual(unwrap.reset_reason, "timestamp_rollback")


if __name__ == "__main__":
    unittest.main()
