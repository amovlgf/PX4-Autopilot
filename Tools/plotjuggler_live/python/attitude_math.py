"""ROS-independent PX4 quaternion conversion and yaw continuity."""

import math

MAX_RECEIPT_GAP_SECONDS = 2.0


def quaternion_to_euler_deg(quaternion):
    """Convert Hamilton (w, x, y, z), body FRD to world NED, to degrees."""
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError("quaternion must contain four finite numbers")
    norm = math.sqrt(sum(v * v for v in quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion norm is zero or non-finite")
    w, x, y, z = (v / norm for v in quaternion)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class YawUnwrapper:
    """Start a new segment after reset, time rollback, or a reception gap.

    Duplicate timestamps are ignored, preserving the previous output. Resetting
    starts at the current wrapped yaw; it does not invent motion across a reset.
    """

    def __init__(self):
        self.last_timestamp = None
        self.last_reset_counter = None
        self.last_yaw = None
        self.unwrapped = None
        self.reset_reason = None
        self.last_received_at = None
        self.receipt_gap_seconds = None

    def update(self, yaw_deg, timestamp_us, reset_counter, received_at=None):
        if not math.isfinite(yaw_deg) or timestamp_us < 0:
            raise ValueError("invalid yaw or timestamp")
        if received_at is not None and not math.isfinite(received_at):
            raise ValueError("invalid monotonic reception time")
        self.receipt_gap_seconds = (received_at - self.last_received_at
                                    if received_at is not None and self.last_received_at is not None else None)
        self.last_received_at = received_at
        self.reset_reason = None
        if self.last_timestamp is None:
            self.reset_reason = "initial"
        elif self.receipt_gap_seconds is not None and self.receipt_gap_seconds > MAX_RECEIPT_GAP_SECONDS:
            self.reset_reason = "receipt_gap"
        elif reset_counter != self.last_reset_counter:
            self.reset_reason = "estimator_reset"
        elif timestamp_us < self.last_timestamp:
            self.reset_reason = "timestamp_rollback"
        elif timestamp_us == self.last_timestamp:
            return self.unwrapped
        if self.reset_reason:
            self.unwrapped = yaw_deg
        else:
            delta = (yaw_deg - self.last_yaw + 180.0) % 360.0 - 180.0
            self.unwrapped += delta
        self.last_yaw = yaw_deg
        self.last_timestamp = timestamp_us
        self.last_reset_counter = reset_counter
        return self.unwrapped
