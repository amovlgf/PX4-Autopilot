#!/usr/bin/env python3
"""Subscribe to PX4 attitude and publish plotting-only Euler angle channels."""

import argparse
import sys
import time

from attitude_math import quaternion_to_euler_deg, YawUnwrapper
from monitor_common import load_manifest, message_class


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    args = parser.parse_args()
    manifest = load_manifest(args.manifest)
    spec = manifest["topics"]["vehicle_attitude"]
    attitude_type = message_class(spec)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Vector3Stamped
    from std_msgs.msg import Float64

    class AttitudeMonitor(Node):
        def __init__(self):
            super().__init__("px4_monitor_attitude")
            self.unwrap = YawUnwrapper()
            self.invalid_count = 0
            self.euler_pub = self.create_publisher(Vector3Stamped, "/px4_monitor/attitude_deg", 10)
            self.yaw_pub = self.create_publisher(Float64, "/px4_monitor/yaw_unwrapped_deg", 10)
            self.subscription = self.create_subscription(
                attitude_type, spec["topic"], self.on_attitude, qos_profile_sensor_data)
            self.get_logger().info("Read-only attitude subscription: " + spec["topic"])

        def on_attitude(self, message):
            try:
                roll, pitch, yaw = quaternion_to_euler_deg(message.q)
                unwrapped = self.unwrap.update(yaw, message.timestamp, message.quat_reset_counter,
                                               received_at=time.monotonic())
            except (ValueError, TypeError, OverflowError) as error:
                self.invalid_count += 1
                if self.invalid_count == 1 or self.invalid_count % 100 == 0:
                    self.get_logger().warning("Rejected invalid attitude sample: " + str(error))
                return
            if self.unwrap.reset_reason == "receipt_gap":
                self.get_logger().warning("Started new yaw segment after %.3f s without valid attitude" %
                                          self.unwrap.receipt_gap_seconds)
            elif self.unwrap.reset_reason not in (None, "initial"):
                self.get_logger().warning("Started new yaw segment: " + self.unwrap.reset_reason)
            output = Vector3Stamped()
            # Receipt time keeps this aligned with the ROS2 PlotJuggler streamer.
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = "px4_ned_frd_euler_degrees"
            output.vector.x, output.vector.y, output.vector.z = roll, pitch, yaw
            self.euler_pub.publish(output)
            continuous_yaw = Float64()
            continuous_yaw.data = unwrapped
            self.yaw_pub.publish(continuous_yaw)

    rclpy.init(args=[])
    node = AttitudeMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, ImportError) as error:
        print("attitude monitor failed: " + str(error), file=sys.stderr)
        sys.exit(1)
