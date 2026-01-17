#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from math import atan2, degrees
import time


class CDrive2(Node):
    def __init__(self):
        super().__init__("c_drive2")

        self.get_logger().info("c_drive2 node has started")

        # Parameters
        self.default_speed = 0.1
        self.timeout_spin = 0.1
        self.wheelbase = 2.269

        # Create Publishers
        self.vel_pub = self.create_publisher(Float64, "cmd_vel", 10)
        self.angle_pub = self.create_publisher(Float64, "SteeringAngle", 10)

    def wait_for_subscribers(self):
        self.get_logger().info("Waiting for subscribers...")
        while (
            self.vel_pub.get_subscription_count() == 0
            and self.angle_pub.get_subscription_count() == 0
        ):
            rclpy.spin_once(self, timeout_sec=self.timeout_spin)
        self.get_logger().info("Subscriber detected, publishing...")

    def compute_slip_angle(self, track_diameter):
        track_radius = track_diameter / 2
        slipAngle = atan2(self.wheelbase, track_radius)
        return degrees(slipAngle)

    def delay(self, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=self.timeout_spin)

    def set_speed(self, speed):
        msg = Float64()
        msg.data = speed
        self.vel_pub.publish(msg)
        self.get_logger().info(f"Set velocity to {speed}")

    def set_steering(self, angle):
        msg = Float64()
        msg.data = angle
        self.angle_pub.publish(msg)
        self.get_logger().info(f"Set steering angle to {angle}")

    def run(self):
        self.wait_for_subscribers()

        # Start moving
        self.set_speed(self.default_speed)

        # Step 1: Adjust angle to move in inner track
        self.inner_track_diameter = 20.6
        self.inner_slip_angle = self.compute_slip_angle(self.inner_track_diameter)
        self.set_steering(-self.inner_slip_angle)
        self.delay(0.5)

        # Step 2: Change lane with angle
        self.set_steering(0.0)
        self.delay(0.6)

        # Step 3: Correct Pose
        self.set_steering(-28.0)
        self.delay(0.4)

        # Step 4: Adjust angle to move in outer track
        self.outer_track_diameter = 29.0
        self.outer_slip_angle = self.compute_slip_angle(self.outer_track_diameter)
        self.set_steering(-self.outer_slip_angle)

        self.get_logger().info("Lane change sequence complete.")


def main(args=None):
    rclpy.init(args=args)
    node = CDrive2()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
