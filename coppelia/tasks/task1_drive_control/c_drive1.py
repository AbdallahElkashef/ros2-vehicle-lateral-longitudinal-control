#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

import time


class CDrive1(Node):
    def __init__(self):
        super().__init__("c_drive1")

        # Parameters
        self.default_speed = 1.0
        self.timeout_spin = 0.1

        self.get_logger().info("c_drive1 node has started")

        #Create Publishers
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

        # Start moving forward
        self.set_speed(self.default_speed)
        self.delay(0.5)

        # Step 1: Change lane with angle
        self.set_steering(3.0)
        self.delay(0.5)

        # Step 2: Straighten
        self.set_steering(0.0)
        self.delay(0.3)

        # Step 3: Reverse angle to complete lane change
        self.set_steering(-2.8)
        self.delay(0.2)

        # Step 4: Straighten
        self.set_steering(0.0)
        
        self.get_logger().info("Lane change sequence complete.")


def main(args=None):
    rclpy.init(args=args)
    node = CDrive1()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
