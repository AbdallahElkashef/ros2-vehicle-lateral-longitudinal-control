#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from math import atan2, degrees


class CDrive0(Node):
    def __init__(self):
        super().__init__("c_drive0")

        self.get_logger().info("C_drive0 node has started")
        self.vel_pub = self.create_publisher(Float64, "cmd_vel", 10)
        self.angle_pub = self.create_publisher(Float64, "SteeringAngle", 10)

        # Wait until at least one subscriber is connected
        self.get_logger().info("Waiting for subscribers...")
        while (
            self.vel_pub.get_subscription_count() == 0
            and self.angle_pub.get_subscription_count() == 0
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Subscriber detected, publishing...")

        #Computing Slip Angle
        track_diameter = 20.6
        track_radius = track_diameter / 2
        wheelbase = 2.269
        slipAngle = atan2(wheelbase, track_radius)

        #Publishing
        speed_msg = Float64()
        steering_msg = Float64()

        speed_msg.data = 0.05  # example speed
        steering_msg.data = -degrees(slipAngle)  # steering for inner lane

        self.vel_pub.publish(speed_msg)
        self.angle_pub.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CDrive0()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
