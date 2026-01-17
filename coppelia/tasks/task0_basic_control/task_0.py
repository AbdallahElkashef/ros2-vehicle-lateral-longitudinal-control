#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

import time


class Task0(Node):
    def __init__(self):
        super().__init__("task_0")

        self.get_logger().info("Task_0 node has started")
        self.acc_pub = self.create_publisher(Float64, "cmd_vel", 10)
        self.brake_pub = self.create_publisher(Float64, "brakes", 10)

        acc_msg = Float64()
        acc_msg.data = 1.0
        self.acc_pub.publish(acc_msg)
        self.get_logger().info(f"Moving with velocity : {acc_msg.data}")

        time.sleep(2.0)

        brake_msg = Float64()
        brake_msg.data = 1.0
        self.brake_pub.publish(brake_msg)
        self.get_logger().info("Stopping")


def main(args=None):
    rclpy.init(args=args)
    node = Task0()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
