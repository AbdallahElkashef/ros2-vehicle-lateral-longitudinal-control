#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer
from my_own_action.action import ReachTarget

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

import time
from math import sqrt, atan2, pi


class Task2PID(Node):
    def __init__(self):
        super().__init__("task2_pid")
        self.get_logger().info("task2_pid node has started")

        # Initiating Parameters:
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_speed_x = 0.0
        self.current_speed_y = 0.0
        self.current_angle = 0.0
        self.desired_speed = 10.0
        # PID Parameters
        self.last_time_throttle, self.last_time_brakes, self.last_time_steering = (
            time.monotonic(),
            time.monotonic(),
            time.monotonic(),
        )
        self.integral_throttle, self.prev_throttle_error = 0.0, 0.0
        self.integral_brake, self.prev_brake_error = (
            0.0,
            self.desired_speed,
        )
        self.integral_steering, self.prev_steering_error = 0.0, 0.0

        # Create Publishers
        self.throttle_pub = self.create_publisher(Float64, "cmd_vel", 10)
        self.brake_pub = self.create_publisher(Float64, "brakes", 10)
        self.steer_pub = self.create_publisher(Float64, "SteeringAngle", 10)

        # Create Subscribers
        self.subscriber_ = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Create Action Server
        self._action_server = ActionServer(
            self, ReachTarget, "reach_target", execute_callback=self.execute_callback
        )

    def delay(self, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=duration)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_speed_x = msg.twist.twist.linear.x
        self.current_speed_y = msg.twist.twist.linear.y
        # Quaternion 2 Euler
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        self.current_angle = yaw

    def anti_windup(self, output, integral, saturation_value):
        saturated_output = max(min(output, saturation_value), -saturation_value)
        if output != saturated_output:
            # Output was clipped, so don't accumulate more integral
            integral = 0.0

        return saturated_output, integral

    def PID(self, error, dt, previous, integral, kp, ki, kd, saturation_value):
        proportional = error
        integral += error * dt
        derivative = (error - previous) / dt
        previous = error
        output = (kp * proportional) + (ki * integral) + (kd * derivative)
        output, integral = self.anti_windup(output, integral, saturation_value)
        return output, integral, previous

    def set_throttle(self, speed_error):
        max_throttle = 1.0
        # PID Values
        kp = 2.0
        ki = 0.1
        kd = 0.001
        now = time.monotonic()
        dt = now - self.last_time_throttle
        dt = max(dt, 1e-3)
        output, self.integral_throttle, self.prev_throttle_error = self.PID(
            speed_error,
            dt,
            self.prev_throttle_error,
            self.integral_throttle,
            kp,
            ki,
            kd,
            max_throttle,
        )
        self.last_time_throttle = time.monotonic()
        msg = Float64()
        msg.data = max(output, 0.0)
        self.throttle_pub.publish(msg)
        self.get_logger().info(f"Set throttle to {msg.data}")

    def set_brakes(self, error):
        max_brakes = 1.0
        # PID Values
        kp = 1e-100
        ki = 0.0
        kd = 0.0
        now = time.monotonic()
        dt = now - self.last_time_brakes
        dt = max(dt, 1e-3)
        output, self.integral_brake, self.prev_brake_error = self.PID(
            error,
            dt,
            self.prev_brake_error,
            self.integral_brake,
            kp,
            ki,
            kd,
            max_brakes,
        )
        self.last_time_brakes = time.monotonic()
        msg = Float64()
        msg.data = abs(output)
        self.brake_pub.publish(msg)
        self.get_logger().info(f"Set brakes to {msg.data}")

    def set_steering(self, target_x):
        max_steering = 30.0
        # PID Values
        kp = 10.0
        ki = 0.01
        kd = 25.0
        distance_x = -(target_x - self.current_x)
        # Angle Handling
        now = time.monotonic()
        dt = now - self.last_time_steering
        dt = max(dt, 1e-3)
        output, self.integral_steering, self.prev_steering_error = self.PID(
            distance_x,
            dt,
            self.prev_steering_error,
            self.integral_steering,
            kp,
            ki,
            kd,
            max_steering,
        )
        self.last_time_steering = time.monotonic()
        msg = Float64()
        msg.data = output  # Steering Angle is reversed
        self.steer_pub.publish(msg)
        self.get_logger().info(f"Set steering angle to {msg.data}")

    def execute_callback(self, goal_handle):
        # Action Msg Handling
        goal_msg = goal_handle.request.target_distance
        self.get_logger().info(
            "Received target distance to the wall: {0}".format(goal_msg)
        )
        feedback_msg = ReachTarget.Feedback()
        result_msg = ReachTarget.Result()
        remaining = 0.0
        stopping_distance = 10

        while True:
            remaining = goal_msg - self.current_y
            current_speed = sqrt(self.current_speed_x**2 + self.current_speed_y**2)

            # Exit only when at wall AND stopped
            if remaining <= 0.0 and current_speed < 5e-4:
                break

            # Longitudinal Distance Feedback
            remaining = goal_msg - self.current_y
            feedback_msg.remaining_distance = remaining
            # Speed Feedback
            current_speed = sqrt(self.current_speed_x**2 + self.current_speed_y**2)
            feedback_msg.current_speed = current_speed
            # Lateral Distance Feedback
            feedback_msg.current_x = self.current_x
            # Publish Feedback
            goal_handle.publish_feedback(feedback_msg)

            # Lateral Control
            self.set_steering(0.0)

            # Longitudinal Control
            speed_error = 0.0
            if remaining > stopping_distance:
                # Maintain Speed
                speed_error = self.desired_speed - current_speed
                if speed_error >= 0.0:
                    self.set_brakes(0.0)
                    self.set_throttle(speed_error)
                else:
                    self.set_throttle(0.0)
                    self.set_brakes(abs(speed_error))
            else:
                # Final lock phase
                self.set_throttle(0.0)
                self.set_brakes(1e100)

            self.delay(0.1)

        self.set_throttle(0.0)
        self.set_brakes(1.0)
        result_msg.success = True
        goal_handle.succeed()
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    node = Task2PID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()