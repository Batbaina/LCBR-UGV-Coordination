#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TrajectoryTracker(Node):

    def __init__(self):
        super().__init__('trajectory_tracker')

        # -----------------------------------------------------
        # ROS interfaces
        # -----------------------------------------------------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/ugv_1/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/ugv_1/odometry',
            self.odom_callback,
            10
        )

        # -----------------------------------------------------
        # Simple test goal
        # -----------------------------------------------------
        self.goal_x = 5.0
        self.goal_y = 0.0

        # -----------------------------------------------------
        # Controller parameters
        # -----------------------------------------------------
        self.k_linear = 0.6
        self.k_angular = 1.5

        self.max_linear = 1.0
        self.max_angular = 0.6

        self.goal_tolerance = 0.10

        self.finished = False

        self.get_logger().info(
            'Trajectory tracker started: goal = (5.0, 0.0)'
        )

    # ---------------------------------------------------------
    # Quaternion -> yaw
    # ---------------------------------------------------------
    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp)

    # ---------------------------------------------------------
    # Normalize angle to [-pi, pi]
    # ---------------------------------------------------------
    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ---------------------------------------------------------
    # Odometry callback / controller
    # ---------------------------------------------------------
    def odom_callback(self, msg):

        if self.finished:
            return

        # Actual position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Actual heading
        theta = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        # -----------------------------------------------------
        # Position error
        # -----------------------------------------------------
        dx = self.goal_x - x
        dy = self.goal_y - y

        distance = math.hypot(dx, dy)

        # Goal reached
        if distance < self.goal_tolerance:

            self.stop_robot()

            self.finished = True

            self.get_logger().info(
                f'Goal reached: x={x:.3f}, y={y:.3f}'
            )

            return

        # -----------------------------------------------------
        # Desired heading
        # -----------------------------------------------------
        desired_heading = math.atan2(dy, dx)

        heading_error = self.normalize_angle(
            desired_heading - theta
        )

        # -----------------------------------------------------
        # Simple feedback controller
        # -----------------------------------------------------
        v = self.k_linear * distance
        omega = self.k_angular * heading_error

        # Saturation
        v = max(
            -self.max_linear,
            min(self.max_linear, v)
        )

        omega = max(
            -self.max_angular,
            min(self.max_angular, omega)
        )

        # Slow down if heading error becomes large
        v *= max(0.0, math.cos(heading_error))

        # -----------------------------------------------------
        # Publish command
        # -----------------------------------------------------
        cmd = Twist()

        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)

        self.cmd_pub.publish(cmd)

    # ---------------------------------------------------------
    # Stop
    # ---------------------------------------------------------
    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryTracker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
