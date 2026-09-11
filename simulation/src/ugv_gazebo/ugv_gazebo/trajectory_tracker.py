#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TrajectoryTracker(Node):

    def __init__(self):
        super().__init__('trajectory_tracker')

        # --------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------
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

        # --------------------------------------------------
        # Test trajectory
        # --------------------------------------------------
        self.waypoints = [
            (0.0, 0.0),
            (3.0, 0.0),
            (5.0, 2.0),
            (5.0, 5.0),
        ]

        # Start with waypoint 1 because waypoint 0
        # corresponds to the initial robot position.
        self.current_index = 1

        # --------------------------------------------------
        # Controller
        # --------------------------------------------------
        self.k_linear = 0.6
        self.k_angular = 1.8

        self.max_linear = 0.5
        self.max_angular = 0.6

        self.waypoint_tolerance = 0.20
        self.final_tolerance = 0.10

        self.finished = False

        self.get_logger().info(
            f'Trajectory tracker started with '
            f'{len(self.waypoints)} waypoints.'
        )

        self.get_logger().info(
            f'First target: {self.waypoints[self.current_index]}'
        )

    # ------------------------------------------------------
    # Quaternion -> yaw
    # ------------------------------------------------------
    @staticmethod
    def quaternion_to_yaw(q):

        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ------------------------------------------------------
    # Normalize angle
    # ------------------------------------------------------
    @staticmethod
    def normalize_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # ------------------------------------------------------
    # Stop vehicle
    # ------------------------------------------------------
    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------
    # Main feedback loop
    # ------------------------------------------------------
    def odom_callback(self, msg):

        if self.finished:
            return

        # Actual pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        theta = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        # Current target waypoint
        goal_x, goal_y = self.waypoints[
            self.current_index
        ]

        dx = goal_x - x
        dy = goal_y - y

        distance = math.hypot(dx, dy)

        # --------------------------------------------------
        # Waypoint reached?
        # --------------------------------------------------
        is_last = (
            self.current_index ==
            len(self.waypoints) - 1
        )

        tolerance = (
            self.final_tolerance
            if is_last
            else self.waypoint_tolerance
        )

        if distance < tolerance:

            if is_last:

                self.stop_robot()
                self.finished = True

                self.get_logger().info(
                    f'Final waypoint reached: '
                    f'x={x:.3f}, y={y:.3f}'
                )

                return

            self.get_logger().info(
                f'Waypoint {self.current_index} reached '
                f'at x={x:.3f}, y={y:.3f}'
            )

            self.current_index += 1

            goal_x, goal_y = self.waypoints[
                self.current_index
            ]

            dx = goal_x - x
            dy = goal_y - y

            distance = math.hypot(dx, dy)

            self.get_logger().info(
                f'Next target: ({goal_x:.2f}, {goal_y:.2f})'
            )

        # --------------------------------------------------
        # Desired heading
        # --------------------------------------------------
        desired_heading = math.atan2(
            dy,
            dx
        )

        heading_error = self.normalize_angle(
            desired_heading - theta
        )

        # --------------------------------------------------
        # Feedback controller
        # --------------------------------------------------
        v = self.k_linear * distance
        omega = self.k_angular * heading_error

        # Velocity saturation
        v = min(
            self.max_linear,
            max(0.0, v)
        )

        omega = max(
            -self.max_angular,
            min(self.max_angular, omega)
        )

        # Reduce speed during large turns.
        heading_factor = max(
            0.0,
            math.cos(heading_error)
        )

        v *= heading_factor

        # Strong heading error: keep only a very small
        # forward velocity so the Ackermann vehicle
        # can recover without making a large loop.
        if abs(heading_error) > math.radians(60.0):
            v = min(v, 0.08)

        # --------------------------------------------------
        # Publish
        # --------------------------------------------------
        cmd = Twist()

        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)

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

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
