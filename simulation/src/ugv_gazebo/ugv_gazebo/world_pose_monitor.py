#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class WorldPoseMonitor(Node):

    def __init__(self):
        super().__init__('world_pose_monitor')

        # Initial pose of agent0 in the LCBR/world frame
        self.x0 = 12.0
        self.y0 = 4.0
        self.theta0 = 1.5708

        self.subscription = self.create_subscription(
            Odometry,
            '/model/ugv_1/odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'World-pose monitor started.'
        )

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

    @staticmethod
    def wrap(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def odom_callback(self, msg):

        # Pose in Gazebo odometry frame
        xo = msg.pose.pose.position.x
        yo = msg.pose.pose.position.y

        theta_o = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        # Odom -> LCBR/world transformation
        c = math.cos(self.theta0)
        s = math.sin(self.theta0)

        xw = self.x0 + c * xo - s * yo
        yw = self.y0 + s * xo + c * yo

        theta_w = self.wrap(
            self.theta0 + theta_o
        )

        self.get_logger().info(
            f'ODOM=({xo:+.3f}, {yo:+.3f}, {theta_o:+.3f})  '
            f'WORLD=({xw:+.3f}, {yw:+.3f}, {theta_w:+.3f})',
            throttle_duration_sec=1.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = WorldPoseMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
