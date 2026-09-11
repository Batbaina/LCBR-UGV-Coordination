#!/usr/bin/env python3

import math
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PrimitiveExecutor(Node):

    def __init__(self):
        super().__init__('primitive_executor')

        # --------------------------------------------------
        # Load LCBR trajectory
        # --------------------------------------------------
        package_dir = Path(__file__).resolve().parents[1]
        trajectory_file = (
            package_dir / 'trajectories' / 'final_lcbr.yaml'
        )

        with trajectory_file.open('r') as f:
            data = yaml.safe_load(f)

        self.states = data['schedule']['agent0']

        # Test only first 3 transitions
        self.max_transitions = 3
        self.k = 0
        self.finished = False

        # Initial LCBR/world pose
        self.x0 = float(self.states[0]['x'])
        self.y0 = float(self.states[0]['y'])
        self.theta0 = float(self.states[0]['yaw'])

        # --------------------------------------------------
        # Command parameters
        # --------------------------------------------------
        self.speed = 0.50

        # Initial nominal angular velocity.
        # R_SHA = 3 m => omega ~= v/R
        self.turn_omega = self.speed / 3.0

        # Completion tolerances
        self.position_tolerance = 0.30
        self.yaw_tolerance = math.radians(12.0)

        # --------------------------------------------------
        # ROS
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

        self.last_reported_k = -1

        self.get_logger().info(
            f'Loaded agent0: {len(self.states)} states.'
        )

        self.get_logger().info(
            f'Test limited to first {self.max_transitions} transitions.'
        )

        self.report_transition()

    # ------------------------------------------------------
    # Helpers
    # ------------------------------------------------------

    @staticmethod
    def wrap(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
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

    def odom_to_world(self, msg):

        xo = msg.pose.pose.position.x
        yo = msg.pose.pose.position.y

        theta_o = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        # Planner and Gazebo use opposite yaw conventions:
        # theta_G = -theta_P.
        #
        # Gazebo odometry is expressed relative to the spawn pose.
        # Convert it back to the planner / LCBR world frame.

        c = math.cos(self.theta0)
        s = math.sin(self.theta0)

        xw = self.x0 + c * xo + s * yo
        yw = self.y0 - s * xo + c * yo

        theta_w = self.wrap(
            self.theta0 - theta_o
        )

        return xw, yw, theta_w

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------
    # Primitive -> Gazebo command
    # ------------------------------------------------------

    def command_for_action(self, action):

        cmd = Twist()

        if action == 0:
            # forward straight
            cmd.linear.x = +self.speed
            cmd.angular.z = 0.0

        elif action == 1:
            # Planner yaw increases, Gazebo yaw must decrease.
            cmd.linear.x = +self.speed
            cmd.angular.z = -self.turn_omega

        elif action == 2:
            # Planner yaw decreases, Gazebo yaw must increase.
            cmd.linear.x = +self.speed
            cmd.angular.z = +self.turn_omega

        elif action == 3:
            # reverse straight
            cmd.linear.x = -self.speed
            cmd.angular.z = 0.0

        elif action == 4:
            # Planner yaw decreases, Gazebo yaw must increase.
            cmd.linear.x = -self.speed
            cmd.angular.z = +self.turn_omega

        elif action == 5:
            # Planner yaw increases, Gazebo yaw must decrease.
            cmd.linear.x = -self.speed
            cmd.angular.z = -self.turn_omega

        elif action == 6:
            # wait
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        else:
            raise RuntimeError(
                f'Unknown SHA* action: {action}'
            )

        return cmd

    # ------------------------------------------------------
    # Transition information
    # ------------------------------------------------------

    def report_transition(self):

        if self.k >= self.max_transitions:
            return

        state = self.states[self.k]
        target = self.states[self.k + 1]

        action = int(state['action'])
        motion = state.get('motion', 'unknown')

        self.get_logger().info(
            f'Transition {self.k}: '
            f't={state["t"]}->{target["t"]} | '
            f'action={action} ({motion}) | '
            f'target=({float(target["x"]):.3f}, '
            f'{float(target["y"]):.3f}, '
            f'yaw={float(target["yaw"]):.3f})'
        )

        self.last_reported_k = self.k

    # ------------------------------------------------------
    # Main loop
    # ------------------------------------------------------

    def odom_callback(self, msg):

        if self.finished:
            self.stop()
            return

        if self.k >= self.max_transitions:
            self.stop()
            self.finished = True

            self.get_logger().info(
                'Three-primitive test completed.'
            )
            return

        x, y, yaw = self.odom_to_world(msg)

        state = self.states[self.k]
        target = self.states[self.k + 1]

        action = int(state['action'])

        tx = float(target['x'])
        ty = float(target['y'])
        tyaw = self.wrap(float(target['yaw']))

        position_error = math.hypot(
            tx - x,
            ty - y
        )

        yaw_error = abs(
            self.wrap(tyaw - yaw)
        )

        # --------------------------------------------------
        # Target state reached
        # --------------------------------------------------
        if (
            position_error <= self.position_tolerance
            and
            yaw_error <= self.yaw_tolerance
        ):
            self.stop()

            self.get_logger().info(
                f'Transition {self.k} reached | '
                f'actual=({x:.3f},{y:.3f},{yaw:.3f}) | '
                f'ep={position_error:.3f} m | '
                f'eyaw={math.degrees(yaw_error):.2f} deg'
            )

            self.k += 1

            if self.k >= self.max_transitions:
                self.finished = True
                self.stop()

                self.get_logger().info(
                    'Three-primitive test completed.'
                )

                return

            self.report_transition()
            return

        # --------------------------------------------------
        # Execute the primitive directly
        # --------------------------------------------------
        cmd = self.command_for_action(action)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'k={self.k} action={action} | '
            f'actual=({x:.2f},{y:.2f},{yaw:.2f}) | '
            f'ep={position_error:.2f} | '
            f'eyaw={math.degrees(yaw_error):.1f} deg | '
            f'v={cmd.linear.x:+.2f} '
            f'omega={cmd.angular.z:+.3f}',
            throttle_duration_sec=0.5
        )


def main(args=None):

    rclpy.init(args=args)

    node = PrimitiveExecutor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
