#!/usr/bin/env python3

import math
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class LCBRTracker(Node):

    def __init__(self):
        super().__init__('lcbr_tracker')

        # --------------------------------------------------
        # Load agent0 trajectory from final_lcbr.yaml
        # --------------------------------------------------
        package_dir = Path(__file__).resolve().parents[1]
        trajectory_file = (
            package_dir / 'trajectories' / 'final_lcbr.yaml'
        )

        if not trajectory_file.is_file():
            raise FileNotFoundError(
                f'Trajectory file not found: {trajectory_file}'
            )

        with trajectory_file.open('r') as f:
            data = yaml.safe_load(f)

        self.states = data['schedule']['agent0']

        # Initial pose in the LCBR / world frame
        self.x0 = float(self.states[0]['x'])
        self.y0 = float(self.states[0]['y'])
        self.theta0 = float(self.states[0]['yaw'])

        # Current transition:
        # states[k] -> states[k+1]
        self.k = 0

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
        # Controller parameters
        # --------------------------------------------------
        self.max_speed = 0.60
        self.min_speed = 0.10
        self.max_omega = 0.60

        self.k_heading = 1.5
        self.position_tolerance = 0.20

        self.finished = False

        self.get_logger().info(
            f'Loaded agent0: {len(self.states)} states, '
            f'{len(self.states) - 1} transitions.'
        )

        self.get_logger().info(
            f'Start pose: '
            f'({self.x0:.3f}, {self.y0:.3f}, '
            f'yaw={self.theta0:.4f})'
        )

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
        """
        Convert Gazebo odometry, which starts at (0,0,0),
        to the LCBR world frame.
        """

        xo = msg.pose.pose.position.x
        yo = msg.pose.pose.position.y

        theta_o = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        c = math.cos(self.theta0)
        s = math.sin(self.theta0)

        xw = self.x0 + c * xo - s * yo
        yw = self.y0 + s * xo + c * yo

        theta_w = self.wrap(
            self.theta0 + theta_o
        )

        return xw, yw, theta_w

    @staticmethod
    def classify_transition(a, b):
        """
        Infer FORWARD / REVERSE / WAIT from two consecutive
        LCBR states.
        """

        dx = float(b['x']) - float(a['x'])
        dy = float(b['y']) - float(a['y'])

        distance = math.hypot(dx, dy)

        if distance < 1e-4:
            return 'WAIT'

        yaw = float(a['yaw'])

        projection = (
            dx * math.cos(yaw)
            + dy * math.sin(yaw)
        )

        if projection >= 0.0:
            return 'FORWARD'

        return 'REVERSE'

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def odom_callback(self, msg):

        if self.finished:
            self.publish_stop()
            return

        # End of trajectory
        if self.k >= 1:
            self.publish_stop()
            self.finished = True

            self.get_logger().info(
                'LCBR trajectory completed.'
            )
            return

        # Actual pose expressed in LCBR world coordinates
        x, y, theta = self.odom_to_world(msg)

        current = self.states[self.k]
        target = self.states[self.k + 1]

        motion = self.classify_transition(
            current,
            target
        )

        tx = float(target['x'])
        ty = float(target['y'])

        dx = tx - x
        dy = ty - y

        distance = math.hypot(dx, dy)

        # --------------------------------------------------
        # Current target reached
        # --------------------------------------------------
        if distance < self.position_tolerance:

            self.get_logger().info(
                f't={current["t"]}->{target["t"]} '
                f'{motion} reached | '
                f'pose=({x:.2f},{y:.2f},{theta:.2f})'
            )

            self.k += 1
            self.publish_stop()
            return

        # --------------------------------------------------
        # WAIT
        # Temporal synchronization will be added later.
        # --------------------------------------------------
        if motion == 'WAIT':
            self.k += 1
            self.publish_stop()
            return

        # --------------------------------------------------
        # Desired direction of translation
        # --------------------------------------------------
        path_heading = math.atan2(
            dy,
            dx
        )

        if motion == 'FORWARD':

            desired_heading = path_heading
            direction = 1.0

        else:

            # In reverse, vehicle orientation is opposite
            # to the direction of translation.
            desired_heading = self.wrap(
                path_heading + math.pi
            )

            direction = -1.0

        heading_error = self.wrap(
            desired_heading - theta
        )

        # --------------------------------------------------
        # Steering command
        # --------------------------------------------------
        omega = (
            self.k_heading
            * heading_error
        )

        omega = max(
            -self.max_omega,
            min(self.max_omega, omega)
        )

        # --------------------------------------------------
        # Longitudinal command
        # --------------------------------------------------
        speed = min(
            self.max_speed,
            max(
                self.min_speed,
                0.5 * distance
            )
        )

        alignment = max(
            0.20,
            math.cos(heading_error)
        )

        speed *= alignment
        speed *= direction

        # --------------------------------------------------
        # Publish
        # --------------------------------------------------
        cmd = Twist()

        cmd.linear.x = float(speed)
        cmd.angular.z = float(omega)

        self.get_logger().info(
            f'k={self.k} {motion} | '
            f'actual=({x:.2f},{y:.2f},{theta:.2f}) | '
            f'target=({tx:.2f},{ty:.2f}) | '
            f'heading_err={heading_error:+.3f} | '
            f'v={speed:+.3f} omega={omega:+.3f}',
            throttle_duration_sec=0.5
        )

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = LCBRTracker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.publish_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
