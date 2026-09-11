#!/usr/bin/env python3
import math
from pathlib import Path

import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PurePursuitTracker(Node):
    """Bidirectional path follower for a dense SHA*/LCBR reference."""

    def __init__(self):
        super().__init__('pure_pursuit_tracker')

        self.declare_parameter('agent', 'agent0')
        self.declare_parameter('trajectory_file', 'agent0_dense.yaml')
        self.declare_parameter('forward_speed', 0.35)
        self.declare_parameter('reverse_speed', -0.25)
        self.declare_parameter('lookahead_forward', 0.45)
        self.declare_parameter('lookahead_reverse', 0.40)
        self.declare_parameter('wheelbase', 1.30)
        self.declare_parameter('steering_limit', 0.448)
        self.declare_parameter('goal_position_tolerance', 0.12)
        self.declare_parameter('goal_yaw_tolerance_deg', 5.0)
        self.declare_parameter('max_tracking_error', 1.50)
        self.declare_parameter('start_index', 0)

        trajectory_name = self.get_parameter('trajectory_file').value
        share = Path(get_package_share_directory('ugv_gazebo'))
        trajectory_file = share / 'trajectories' / trajectory_name
        if not trajectory_file.exists():
            # Helpful when running directly from the source tree.
            source_fallback = Path(__file__).resolve().parents[1] / 'trajectories' / trajectory_name
            trajectory_file = source_fallback

        with trajectory_file.open('r') as f:
            data = yaml.safe_load(f)
        self.path = data['trajectory']

        self.start_index = int(self.get_parameter('start_index').value)
        if not 0 <= self.start_index < len(self.path) - 1:
            raise ValueError(f'Invalid start_index={self.start_index}')

        self.x0 = float(self.path[self.start_index]['x'])
        self.y0 = float(self.path[self.start_index]['y'])
        self.theta0 = float(self.path[self.start_index]['yaw'])

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.lookahead_forward = float(self.get_parameter('lookahead_forward').value)
        self.lookahead_reverse = float(self.get_parameter('lookahead_reverse').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.steering_limit = float(self.get_parameter('steering_limit').value)
        self.goal_position_tolerance = float(self.get_parameter('goal_position_tolerance').value)
        self.goal_yaw_tolerance = math.radians(
            float(self.get_parameter('goal_yaw_tolerance_deg').value))
        self.max_tracking_error = float(self.get_parameter('max_tracking_error').value)

        # Gazebo's AckermannSteering interprets Twist as (linear velocity,
        # yaw rate), computes turningRadius = v / omega, and clamps that
        # radius using steering_limit. Therefore limit omega consistently.
        self.minimum_turning_radius = self.wheelbase / math.sin(self.steering_limit)

        self.closest_index = self.start_index
        self.current_direction = self.path[self.start_index]['direction']
        # Final-goal overshoot protection.
        self.best_final_ep = float('inf')
        self.best_final_eyaw = float('inf')
        self.final_overshoot_margin = 0.03

        self.finished = False

        self.cmd_pub = self.create_publisher(Twist, '/ugv_1/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/model/ugv_1/odometry', self.odom_callback, 10)

        self.get_logger().info(
            f'Loaded {len(self.path)} dense points; start={self.start_index}; '
            f'direction={self.current_direction}; Rmin={self.minimum_turning_radius:.3f} m')

    @staticmethod
    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def quaternion_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def odom_to_planner(self, msg):
        xo = msg.pose.pose.position.x
        yo = msg.pose.pose.position.y
        theta_o = self.quaternion_to_yaw(msg.pose.pose.orientation)
        c = math.cos(self.theta0)
        s = math.sin(self.theta0)
        xp = self.x0 + c * xo + s * yo
        yp = self.y0 - s * xo + c * yo
        theta_p = self.wrap(self.theta0 - theta_o)
        return xp, yp, theta_p

    def stop(self):
        self.cmd_pub.publish(Twist())

    def find_closest_index(self, x, y):
        end = min(len(self.path), self.closest_index + 60)
        best_index = self.closest_index
        best_distance = float('inf')
        for i in range(self.closest_index, end):
            d = math.hypot(float(self.path[i]['x']) - x,
                           float(self.path[i]['y']) - y)
            if d < best_distance:
                best_distance = d
                best_index = i
        self.closest_index = best_index
        return best_index, best_distance

    def find_direction_boundary(self, start_index):
        direction = self.path[start_index]['direction']
        for i in range(start_index + 1, len(self.path)):
            if self.path[i]['direction'] != direction:
                return i
        return len(self.path) - 1

    def find_lookahead(self, x, y, closest, direction, boundary):
        lookahead = self.lookahead_forward if direction == 'forward' else self.lookahead_reverse
        for i in range(closest, min(boundary + 1, len(self.path))):
            if math.hypot(float(self.path[i]['x']) - x,
                          float(self.path[i]['y']) - y) >= lookahead:
                return i
        return boundary

    def odom_callback(self, msg):
        if self.finished:
            self.stop()
            return

        x, y, theta = self.odom_to_planner(msg)
        goal = self.path[-1]
        gx, gy = float(goal['x']), float(goal['y'])
        gyaw = self.wrap(float(goal['yaw']))
        final_ep = math.hypot(gx - x, gy - y)
        final_eyaw = abs(self.wrap(gyaw - theta))

        # --------------------------------------------------
        # Final-goal overshoot protection
        # --------------------------------------------------
        # Near the end of the reference, remember the closest
        # approach to the exact SHA*/LCBR goal. If the distance
        # starts increasing again, the Ackermann vehicle has
        # passed its closest achievable point on the final
        # primitive. Stop immediately instead of continuing
        # away from the goal until TRACKING LOST.
        if self.closest_index >= len(self.path) - 20:

            if final_ep < self.best_final_ep:
                self.best_final_ep = final_ep
                self.best_final_eyaw = final_eyaw

            elif (
                self.best_final_ep <= 0.25
                and
                final_ep >
                self.best_final_ep + self.final_overshoot_margin
            ):
                self.stop()
                self.finished = True

                self.get_logger().info(
                    f'AGENT COMPLETE - OVERSHOOT GUARD | '
                    f'best_ep={self.best_final_ep:.3f} m | '
                    f'best_eyaw='
                    f'{math.degrees(self.best_final_eyaw):.2f} deg | '
                    f'current_ep={final_ep:.3f} m'
                )
                return

        if (self.closest_index >= len(self.path) - 3 and
                final_ep <= self.goal_position_tolerance and
                final_eyaw <= self.goal_yaw_tolerance):
            self.stop()
            self.finished = True
            self.get_logger().info(
                f'AGENT COMPLETE | ep={final_ep:.3f} m | '
                f'eyaw={math.degrees(final_eyaw):.2f} deg')
            return

        closest, tracking_error = self.find_closest_index(x, y)
        if tracking_error > self.max_tracking_error:
            self.stop()
            self.finished = True
            self.get_logger().error(
                f'TRACKING LOST - STOPPING | idx={closest} | error={tracking_error:.3f} m')
            return

        direction = self.path[closest]['direction']
        if direction == 'goal':
            # The final dense sample is a marker, not a motion mode. Keep
            # following the final real primitive until the pose tolerance
            # above is satisfied. This avoids stopping short at index 700.
            direction = self.path[-2]['direction']
            closest = len(self.path) - 2
            self.closest_index = closest

        if direction == 'wait':
            self.stop()
            self.closest_index = min(closest + 1, len(self.path) - 1)
            return

        if direction != self.current_direction:
            self.stop()
            self.get_logger().info(
                f'DIRECTION SWITCH: {self.current_direction} -> {direction} at {closest}')
            self.current_direction = direction
            return

        boundary = self.find_direction_boundary(closest)
        target_index = self.find_lookahead(x, y, closest, direction, boundary)
        target = self.path[target_index]
        dx = float(target['x']) - x
        dy = float(target['y']) - y
        target_heading = math.atan2(dy, dx)

        speed = self.forward_speed if direction == 'forward' else self.reverse_speed

        # On the last real primitive, reduce speed continuously with the
        # remaining goal distance while preserving the SAME path-following
        # law. Do not replace the SHA*/LCBR curve with point chasing.
        if closest >= len(self.path) - 20:
            sign = 1.0 if speed > 0.0 else -1.0
            terminal_mag = max(0.035, min(abs(speed), 0.45 * final_ep))
            speed = sign * terminal_mag
        # Planner uses theta_P = -theta_G. For reverse, keeping the body
        # heading here and the negative speed gives the correct signed omega.
        geometric_heading = -theta
        alpha = self.wrap(target_heading - geometric_heading)
        ld = max(math.hypot(dx, dy), 1e-3)
        curvature = 2.0 * math.sin(alpha) / ld
        omega = speed * curvature

        # Match Gazebo Sim 8 AckermannSteering exactly: R = v / omega and
        # Rmin = wheel_base / sin(steering_limit).
        omega_max = abs(speed) / self.minimum_turning_radius
        omega = max(-omega_max, min(omega_max, omega))

        # Slow near a forward/reverse boundary only (not every curvature flip).
        bx, by = float(self.path[boundary]['x']), float(self.path[boundary]['y'])
        boundary_distance = math.hypot(bx - x, by - y)
        if boundary_distance < 0.70:
            sign = 1.0 if speed > 0 else -1.0
            speed = sign * min(abs(speed), max(0.10, 0.55 * boundary_distance))
            omega_max = abs(speed) / self.minimum_turning_radius
            omega = max(-omega_max, min(omega_max, speed * curvature))

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'idx={closest:03d}->{target_index:03d} [{direction}] | '
            f'etrack={tracking_error:.2f} | goal={final_ep:.2f} | '
            f'alpha={math.degrees(alpha):+.1f}deg | v={speed:+.2f} | '
            f'omega={omega:+.3f}/{omega_max:.3f}',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTracker()
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
