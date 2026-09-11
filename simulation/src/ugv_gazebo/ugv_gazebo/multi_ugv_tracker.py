#!/usr/bin/env python3

import math
from pathlib import Path

import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


NUM_AGENTS = 6


class AgentTracker:

    def __init__(
        self,
        node,
        agent_index,
        path,
        forward_speed,
        reverse_speed,
        lookahead_forward,
        lookahead_reverse,
        wheelbase,
        steering_limit,
        goal_position_tolerance,
        goal_yaw_tolerance,
        max_tracking_error,
    ):

        self.node = node

        self.agent_index = agent_index
        self.agent_name = f'agent{agent_index}'
        self.ugv_name = f'ugv_{agent_index + 1}'

        self.path = path

        # --------------------------------------------------
        # Planner reference pose = Gazebo spawn pose
        # --------------------------------------------------

        self.x0 = float(path[0]['x'])
        self.y0 = float(path[0]['y'])
        self.theta0 = float(path[0]['yaw'])

        # --------------------------------------------------
        # Controller parameters
        # --------------------------------------------------

        self.forward_speed = forward_speed
        self.reverse_speed = reverse_speed

        self.lookahead_forward = lookahead_forward
        self.lookahead_reverse = lookahead_reverse

        self.wheelbase = wheelbase
        self.steering_limit = steering_limit

        self.goal_position_tolerance = (
            goal_position_tolerance
        )

        self.goal_yaw_tolerance = (
            goal_yaw_tolerance
        )

        self.max_tracking_error = (
            max_tracking_error
        )

        self.minimum_turning_radius = (
            self.wheelbase
            / math.sin(self.steering_limit)
        )

        # --------------------------------------------------
        # Tracking state
        # --------------------------------------------------

        self.closest_index = 0

        self.current_direction = (
            self.first_motion_direction()
        )

        self.best_final_ep = float('inf')
        self.best_final_eyaw = float('inf')

        self.final_overshoot_margin = 0.03

        self.finished = False
        self.failed = False

        self.latest_odom = None

        # Latest Gazebo simulation timestamp received through odometry.
        # All six odometry messages originate from the same Gazebo world,
        # therefore they share the same simulation-time basis.
        self.latest_sim_time = None

        # WAIT bookkeeping
        self.waiting = False
        self.wait_until = None

        # --------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------

        self.cmd_pub = node.create_publisher(
            Twist,
            f'/{self.ugv_name}/cmd_vel',
            10
        )

        self.odom_sub = node.create_subscription(
            Odometry,
            f'/model/{self.ugv_name}/odometry',
            self.odom_callback,
            10
        )

    # ======================================================
    # Basic geometry
    # ======================================================

    @staticmethod
    def wrap(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    @staticmethod
    def quaternion_to_yaw(q):

        siny = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy = (
            1.0
            - 2.0 * (
                q.y * q.y
                + q.z * q.z
            )
        )

        return math.atan2(
            siny,
            cosy
        )

    def odom_to_planner(self, msg):

        xo = msg.pose.pose.position.x
        yo = msg.pose.pose.position.y

        theta_o = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        c = math.cos(self.theta0)
        s = math.sin(self.theta0)

        xp = (
            self.x0
            + c * xo
            + s * yo
        )

        yp = (
            self.y0
            - s * xo
            + c * yo
        )

        theta_p = self.wrap(
            self.theta0 - theta_o
        )

        return xp, yp, theta_p

    # ======================================================
    # ROS
    # ======================================================

    def odom_callback(self, msg):
        self.latest_odom = msg

        stamp = msg.header.stamp

        self.latest_sim_time = (
            float(stamp.sec)
            + float(stamp.nanosec) * 1e-9
        )

    def stop(self):
        self.cmd_pub.publish(
            Twist()
        )

    # ======================================================
    # Path utilities
    # ======================================================

    def first_motion_direction(self):

        for p in self.path:

            direction = p['direction']

            if direction in (
                'forward',
                'reverse'
            ):
                return direction

        return 'forward'

    def find_closest_index(
        self,
        x,
        y,
        max_index=None
    ):

        end = min(
            len(self.path),
            self.closest_index + 60
        )

        if max_index is not None:
            end = min(
                end,
                max_index + 1
            )

        best_index = self.closest_index
        best_distance = float('inf')

        for i in range(
            self.closest_index,
            end
        ):

            d = math.hypot(
                float(self.path[i]['x']) - x,
                float(self.path[i]['y']) - y
            )

            if d < best_distance:
                best_distance = d
                best_index = i

        self.closest_index = best_index

        return (
            best_index,
            best_distance
        )

    def find_direction_boundary(
        self,
        start_index,
        max_index
    ):

        direction = (
            self.path[start_index]
            ['direction']
        )

        for i in range(
            start_index + 1,
            min(
                len(self.path),
                max_index + 1
            )
        ):

            if (
                self.path[i]['direction']
                != direction
            ):
                return i

        return min(
            max_index,
            len(self.path) - 1
        )

    def find_lookahead(
        self,
        x,
        y,
        closest,
        direction,
        boundary
    ):

        lookahead = (
            self.lookahead_forward
            if direction == 'forward'
            else self.lookahead_reverse
        )

        for i in range(
            closest,
            min(
                boundary + 1,
                len(self.path)
            )
        ):

            d = math.hypot(
                float(self.path[i]['x']) - x,
                float(self.path[i]['y']) - y
            )

            if d >= lookahead:
                return i

        return boundary

    # ======================================================
    # LCBR time
    # ======================================================

    def allowed_index(self, mission_time):

        """
        Largest dense point whose planner time is <=
        the current common LCBR mission time.
        """

        lo = 0
        hi = len(self.path) - 1

        while lo <= hi:

            mid = (lo + hi) // 2

            if (
                float(self.path[mid]['t'])
                <= mission_time + 1e-9
            ):
                lo = mid + 1
            else:
                hi = mid - 1

        return max(
            0,
            min(
                hi,
                len(self.path) - 1
            )
        )

    def wait_active(
        self,
        mission_time
    ):

        idx = self.allowed_index(
            mission_time
        )

        direction = (
            self.path[idx]['direction']
        )

        return (
            direction == 'wait'
        )

    def release_expired_wait(self, planner_time):
        """Release a dense WAIT block once its LCBR interval has expired.

        Dense WAIT primitives contain many samples at the same pose.  A
        nearest-point search can therefore remain pinned to the first WAIT
        sample forever after the temporal interval has ended.  When the
        current tracking index lies in such a block and planner_time has
        reached the first post-WAIT sample, jump the *reference index only*
        to that first post-WAIT sample.  The vehicle pose is never teleported.
        """
        idx = self.closest_index

        if idx >= len(self.path) - 1:
            return False

        if int(self.path[idx].get('action', -1)) != 6:
            return False

        # Walk to the first sample after this contiguous WAIT block.
        j = idx
        while (
            j < len(self.path) - 1
            and int(self.path[j].get('action', -1)) == 6
        ):
            j += 1

        release_time = float(self.path[j]['t'])

        if planner_time + 1e-9 < release_time:
            return False

        old_idx = self.closest_index
        self.closest_index = j
        self.waiting = False
        self.wait_until = None

        self.node.get_logger().info(
            f'[{self.agent_name}] WAIT RELEASE | '
            f't={planner_time:.2f} | idx={old_idx}->{j} | '
            f'next={self.path[j]["direction"]}'
        )

        return True


    def active_wait_interval(self, planner_time):
        # Return the contiguous LCBR WAIT interval containing
        # planner_time, or None when the agent may move.

        i = 0

        while i < len(self.path) - 1:

            if int(self.path[i].get('action', -1)) != 6:
                i += 1
                continue

            t0 = float(self.path[i]['t'])

            j = i

            while (
                j + 1 < len(self.path) - 1
                and
                int(self.path[j + 1].get('action', -1)) == 6
            ):
                j += 1

            # First sample after the contiguous WAIT block.
            t1 = float(self.path[j + 1]['t'])

            if t0 <= planner_time < t1:
                return t0, t1

            i = j + 1

        return None


    # ======================================================
    # Control iteration
    # ======================================================

    def update(
        self,
        mission_time
    ):

        if self.finished or self.failed:
            self.stop()
            return

        if self.latest_odom is None:
            self.stop()
            return

        x, y, theta = (
            self.odom_to_planner(
                self.latest_odom
            )
        )

        # Release an expired dense WAIT block before any nearest-point
        # search.  This prevents the tracker from remaining pinned to the
        # repeated WAIT pose after the LCBR interval has ended.
        self.release_expired_wait(mission_time)

        # --------------------------------------------------
        # Pose-aware temporal LCBR WAIT
        # --------------------------------------------------
        # A robot that is physically late must first reach the WAIT pose.
        # Stopping it merely because the global clock entered a WAIT window
        # destroys the planned space-time trajectory.
        wait_interval = self.active_wait_interval(mission_time)
        if wait_interval is not None:
            t0, t1 = wait_interval
            wait_start = next(
                (q for q in self.path
                 if int(q.get('action', -1)) == 6 and abs(float(q['t']) - t0) < 1e-6),
                None)
            if wait_start is not None:
                wait_ep = math.hypot(float(wait_start['x']) - x, float(wait_start['y']) - y)
                if wait_ep <= 0.18:
                    self.stop()
                    self.node.get_logger().info(
                        f'[{self.agent_name}] LCBR WAIT | t={mission_time:.2f} '
                        f'in [{t0:.0f},{t1:.0f}) | pose_error={wait_ep:.2f} m',
                        throttle_duration_sec=0.5)
                    return

        # ----------------------------------------------
        # Exact final goal
        # ----------------------------------------------

        goal = self.path[-1]

        gx = float(goal['x'])
        gy = float(goal['y'])

        gyaw = self.wrap(
            float(goal['yaw'])
        )

        final_ep = math.hypot(
            gx - x,
            gy - y
        )

        final_eyaw = abs(
            self.wrap(
                gyaw - theta
            )
        )

        # ----------------------------------------------
        # LCBR temporal gate
        # ----------------------------------------------

        max_index = self.allowed_index(
            mission_time
        )

        # Hard temporal frontier. Never drive past the pose that is
        # currently authorized by the LCBR clock. If the robot catches the
        # frontier, hold it there until simulated time advances.
        frontier = self.path[max_index]
        frontier_ep = math.hypot(float(frontier['x']) - x, float(frontier['y']) - y)
        if self.closest_index >= max_index and frontier_ep <= 0.12:
            self.stop()
            self.node.get_logger().info(
                f'[{self.agent_name}] TIME FRONTIER HOLD | t={mission_time:.2f} | '
                f'idx={max_index} | ep={frontier_ep:.2f} m',
                throttle_duration_sec=0.5)
            return

        # ----------------------------------------------
        # Final overshoot guard
        # ----------------------------------------------

        if (
            self.closest_index
            >= len(self.path) - 20
        ):

            if (
                final_ep
                < self.best_final_ep
            ):

                self.best_final_ep = (
                    final_ep
                )

                self.best_final_eyaw = (
                    final_eyaw
                )

            elif (
                self.best_final_ep <= 0.25
                and
                final_ep >
                self.best_final_ep
                + self.final_overshoot_margin
            ):

                self.stop()

                self.finished = True

                self.node.get_logger().info(
                    f'[{self.agent_name}] '
                    f'COMPLETE - OVERSHOOT | '
                    f'best_ep='
                    f'{self.best_final_ep:.3f} m | '
                    f'best_eyaw='
                    f'{math.degrees(self.best_final_eyaw):.2f} deg'
                )

                return

        # ----------------------------------------------
        # Normal goal completion
        # ----------------------------------------------

        if (
            self.closest_index
            >= len(self.path) - 3
            and
            final_ep
            <= self.goal_position_tolerance
            and
            final_eyaw
            <= self.goal_yaw_tolerance
        ):

            self.stop()

            self.finished = True

            self.node.get_logger().info(
                f'[{self.agent_name}] '
                f'COMPLETE | '
                f'ep={final_ep:.3f} m | '
                f'eyaw='
                f'{math.degrees(final_eyaw):.2f} deg'
            )

            return

        # ----------------------------------------------
        # Find closest reference point, but NEVER
        # beyond current LCBR time.
        # ----------------------------------------------

        closest, tracking_error = (
            self.find_closest_index(
                x,
                y,
                max_index=max_index
            )
        )

        if (
            tracking_error
            > self.max_tracking_error
        ):

            self.stop()

            self.failed = True

            self.node.get_logger().error(
                f'[{self.agent_name}] '
                f'TRACKING LOST | '
                f'idx={closest} | '
                f'error={tracking_error:.3f} m'
            )

            return

        direction = (
            self.path[closest]
            ['direction']
        )

        # ----------------------------------------------
        # WAIT is controlled by mission time.
        # Do NOT advance closest_index artificially.
        # ----------------------------------------------

        if direction == 'wait':

            # We can only be here while the WAIT is still temporally active.
            # If it has expired, release_expired_wait() above moves the
            # reference index to the first post-WAIT sample.
            if self.release_expired_wait(mission_time):
                closest = self.closest_index
                direction = self.path[closest]['direction']
            else:
                self.stop()
                self.node.get_logger().info(
                    f'[{self.agent_name}] WAIT | '
                    f't={mission_time:.2f} | idx={closest}',
                    throttle_duration_sec=0.5
                )
                return

        # Final sample is only a marker.
        if direction == 'goal':

            direction = (
                self.path[-2]
                ['direction']
            )

            closest = (
                len(self.path) - 2
            )

            self.closest_index = closest

        # ----------------------------------------------
        # Direction switch
        # ----------------------------------------------

        if (
            direction
            != self.current_direction
        ):

            self.stop()

            self.node.get_logger().info(
                f'[{self.agent_name}] '
                f'DIRECTION SWITCH: '
                f'{self.current_direction} '
                f'-> {direction} '
                f'at {closest}'
            )

            self.current_direction = (
                direction
            )

            return

        # ----------------------------------------------
        # Lookahead
        # ----------------------------------------------

        boundary = (
            self.find_direction_boundary(
                closest,
                max_index
            )
        )

        target_index = (
            self.find_lookahead(
                x,
                y,
                closest,
                direction,
                boundary
            )
        )

        target = self.path[
            target_index
        ]

        dx = (
            float(target['x'])
            - x
        )

        dy = (
            float(target['y'])
            - y
        )

        target_heading = math.atan2(
            dy,
            dx
        )

        # ----------------------------------------------
        # Speed
        # ----------------------------------------------

        # Time-consistent speed law. All SHA* motion primitives have the
        # same nominal arc length (~R*deltat = 2.118 m), therefore forward
        # and reverse must use the same nominal magnitude if one planner
        # step is to represent one common Ts. A small phase correction lets
        # a late robot catch up without allowing it to jump reference points.
        sign = 1.0 if direction == 'forward' else -1.0
        nominal_mag = abs(self.forward_speed)
        reference_t = float(self.path[closest]['t'])
        phase_error = mission_time - reference_t
        speed_mag = nominal_mag + 0.035 * phase_error
        speed_mag = max(0.10, min(0.32, speed_mag))

        # Tracking-error governor: nominal paths pass close to some static
        # obstacles, so large lateral error must reduce speed before contact.
        if tracking_error > 0.08:
            scale = max(0.35, 1.0 - 2.5 * (tracking_error - 0.08))
            speed_mag *= scale

        speed = sign * speed_mag

        # Final slowdown
        if (
            closest
            >= len(self.path) - 20
        ):

            sign = (
                1.0
                if speed > 0.0
                else -1.0
            )

            terminal_mag = max(
                0.035,
                min(
                    abs(speed),
                    0.45 * final_ep
                )
            )

            speed = (
                sign * terminal_mag
            )

        # ----------------------------------------------
        # Pure Pursuit
        # ----------------------------------------------

        geometric_heading = -theta

        alpha = self.wrap(
            target_heading
            - geometric_heading
        )

        ld = max(
            math.hypot(dx, dy),
            1e-3
        )

        curvature = (
            2.0
            * math.sin(alpha)
            / ld
        )

        omega = (
            speed
            * curvature
        )

        omega_max = (
            abs(speed)
            / self.minimum_turning_radius
        )

        omega = max(
            -omega_max,
            min(
                omega_max,
                omega
            )
        )

        # ----------------------------------------------
        # Slow near forward/reverse boundary
        # ----------------------------------------------

        bx = float(
            self.path[boundary]['x']
        )

        by = float(
            self.path[boundary]['y']
        )

        boundary_distance = math.hypot(
            bx - x,
            by - y
        )

        if boundary_distance < 0.70:

            sign = (
                1.0
                if speed > 0
                else -1.0
            )

            speed = (
                sign
                * min(
                    abs(speed),
                    max(
                        0.10,
                        0.55
                        * boundary_distance
                    )
                )
            )

            omega_max = (
                abs(speed)
                / self.minimum_turning_radius
            )

            omega = max(
                -omega_max,
                min(
                    omega_max,
                    speed * curvature
                )
            )

        # ----------------------------------------------
        # Command
        # ----------------------------------------------

        cmd = Twist()

        cmd.linear.x = float(
            speed
        )

        cmd.angular.z = float(
            omega
        )

        self.cmd_pub.publish(
            cmd
        )

        self.node.get_logger().info(
            f'[{self.agent_name}] '
            f't={mission_time:.2f} | '
            f'idx={closest:03d}'
            f'->{target_index:03d} '
            f'[{direction}] | '
            f'etrack={tracking_error:.2f} | '
            f'goal={final_ep:.2f} | '
            f'v={speed:+.2f}',
            throttle_duration_sec=0.5
        )


class MultiUGVTracker(Node):

    def __init__(self):

        super().__init__(
            'multi_ugv_tracker'
        )

        # --------------------------------------------------
        # Controller parameters
        # --------------------------------------------------

        self.declare_parameter(
            'forward_speed',
            0.25
        )

        self.declare_parameter(
            'reverse_speed',
            -0.25
        )

        self.declare_parameter(
            'lookahead_forward',
            0.45
        )

        self.declare_parameter(
            'lookahead_reverse',
            0.40
        )

        self.declare_parameter(
            'wheelbase',
            1.30
        )

        self.declare_parameter(
            'steering_limit',
            0.448
        )

        self.declare_parameter(
            'goal_position_tolerance',
            0.12
        )

        self.declare_parameter(
            'goal_yaw_tolerance_deg',
            5.0
        )

        self.declare_parameter(
            'max_tracking_error',
            1.50
        )

        # IMPORTANT:
        # One planner T_s step corresponds initially
        # to this many SIMULATION seconds.
        self.declare_parameter(
            'planner_step_seconds',
            8.5
        )

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        forward_speed = float(
            self.get_parameter(
                'forward_speed'
            ).value
        )

        reverse_speed = float(
            self.get_parameter(
                'reverse_speed'
            ).value
        )

        lookahead_forward = float(
            self.get_parameter(
                'lookahead_forward'
            ).value
        )

        lookahead_reverse = float(
            self.get_parameter(
                'lookahead_reverse'
            ).value
        )

        wheelbase = float(
            self.get_parameter(
                'wheelbase'
            ).value
        )

        steering_limit = float(
            self.get_parameter(
                'steering_limit'
            ).value
        )

        goal_position_tolerance = float(
            self.get_parameter(
                'goal_position_tolerance'
            ).value
        )

        goal_yaw_tolerance = math.radians(
            float(
                self.get_parameter(
                    'goal_yaw_tolerance_deg'
                ).value
            )
        )

        max_tracking_error = float(
            self.get_parameter(
                'max_tracking_error'
            ).value
        )

        self.planner_step_seconds = float(
            self.get_parameter(
                'planner_step_seconds'
            ).value
        )

        # --------------------------------------------------
        # Load six dense trajectories
        # --------------------------------------------------

        share = Path(
            get_package_share_directory(
                'ugv_gazebo'
            )
        )

        dense_dir = (
            share
            / 'trajectories'
            / 'dense'
        )

        self.agents = []

        for i in range(NUM_AGENTS):

            trajectory_file = (
                dense_dir
                / f'agent{i}_dense.yaml'
            )

            if not trajectory_file.exists():

                source_fallback = (
                    Path(__file__)
                    .resolve()
                    .parents[1]
                    / 'trajectories'
                    / 'dense'
                    / f'agent{i}_dense.yaml'
                )

                trajectory_file = (
                    source_fallback
                )

            with trajectory_file.open(
                'r'
            ) as f:

                data = yaml.safe_load(
                    f
                )

            path = data[
                'trajectory'
            ]

            tracker = AgentTracker(
                node=self,
                agent_index=i,
                path=path,
                forward_speed=forward_speed,
                reverse_speed=reverse_speed,
                lookahead_forward=lookahead_forward,
                lookahead_reverse=lookahead_reverse,
                wheelbase=wheelbase,
                steering_limit=steering_limit,
                goal_position_tolerance=goal_position_tolerance,
                goal_yaw_tolerance=goal_yaw_tolerance,
                max_tracking_error=max_tracking_error,
            )

            self.agents.append(
                tracker
            )

            self.get_logger().info(
                f'agent{i} -> ugv_{i+1}: '
                f'{len(path)} dense points'
            )

        # --------------------------------------------------
        # Common mission clock
        # --------------------------------------------------

        # Common Gazebo simulation-time origin.
        # Initialized after all six UGVs have received odometry.
        self.sim_start_time = None

        self.timer = self.create_timer(
            0.005,
            self.control_loop
        )

        self.get_logger().info(
            'MULTI-UGV TRACKER READY | '
            '6 agents | common LCBR clock'
        )

    def mission_time(self):

        # Do not start the LCBR scheduler until all six robots
        # have supplied odometry.
        if any(
            agent.latest_sim_time is None
            for agent in self.agents
        ):
            return None

        # All odometry timestamps originate from the same Gazebo
        # simulation clock. Use the oldest latest sample so the
        # scheduler cannot advance ahead of any feedback stream.
        sim_now = min(
            agent.latest_sim_time
            for agent in self.agents
        )

        if self.sim_start_time is None:

            self.sim_start_time = sim_now

            self.get_logger().info(
                f'COMMON GAZEBO CLOCK START | '
                f't_sim={sim_now:.3f} s | '
                f'Ts={self.planner_step_seconds:.3f} s'
            )

            return 0.0

        elapsed_sim = max(
            0.0,
            sim_now - self.sim_start_time
        )

        # Convert Gazebo simulated seconds to the discrete
        # LCBR time coordinate.
        return (
            elapsed_sim
            / self.planner_step_seconds
        )


    def control_loop(self):

        t = self.mission_time()

        if t is None:

            for agent in self.agents:
                agent.stop()

            return

        for agent in self.agents:
            agent.update(t)

        if all(
            a.finished or a.failed
            for a in self.agents
        ):

            for agent in self.agents:
                agent.stop()

            success = sum(
                1
                for a in self.agents
                if a.finished
            )

            failed = sum(
                1
                for a in self.agents
                if a.failed
            )

            self.get_logger().info(
                f'MULTI-UGV MISSION END | '
                f'complete={success}/6 | '
                f'failed={failed}/6'
            )

            self.timer.cancel()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = MultiUGVTracker()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        # Publish final zero commands only while the ROS context is valid.
        if rclpy.ok():
            for agent in node.agents:
                agent.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
