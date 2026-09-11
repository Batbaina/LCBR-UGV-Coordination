# Multi-UGV execution fix

## Root causes found

1. The previous scheduler only prevented looking ahead in the dense path; it did not hold a robot at the temporal frontier. A robot could therefore physically enter a future LCBR segment early.
2. WAIT was clock-triggered before checking whether a late physical robot had actually reached the WAIT pose. This can freeze an agent at the wrong spatial state.
3. The planner gives every motion primitive one time step, while the executor used different forward/reverse speed magnitudes. That makes the physical phase drift systematically whenever reverse primitives occur.
4. The nominal solution has small clearance at some obstacles. Tracking error therefore needs to be governed before it becomes large.
5. RTF=30 with a 50 Hz wall-clock controller gives too few feedback corrections per simulated second on typical hardware.

## Changes in this version

- Common nominal speed magnitude: 0.25 m/s forward and reverse.
- Initial Ts: 8.5 simulated seconds (2.118 m / 8.5 s ~= 0.249 m/s).
- Pose-aware WAIT: WAIT is enforced only after the robot has reached the corresponding WAIT pose.
- Hard temporal frontier hold: a robot at the currently authorized LCBR pose receives zero command until the common simulated clock advances.
- Phase-aware speed correction, bounded to 0.10--0.32 m/s; it changes speed but never skips reference points.
- Tracking-error speed governor above 0.08 m.
- Controller period 0.005 s (200 Hz wall-clock callback target).
- Gazebo target RTF reduced from 30 to 20 with max_step_size=0.01. This is faster than the original run while leaving more control updates per simulated second.
- Safe ROS shutdown to avoid publishing after rclpy context invalidation.

## Important scientific limitation

The Gazebo executor cannot create collision robustness that is absent from the planned path. If a nominal path passes an obstacle with only a few decimetres of clearance, a physically realistic tracking error can still cause contact. The definitive robust solution is to include an execution-error margin in the planner (inflate obstacles and inter-robot footprints by a tracking-error bound) and regenerate final_lcbr.yaml. This workspace improves execution fidelity; it does not alter the LCBR solution itself.
