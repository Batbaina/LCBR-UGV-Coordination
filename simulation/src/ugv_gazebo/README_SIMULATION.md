# UGV Gazebo validation package

This cleaned package executes the dense SHA*/LCBR trajectory of `agent0` with Gazebo Sim 8's `AckermannSteering` system.

## Important control fact

Gazebo Sim 8 `AckermannSteering` interprets `Twist.linear.x` as longitudinal speed and `Twist.angular.z` as desired yaw rate. Internally it computes `turningRadius = linear / angular` and clamps it to `wheel_base / sin(steering_limit)`. The tracker therefore limits yaw rate using exactly the same relation.

The planner uses the opposite yaw convention from Gazebo, so the world spawn yaw is `-planner_yaw` and odometry is transformed back before tracking.

`steer_p_gain` is set to 8.0 because the SHA*/LCBR planner assumes steering primitives can switch curvature at state boundaries; Gazebo's default steering P gain of 1.0 produced substantial steering lag at `action 1 -> action 2` transitions.

## Build

```bash
cd ~/Downloads/LABO/"ROS PROJECT"/ugv_gazebo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ugv_gazebo --symlink-install
source install/setup.bash
```

## Run (single command)

```bash
ros2 launch ugv_gazebo simulation.launch.py
```

The launch file starts Gazebo, the ROS-Gazebo bridge and the trajectory tracker. Gazebo starts unpaused (`-r`).

## Manual fallback

Terminal 1:
```bash
ros2 run ros_gz_sim gz_sim $(ros2 pkg prefix ugv_gazebo)/share/ugv_gazebo/worlds/test_world.sdf
```

Terminal 2:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /ugv_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/ugv_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry
```

Terminal 3:
```bash
ros2 run ugv_gazebo pure_pursuit_tracker --ros-args \
  --params-file $(ros2 pkg prefix ugv_gazebo)/share/ugv_gazebo/config/tracker.yaml
```

## Current scope

This version intentionally validates one UGV (`agent0`). Multi-UGV spawning, time synchronization / WAIT semantics, and collision-preservation validation should be added only after single-agent tracking is confirmed on the target Jazzy + Gazebo Sim 8.11 installation.
