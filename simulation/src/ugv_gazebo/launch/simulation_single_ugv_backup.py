import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('ugv_gazebo')
    world = os.path.join(share, 'worlds', 'paper_corridors_6ugv.sdf')
    models = os.path.join(share, 'models')
    params = os.path.join(share, 'config', 'tracker.yaml')
    old_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = models if not old_resource else models + os.pathsep + old_resource

    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/ugv_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/ugv_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ])
    tracker = Node(
        package='ugv_gazebo', executable='pure_pursuit_tracker',
        name='pure_pursuit_tracker', output='screen', parameters=[params])

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        TimerAction(period=2.0, actions=[bridge]),
        TimerAction(period=3.0, actions=[tracker]),
    ])
