import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    share = get_package_share_directory('ugv_gazebo')

    world = os.path.join(
        share,
        'worlds',
        'paper_corridors_6ugv.sdf'
    )

    models = os.path.join(
        share,
        'models'
    )

    old_resource = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    resource_path = (
        models
        if not old_resource
        else models + os.pathsep + old_resource
    )

    # --------------------------------------------------
    # Gazebo
    # --------------------------------------------------

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            world,
        ],
        output='screen'
    )

    # --------------------------------------------------
    # ROS <-> Gazebo bridge for all 6 UGVs
    # --------------------------------------------------

    bridge_arguments = []

    for i in range(1, 7):

        bridge_arguments.append(
            f'/ugv_{i}/cmd_vel'
            '@geometry_msgs/msg/Twist'
            '@gz.msgs.Twist'
        )

        bridge_arguments.append(
            f'/model/ugv_{i}/odometry'
            '@nav_msgs/msg/Odometry'
            '@gz.msgs.Odometry'
        )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='multi_ugv_bridge',
        output='screen',
        arguments=bridge_arguments,
    )

    # NOTE:
    # No tracker is started yet.
    #
    # First validate that all six robots expose independent
    # ROS command and odometry topics. The synchronized
    # multi-UGV executor will be added afterwards.

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            resource_path
        ),

        gazebo,

        TimerAction(
            period=2.0,
            actions=[bridge]
        ),
    ])
