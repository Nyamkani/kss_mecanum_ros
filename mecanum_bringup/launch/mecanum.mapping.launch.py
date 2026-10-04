"""Start mapping only; mecanum.base.launch.py must already be running."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'config', 'slam_toolbox_mapping.yaml',
    )
    slam_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items(),
        ),
    ])
