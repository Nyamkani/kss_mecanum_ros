"""Start saved-map localization and Nav2; the base must already be running."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    nav_config_share = get_package_share_directory('bme_ros2_navigation')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_config_share, 'maps', 'my_map.yaml'),
            description='Full path to the saved map YAML file',
        ),
        # Both Nav2 launch files use params_file; keep their configurations scoped.
        GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, 'localization_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_file,
                    'params_file': os.path.join(
                        nav_config_share, 'config', 'amcl_localization.yaml'
                    ),
                }.items(),
            ),
        ]),
        GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': os.path.join(
                        nav_config_share, 'config', 'navigation.yaml'
                    ),
                }.items(),
            ),
        ]),
    ])
