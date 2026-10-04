"""Start the persistent hardware, state estimation, and gateway for the real robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_urdf_file = os.path.join(
        get_package_share_directory('mecanum_description'), 'urdf', 'mecanum.urdf'
    )
    ekf_config_file = os.path.join(
        get_package_share_directory('ekf_config'), 'config', 'ekf.yaml'
    )
    scan_filter_file = os.path.join(
        get_package_share_directory('scan_filter'), 'config', 'scan_filter.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', robot_urdf_file]), value_type=str
                ),
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='arduino_motor_driver_bridge',
            executable='arduino_motor_driver_bridge',
            output='both',
            parameters=[{
                'serial_port': '/dev/ttymotoruno',
                'serial_baudrate': 115200,
            }],
        ),
        Node(
            package='wt901c_imu',
            executable='wt901c_imu',
            output='both',
            parameters=[{
                'serial_port': '/dev/ttyimu',
                'serial_baudrate': 115200,
                'x_axis_inverted': True,
                'y_axis_inverted': True,
                'z_axis_inverted': False,
            }],
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttylidar',
                'serial_baudrate': 460800,
                'inverted': False,
                'frame_id': 'scan_link',
                'channel_type': 'serial',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_node',
            output='screen',
            parameters=[scan_filter_file],
            remappings=[('scan', '/scan'), ('scan_filtered', '/scan/filtered')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='mecanum_gateway',
            executable='gateway_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
