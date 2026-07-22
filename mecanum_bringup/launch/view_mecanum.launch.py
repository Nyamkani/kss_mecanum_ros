# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('mecanum_bringup')
    pkg_project_gazebo = get_package_share_directory('mecanum_gazebo')
    pkg_project_description = get_package_share_directory('mecanum_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    # Get the rviz config file path
    rviz_config_file = PathJoinSubstitution([os.path.join(pkg_project_description), "config", "rviz.rviz"] )
    # Get the urdf/xacro file path
    # robot_xacro_file = PathJoinSubstitution([os.path.join(pkg_project_description), "urdf", "mecanum.urdf.xacro"] )
    robot_urdf_file = PathJoinSubstitution([os.path.join(pkg_project_description), "urdf", "mecanum.urdf"] )
    gazebo_world_file = PathJoinSubstitution([os.path.join(pkg_project_gazebo), "worlds", "home.sdf"] )


    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    # )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[  {'use_sim_time': use_sim_time},],
    )

    #Showing robot data
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ',' ', robot_urdf_file]), value_type=str)}, {'use_sim_time': use_sim_time},],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    #rviz with config files
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time},],
    )

    #gazebo sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_project_gazebo,(gazebo_world_file)
        ]),
        # TextSubstitution(text=' -r -v -v1')],
        TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        # TextSubstitution(text=' -r -v -v1 --render-engine ogre')],
        'on_exit_shutdown': 'true'}.items()
    )


    # Bridge ROS topics and Gazebo messages for establishing communication
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_gazebo, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        },
         {'use_sim_time': use_sim_time},],
        output='screen'
    )
    # Spawn the robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mecanum",  #name
            "-topic",  "/robot_description",
            "-x", "0", 
            "-y", "0",
            "-z", "0",
        ],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time},],
    )

    rplidar_static_tarnsform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="both",
        arguments=["0","0","0","0","0","0", "map", "odom"],
    )

    nodes = [
        DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use sim time if true'),

        # state_publisher,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gz_sim,
        gz_ros_bridge,
        gz_spawn_entity,
        rviz_node,
        # rplidar_static_tarnsform_publisher_node,
    ]

    return LaunchDescription(nodes)