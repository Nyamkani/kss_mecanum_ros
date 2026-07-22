import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    # astra_parameters=[{
    #       'frame_id':'camera_link',
    #       'subscribe_depth':True,
    #       'subscribe_odom_info':True,
    #       'approx_sync':False}]

    rtabmap_parameters=[{
        #   'frame_id':'camera_link',
        #   'subscribe_depth':True,
        #   'subscribe_odom_info':True,
        #   'approx_sync':True,
        #   'qos':2,
        #   'topic_queue_size':30,
        #   'sync_queue_size':30,
        #   'use_sim_time':False,

        #   'rgb_image_transport':'compressed',
        #   'depth_image_transport':'compressedDepth',
        #   'approx_sync_max_interval': 0.05,

          'frame_id':'camera_link',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'subscribe_odom':True,
          'approx_sync':True,
          'sync_queue_size': 10,
          # RTAB-Map's internal parameters should be strings
          'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
          'RGBD/ProximityBySpace':     'true',    # Local loop closure detection (using estimated position) with locations in WM
          'RGBD/ProximityByTime':      'false',   # Local loop closure detection with locations in STM
          'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
          'Reg/Strategy':              '1',       # 0=Visual, 1=ICP, 2=Visual+ICP
          'Vis/MinInliers':            '12',      # 3D visual words minimum inliers to accept loop closure
          'RGBD/OptimizeFromGraphEnd': 'false',   # Optimize graph from initial node so /map -> /odom transform will be generated
          'RGBD/OptimizeMaxError':     '4',       # Reject any loop closure causing large errors (>3x link's covariance) in the map
          'Reg/Force3DoF':             'true',    # 2D SLAM
          'Grid/FromDepth':            'false',   # Create 2D occupancy grid from laser scan
          'Mem/STMSize':               '30',      # increased to 30 to avoid adding too many loop closures on just seen locations
          'RGBD/LocalRadius':          '5',       # limit length of proximity detections
          'Icp/CorrespondenceRatio':   '0.2',     # minimum scan overlap to accept loop closure
          'Icp/PM':                    'false',
          'Icp/PointToPlane':          'false',
          'Icp/MaxCorrespondenceDistance': '0.15',
          'Icp/VoxelSize':             '0.05',
          'use_sim_time':False,
          'wait_for_transform': 1.0,
          }]



    rtabmap_remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]
    # rtabmap_remappings=[
    #      ('rgb/image',       '/data_throttled_image'),
    #      ('depth/image',     '/data_throttled_image_depth'),
    #      ('rgb/camera_info', '/data_throttled_camera_info')
    #      ]

    laser_scan_remappings=[
         ('scan',            '/jn0/base_scan')]

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    #rplidar_serial_port =  {"serial_port": "/dev/ttyUSB0"}
    rplidar_serial_port =  {"serial_port": "/dev/ttylidar"}
    rplidar_serial_baudrate =  {"serial_baudrate": 115200}
    rplidar_data_inverted = {"inverted": False}
    rplidar_frame_id = {"frame_id": "camera_link"}
    rplidar_topic_name = {"topic_name": "scan"}

    return LaunchDescription([

        # Make sure IR emitter is enabled
        # SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=False),

        # Launch camera driver
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('astra_camera'),
                        'launch/astra_pro.launch.xml'
                    )
                ),
                launch_arguments=list({
                    'depth_registration': 'true',
                    'color_depth_synchronization': 'true',
                    'use_uvc_camera': 'true',
                    'color_width': '640',
                    'color_height': '480',
                    'color_fps': '15',
                    'tf_publish_rate': '60.0'
                }.items())
            ),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{**rtabmap_parameters[0], 'approx_sync_max_interval': 0.1}],
            #   {'rgb_image_transport':'compressed',
            #    'depth_image_transport':'compressedDepth',
            #    {'approx_sync_max_interval': 0.02}],
            remappings=rtabmap_remappings),
        
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings),

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters[0]],
            remappings=rtabmap_remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters[0],
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=rtabmap_remappings),

        # Visualization:
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings),
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),


        # Node(
        #     package='rplidar_ros', executable='rplidar_composition', name="rplidar", output='both',
        #             parameters=[rplidar_serial_port, 
        #             rplidar_serial_baudrate,
        #             rplidar_data_inverted,
        #             rplidar_frame_id,
        #             rplidar_topic_name]),


        #Nodes to launch
        # Node(
        #     package='rtabmap_sync', executable='rgbd_sync', output='screen',
        #     parameters=rtabmap_parameters,
        #     remappings=rtabmap_remappings),

        # Node(
        #     package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #     parameters=rtabmap_parameters,
        #     remappings=rtabmap_remappings),

        # Node(
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=rtabmap_parameters,
        #     remappings=rtabmap_remappings,
        #     arguments=['-d']),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=rtabmap_parameters,
        #     remappings=rtabmap_remappings),
    ])
