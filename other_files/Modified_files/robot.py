                                                                                                  
#!/usr/bin/env python3
#
# Authors: Darby Lim (modified by Faheem for RPLiDAR C1)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        # State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(
                    get_package_share_directory('turtlebot3_bringup'),
                    'launch',
                    'turtlebot3_state_publisher.launch.py')]
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # RPLiDAR C1 Integration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(
                    get_package_share_directory('rplidar_ros'),
                    'launch',
                    'rplidar_c1_launch.py')]
            ),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'base_scan'
            }.items(),
        ),

        # TurtleBot3 Node
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),

        # Camera Publisher Node
        Node(
            package='camera_pub',
            executable='camera_pub',
            name='camera_pub',
            output='screen'
        ),
    ])








