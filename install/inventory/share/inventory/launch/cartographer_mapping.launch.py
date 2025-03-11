from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
def generate_launch_description():
    config_dir= os.path.join(get_package_share_directory('nav'),'config')
    rviz_mapping_config = os.path.join(get_package_share_directory('nav'),'rviz2','view_mapping.rviz')
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            output='screen',
            executable='cartographer_node',
            name='cartographer_node',
            arguments=['-configuration_directory', config_dir ,'-configuration_basename', 'cartographer.lua'] 
            #(using the main arguments from cartographer_ros/node_main.cc )
            #cartographer lua contains
            #config_dir contains 
        ),
        Node(
            package='cartographer_ros',
            output='screen',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_og_node'
            #arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d',rviz_mapping_config]
        )
    ])