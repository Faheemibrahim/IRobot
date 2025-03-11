from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

# bring in other launchfiles 
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('nav'),'config')
    rviz_dir = os.path.join(get_package_share_directory('nav'),'rviz2')
    map_file = os.path.join(get_package_share_directory('nav'), 'maps', 'dubai_house_basic.yaml') # 'my_house_india.yaml' 
    params_file = os.path.join(config_dir,'wafflepi_parms.yaml')
    rviz_config= os.path.join(rviz_dir,'tb3_nav.rviz')
    return LaunchDescription([


    # Integerating Nav2 Stack
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'map':map_file,
        'params_file': params_file}.items(),

    ),
    
    #map server 
    Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{'yaml_filename': map_file}]
    ),


    # Rviz2 bringup
    Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d',rviz_config]

    ),

    ])