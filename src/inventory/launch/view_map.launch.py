import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    map_file = os.path.join(get_package_share_directory('nav'), 'maps', 'my_house_india.yaml') # change name of map to view diffrent map 
    rviz = os.path.join(get_package_share_directory('nav'), 'rviz2', 'view_map.rviz')
    params_file = os.path.join(get_package_share_directory('nav'), 'config', 'wafflepi_parms.yaml')  # Define the path for the params file

    #launch map server 
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': map_file}]
    )
    
    #launch nav stack  
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'map': map_file,
            'params_file': params_file
        }.items()
    )
        
    # Start RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz]  # Adjust path to your RViz config
    )
    
    # Create and return the LaunchDescription
    ld = LaunchDescription()

    # Add map_server, nav, and rviz_node to the launch description
    ld.add_action(map_server)
    ld.add_action(nav)
    ld.add_action(rviz_node)

    return ld
