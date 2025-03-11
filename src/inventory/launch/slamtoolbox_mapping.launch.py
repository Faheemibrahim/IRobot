import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz = os.path.join(get_package_share_directory('nav'), 'rviz2', 'view_map.rviz')
    
    # Start SLAM Toolbox
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        )
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

    # Add both SLAM Toolbox and RViz to the launch description
    ld.add_action(slam_mapping)
    ld.add_action(rviz_node)

    return ld
