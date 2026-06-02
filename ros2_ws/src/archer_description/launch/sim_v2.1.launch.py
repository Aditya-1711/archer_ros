"""
ros2_ws/src/archer_description/launch/sim_v2.1.launch.py
======================================================
Unified launch file for ARCHER v2.1.
Wraps the core v2.0 simulation launch and layers safety, vision, and power manager nodes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('archer_description')
    
    # 1. Include base sim.launch.py (v2.0)
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time', default='true'),
            'gui': LaunchConfiguration('gui', default='true'),
            'use_rviz': LaunchConfiguration('use_rviz', default='true'),
            'use_slam': LaunchConfiguration('use_slam', default='false'),
            'use_nav2': LaunchConfiguration('use_nav2', default='false'),
            'use_3d_map': LaunchConfiguration('use_3d_map', default='false'),
            'use_yolo': LaunchConfiguration('use_yolo', default='true'),
        }.items()
    )
    
    # 2. Safety Supervisor Node
    safety_node = Node(
        package='archer_bridge',
        executable='safety_supervisor_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}]
    )
    
    # 3. Vision AI Node
    vision_node = Node(
        package='archer_bridge',
        executable='vision_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}]
    )
    
    # 4. Power Manager Node
    power_node = Node(
        package='archer_bridge',
        executable='power_manager_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}]
    )
    
    # 5. System Watchdog Node
    watchdog_node = Node(
        package='archer_bridge',
        executable='watchdog_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='false'),
        DeclareLaunchArgument('use_nav2', default_value='false'),
        DeclareLaunchArgument('use_3d_map', default_value='false'),
        DeclareLaunchArgument('use_yolo', default_value='true'),
        
        base_sim,
        safety_node,
        vision_node,
        power_node,
        watchdog_node,
    ])
