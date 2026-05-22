import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Paths
    pkg_description = get_package_share_directory('archer_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Check for optional packages
    try:
        pkg_nav2 = get_package_share_directory('nav2_bringup')
        pkg_slam = get_package_share_directory('slam_toolbox')
    except:
        pkg_nav2 = None
        pkg_slam = None

    # Config files
    xacro_file = os.path.join(pkg_description, 'urdf', 'archer.urdf.xacro')
    world_file = os.path.join(pkg_description, 'worlds', 'archer_world.sdf')
    bridge_config = os.path.join(pkg_description, 'config', 'ros_gz_bridge.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'sim.rviz')

    # 2. Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_slam = LaunchConfiguration('use_slam', default='false')
    use_nav2 = LaunchConfiguration('use_nav2', default='false')

    # 3. Environment Variables (Critical for WSLg)
    set_render_engine = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE_GUI', 'ogre')
    set_gallium_driver = SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12')
    
    # Add project resources to Gazebo path
    set_gz_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', 
        pkg_description + ':' + os.path.dirname(pkg_description)
    )

    # 4. Nodes & Includes
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression(["' -r ' + '", world_file, "' if '", gui, "' == 'true' else ' -r -s ' + '", world_file, "'"])
        }.items(),
    )

    # Spawn Robot (Archer V2)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'archer_v2',
            '-z', '1.0'
        ],
        output='screen',
    )

    # ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/archer_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/archer/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/archer/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/archer/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/world/archer_world/clock', '/clock')
        ],
        output='screen'
    )

    # Joint State Broadcaster (triggered after spawn_robot exits with a 5.0s delay to guarantee manager is ready)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60", "--switch-timeout", "60"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    # Position Controller (triggered after joint_state_broadcaster exits to prevent lock contention)
    load_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["forward_position_controller", "--controller-manager-timeout", "60", "--switch-timeout", "60"],
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Archer AI Bridge & Depth Converter Node
    archer_bridge = Node(
        package='archer_bridge',
        executable='bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    depth_to_pointcloud = Node(
        package='archer_bridge',
        executable='depth_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Sensor Fusion (EKF)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_description, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # Specialized Image Bridge
    # Standard Bridge is already handling topics from the YAML

    # OctoMap Server (3D Metric Mapping)
    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05,
            'frame_id': 'map',
            'base_frame_id': 'base_link',
            'sensor_model/max_range': 5.0,
            'filter_ground': True,
            'pointcloud_min_z': 0.10,
            'pointcloud_max_z': 2.0
        }],
        remappings=[
            ('cloud_in', '/archer/camera/depth/points')
        ],
        condition=IfCondition(use_slam)
    )

    # SLAM Toolbox (Conditional with 5.0s Delay to prevent frozen clock transition aborts)
    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': os.path.join(pkg_description, 'config', 'slam.yaml')
                }.items()
            )
        ],
        condition=IfCondition(use_slam)
    ) if pkg_slam else None

    # Nav2 (Conditional with 7.0s Delay to ensure clock and SLAM mapping topics are active)
    nav2 = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': os.path.join(pkg_description, 'maps', 'archer_map.yaml'),
                    'params_file': os.path.join(pkg_description, 'config', 'nav2.yaml'),
                    'use_composition': 'False',
                    'use_bond': 'False',
                    'use_localization': PythonExpression(["'False' if '", use_slam, "' == 'true' else 'True'"])
                }.items()
            )
        ],
        condition=IfCondition(use_nav2)
    ) if pkg_nav2 else None

    # 5. Final Launch
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='false'),
        DeclareLaunchArgument('use_nav2', default_value='false'),
        
        # Env
        set_render_engine,
        set_gallium_driver,
        set_gz_path,

        # Core Simulation
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        load_joint_state_broadcaster,
        load_position_controller,
        robot_localization_node,
        
        # Tooling
        rviz,
        octomap,

        # AI Bridge & Conversion Node
        archer_bridge,
        depth_to_pointcloud,

        # Mapping & Navigation
        slam if pkg_slam else Node(package='std_msgs', executable='relay', name='slam_stub', condition=IfCondition('false')),
        nav2 if pkg_nav2 else Node(package='std_msgs', executable='relay', name='nav2_stub', condition=IfCondition('false')),
    ])
