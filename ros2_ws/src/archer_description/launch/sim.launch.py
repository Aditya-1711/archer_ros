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
    except Exception:
        pkg_nav2 = None
        pkg_slam = None

    # Config files
    xacro_file         = os.path.join(pkg_description, 'urdf', 'archer.urdf.xacro')
    world_file         = os.path.join(pkg_description, 'worlds', 'archer_world.sdf')
    rviz_config        = os.path.join(pkg_description, 'rviz', 'sim.rviz')
    visual_slam_config = os.path.join(pkg_description, 'config', 'visual_slam.yaml')

    # 2. Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui          = LaunchConfiguration('gui',          default='true')
    use_rviz     = LaunchConfiguration('use_rviz',     default='true')
    use_slam     = LaunchConfiguration('use_slam',     default='false')
    use_nav2     = LaunchConfiguration('use_nav2',     default='false')
    use_3d_map   = LaunchConfiguration('use_3d_map',   default='false')
    use_yolo     = LaunchConfiguration('use_yolo',     default='true')
    use_cuvslam  = LaunchConfiguration('use_cuvslam',  default='false')

    # 3. Environment Variables (Critical for WSLg)
    set_render_engine = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE_GUI', 'ogre')
    set_gallium_driver = SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12')
    set_gz_ip  = SetEnvironmentVariable('GZ_IP',  '127.0.0.1')
    set_ign_ip = SetEnvironmentVariable('IGN_IP', '127.0.0.1')
    set_gz_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        pkg_description + ':' + os.path.dirname(pkg_description)
    )

    # ─────────────────────────────────────────────────────────────────
    # 4. Core Simulation Nodes
    # ─────────────────────────────────────────────────────────────────

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' use_cuvslam:=', use_cuvslam]), value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression([
                "' -r ' + '", world_file, "' if '", gui, "' == 'true' else ' -r -s ' + '", world_file, "'"
            ])
        }.items(),
    )

    # Spawn Robot (Archer V2) with delay to allow Gazebo server to start
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'archer_v2', '-z', '0.01'],
            output='screen',
        )]
    )

    # ─────────────────────────────────────────────────────────────────
    # 5. ROS-GZ Bridge Nodes
    # ─────────────────────────────────────────────────────────────────

    # Main topic bridge (use_sim_time=false to avoid deadlock — it is NOT the clock publisher)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # cmd_vel: ROS → GZ
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Clock: GZ → ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # odom, scan, tf, imu, joint_states: GZ → ROS
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan_torso@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/archer/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Monocular camera (YOLO / OctoMap): GZ → ROS
            '/archer/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Stereo cameras (cuVSLAM): GZ → ROS
            '/archer/camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/archer/camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/model/archer/tf', '/tf'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='ros_gz_image_bridge',
        arguments=[
            '/archer/camera/image_raw',
            '/archer/camera/depth',
            '/archer/camera/left/image_raw',
            '/archer/camera/right/image_raw'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )


    # ─────────────────────────────────────────────────────────────────
    # 7. Sensor Fusion
    # ─────────────────────────────────────────────────────────────────
    
    # ─────────────────────────────────────────────────────────────────
    # 8. Visualisation
    # ─────────────────────────────────────────────────────────────────

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ─────────────────────────────────────────────────────────────────
    # 9. AI Bridge & Depth Converter
    # ─────────────────────────────────────────────────────────────────

    archer_bridge = Node(
        package='archer_bridge',
        executable='bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    depth_to_pointcloud = Node(
        package='archer_bridge',
        executable='depth_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_3d_map),
    )


    # ─────────────────────────────────────────────────────────────────
    # 11. OctoMap Server (3D Semantic/Metric Mapping)
    # ─────────────────────────────────────────────────────────────────

    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            'resolution':                0.05,
            'frame_id':                  'map',
            'base_frame_id':             'base_link',
            'sensor_model/max_range':    5.0,
            'filter_ground':             True,
            'pointcloud_min_z':          0.10,
            'pointcloud_max_z':          2.0,
        }],
        remappings=[('cloud_in', '/archer/camera/depth/points')],
        condition=IfCondition(use_3d_map),
    )

    # ─────────────────────────────────────────────────────────────────
    # 12. YOLO Vision Node (YOLOv8n — ultralytics)
    # ─────────────────────────────────────────────────────────────────

    watchdog_node = Node(
        package='archer_bridge',
        executable='watchdog_node',
        name='watchdog_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    demonstration_node = Node(
        package='archer_bridge',
        executable='demonstration_node',
        name='demonstration_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    semantic_mapper_node = Node(
        package='archer_bridge',
        executable='semantic_mapper_node',
        name='semantic_mapper_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    yolo_node = Node(
        package='archer_yolo',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path':   os.path.expanduser('~/archer_ros/models/yolov8n.pt'),
            'conf_thresh':  0.4,
            'device':       'cpu',
        }],
        remappings=[
            ('/image_raw',   '/archer/camera/image_raw'),
            ('/detections',  '/yolo/detections'),
        ],
        condition=IfCondition(use_yolo),
    )

    # ─────────────────────────────────────────────────────────────────
    # 13. AI Explorer (Random Nav2 Goals)
    # ─────────────────────────────────────────────────────────────────
    
    random_explorer = Node(
        package='archer_bridge',
        executable='random_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_explorer')),
    )

    # ─────────────────────────────────────────────────────────────────
    # 14. SLAM Toolbox (5 s delay → clock must be up)
    # ─────────────────────────────────────────────────────────────────

    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time':     use_sim_time,
                    'slam_params_file': os.path.join(pkg_description, 'config', 'slam.yaml'),
                }.items(),
            )
        ],
        condition=IfCondition(use_slam),
    ) if pkg_slam else None

    # ─────────────────────────────────────────────────────────────────
    # 14. Nav2 (7 s delay → SLAM map must be initialised first)
    # ─────────────────────────────────────────────────────────────────

    nav2 = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time':     use_sim_time,
                    'map':              os.path.join(pkg_description, 'maps', 'archer_map.yaml'),
                    'params_file':      os.path.join(pkg_description, 'config', 'nav2.yaml'),
                    'use_composition':  'False',
                    'use_bond':         'False',
                    'use_localization': PythonExpression(["'False' if '", use_slam, "' == 'true' else 'True'"]),
                }.items(),
            )
        ],
        condition=IfCondition(use_nav2),
    ) if pkg_nav2 else None

    # ─────────────────────────────────────────────────────────────────
    # cuVSLAM — PyCuVSLAM-based stereo visual odometry
    # Uses our custom archer_bridge/cuvslam_node (wraps the PyCuVSLAM pip wheel).
    # No Isaac ROS apt package needed — install the wheel from:
    #   https://github.com/nvidia-isaac/cuVSLAM/releases
    #   pip install cuvslam-*-cp312-*-linux_x86_64.whl
    # Enabled via: use_cuvslam:=true
    # Requires: NVIDIA GPU + CUDA 12 accessible in WSL2
    # ─────────────────────────────────────────────────────────────────

    visual_slam = TimerAction(
        period=7.0,
        actions=[Node(
            package='archer_bridge',
            executable='cuvslam_node',
            name='cuvslam_node',
            output='screen',
            parameters=[{
                'use_sim_time':   use_sim_time,
                'odom_frame':     'odom',
                'base_frame':     'base_link',
                'map_frame':      'map',
                'warmup_frames':  30,
                'jitter_ms':      34.0,
            }],
            condition=IfCondition(use_cuvslam),
        )],
        condition=IfCondition(use_cuvslam),
    )

    # ─────────────────────────────────────────────────────────────────
    # 15. Assemble LaunchDescription
    # ─────────────────────────────────────────────────────────────────

    ld = LaunchDescription([
        # ── Launch Arguments ──────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui',          default_value='true'),
        DeclareLaunchArgument('use_rviz',     default_value='true'),
        DeclareLaunchArgument('use_slam',     default_value='false'),
        DeclareLaunchArgument('use_nav2',     default_value='false'),
        DeclareLaunchArgument('use_3d_map',   default_value='false'),
        DeclareLaunchArgument('use_yolo',     default_value='true'),
        DeclareLaunchArgument('use_explorer', default_value='true'),
        DeclareLaunchArgument('use_cuvslam',  default_value='false',
                              description='Launch cuVSLAM stereo visual odometry node (archer_bridge/cuvslam_node). '
                                          'Requires: PyCuVSLAM pip wheel + NVIDIA GPU in WSL2.'),

        # ── Environment ───────────────────────────────────────────────
        set_render_engine,
        set_gallium_driver,
        set_gz_path,
        set_gz_ip,
        set_ign_ip,

        # ── Core Simulation ───────────────────────────────────────────
        gz_sim,
        spawn_robot,
        robot_state_publisher,

        # ── Bridges ───────────────────────────────────────────────────
        bridge,
        image_bridge,

        # ── Controllers ───────────────────────────────────────────────

        # ── Perception & Conversion ───────────────────────────────────
        depth_to_pointcloud,
        yolo_node,
        octomap,

        # ── Visualisation ─────────────────────────────────────────────
        rviz,

        # ── AI Bridge ────────────────────────────────────────────────
        archer_bridge,

        # ── Mapping & Navigation ──────────────────────────────────────
        slam if pkg_slam else Node(package='std_msgs', executable='relay', name='slam_stub', condition=IfCondition('false')),
        nav2  if pkg_nav2  else Node(package='std_msgs', executable='relay', name='nav2_stub',  condition=IfCondition('false')),

        # ── cuVSLAM Visual Odometry (optional, use_cuvslam:=true) ─────
        visual_slam,

        # ── Explorer ──────────────────────────────────────────────────
        random_explorer,
        
        # ── Supervisors & Learning ────────────────────────────────────
        watchdog_node,
        demonstration_node,
        semantic_mapper_node,
    ])

    return ld
