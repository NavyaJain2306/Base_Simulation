import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import json


def generate_launch_description():

    package_name  = 'robot'
    urdf_filename = 'main_robot.xacro'
    robot_name    = 'mecanum_robot'
    world_file    = 'empty.sdf'

    pkg_path   = get_package_share_directory(package_name)
    urdf_path  = os.path.join(pkg_path, 'urdf', urdf_filename)
    world_path = os.path.join(pkg_path, 'worlds', world_file)

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

   
    robot_state_publisher = TimerAction(
    period=2.0,  # wait for gazebo + clock bridge
    actions=[
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }]
        )
    ]
)
#     slam_node = TimerAction(
#     period=23.0,
#     actions=[
#         Node(
#             package='slam_toolbox',
#             executable='async_slam_toolbox_node',
#             name='slam_toolbox',
#             output='screen',
#             parameters=[
#                 os.path.join(pkg_path, 'config', 'slam.yaml')
#             ]
#         )
#     ]
# )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=pkg_path
    )

    set_resource_path2 = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_path
    )

    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

  
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

#     gazebo = ExecuteProcess(
#     cmd=['ign', 'gazebo', '-r', world_path],
#     output='screen',
#     additional_env={
#         'IGN_FUEL_URL': '',           # disable fuel downloads
#         'OGRE_RTT_MODE': 'Copy',      # reduce GPU load
#     }
# )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                #    '/world/empty/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    # "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    # "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    # "/depth_camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                    # "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
        parameters=[{'use_sim_time': True}],
    )
    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-topic', '/robot_description',
                    '-name',  robot_name,
                    '-x', '-1',
                    '-y', '-2.75',
                    '-z', '0.11',
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', '1.5708',
                ]
            )
        ]
    )

  
    # tf_relay = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     arguments=[
    #         '/mecanum_controller/tf_odometry',
    #         '/tf'
    #     ],
    #     output='screen'
    # )



    ekf_node = TimerAction(
    period=20.0,   
    actions=[
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(pkg_path, 'config', 'ekf.yaml'),
                {'use_sim_time': True}
            ]
        )
    ]
)

   
    joint_state_broadcaster_spawner = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ]
    )

    mecanum_controller_spawner = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['mecanum_controller'],
                output='screen',
            )
        ]
    )

    # FIX: Removed /depth_camera/depth/image_raw from image_bridge.
    # That topic is raw float32 depth data — image_bridge is for compressed
    # colour transport and double-bridging the same topic causes conflicts.
    # The depth image is already fully accessible via parameter_bridge above.
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image_raw",
        ],
        parameters=[{'use_sim_time': True}],
    )

    arm_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
        )]
    )

    gripper_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller"],
        )]
    )

    # wb_top:    pose in SDF -> (-2.5,  2.25, 0.5), yaw 0.0
    wb_top_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wb_top_tf_publisher',
        output='screen',
        arguments=[
            '--x', '4.095', 
            '--y', '1.565', 
            '--z', '0.5',   # Note: Change this to 0.5 if this is one of your tables!
            '--yaw', '1.611', 
            '--pitch', '0.0', 
            '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'wb_top'
        ]
    )

    # wb_left:   pose in SDF -> (-3.75, 1.25, 0.5), yaw 1.5708
    wb_left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wb_left_tf_publisher',
        output='screen',
        arguments=[
            '--x', '3.829', 
            '--y', '1.950', 
            '--z', '0.5',   # Change to 0.5 if this is a table/counter!
            '--yaw', '-0.010', 
            '--pitch', '0.0', 
            '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'wb_left'
        ]
    )

    # wb_bottom: pose in SDF -> (-4.75, -2.5,  0.5), yaw 1.5708
    wb_bottom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wb_bottom_tf_publisher',
        output='screen',
        arguments=[
            '--x', '0.461', 
            '--y', '2.942', 
            '--z', '0.5',   # Change to 0.5 if this is a table/counter!
            '--yaw', '0.130', 
            '--pitch', '0.0', 
            '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'wb_bottom'
        ]
    )

    
    s_top_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='s_top_tf_publisher',
        output='screen',
        arguments=[
            '--x', '0.815',
            '--y', '1.564',
            '--z', '0.0',
            '--yaw', '3.099',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 's_top'
    ]
)
    
    cc_1_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cc_1_tf_publisher',
        output='screen',
        arguments=[
            '--x', '4.786',        # ← replace with tf2_echo value
            '--y', '-0.068',      # ← replace with tf2_echo value
            '--z', '0.0',
            '--yaw', '0.040',   # ← replace with tf2_echo value
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'cc_1'
        ]
)
    s_center_mid_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='s_center_mid_tf_publisher',
            output='screen',
            arguments=[
                '--x', '1.027',        # ← replace with tf2_echo value
                '--y', '1.629',      # ← replace with tf2_echo value
                '--z', '0.0',
                '--yaw', '-1.539',   # ← replace with tf2_echo value
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'map', '--child-frame-id', 's_center_mid'
            ]
    )

    line_extraction_node = TimerAction(
    period=22.0,
    actions=[
        Node(
            package='laser_line_extraction',
            executable='line_extraction_node',
            name='line_extractor',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'scan_topic': '/scan',
                'frame_id': 'lidar_link',
                'publish_markers': True,
                'frequency': 30.0,
                'bearing_std_dev': 1e-5,
                'range_std_dev': 0.012,
                'least_sq_angle_thresh': 0.0001,
                'least_sq_radius_thresh': 0.0001,
                'max_line_gap': 0.5,
                'min_line_length': 0.7,
                'max_line_length': 0.9,
                'min_range': 0.2,
                'max_range': 10.0,
                'min_split_dist': 0.04,
                'outlier_dist': 0.06,
                'min_line_points': 5,
            }]
        )
    ]
)
    

    align_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='robot',
                executable='align.py',
                name='align',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'require_workspace_match': False,   
                    'workspace_length': 0.95,
                    'distance_threshold': 1.2,
                    'angle_threshold': 30.0,
                    'workspace_length_error_threshold': 0.2,
                    'workspace_safety_distance': 0.14,
                    'num_of_msgs': 50,
                    # 'max_align_iterations': 3,
                    # 'align_settle_time': 4.0,
                    # 'converged_angle_threshold': 0.03,
                    # 'converged_lateral_threshold': 0.03,

                    'max_align_iterations': 3,
                    'nav_arrival_timeout': 20.0,
                    # 'nav_arrival_xy_tolerance': 0.1,
                    # 'nav_arrival_yaw_tolerance': 0.1,
                    'converged_angle_threshold': 0.03,
                    'converged_lateral_threshold': 0.03,
                }],
                remappings=[
                    ('line_segments', '/line_segments'),
                    ('destination_pose', '/goal_pose'),
                ]
            )
        ]
    )


    return LaunchDescription([
        set_resource_path,
        set_resource_path2,
        set_plugin_path,
        gazebo,
        rviz_node,
        robot_state_publisher,
        bridge,
        spawn_robot,
        # tf_relay,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
        # ros_gz_image_bridge,
        # arm_controller_spawner,
        # gripper_controller_spawner,
        ekf_node,
        # slam_node,
        wb_top_tf,
        wb_bottom_tf,
        wb_left_tf ,
        s_top_tf,
        cc_1_tf,
        s_center_mid_tf,
        line_extraction_node,
        align_node
    ])


