import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, \
    SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration, \
    PathJoinSubstitution, PythonExpression, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    gpu_offload = SetEnvironmentVariable(name="__NV_PRIME_RENDER_OFFLOAD", value="1")
    glx_vendor = SetEnvironmentVariable(name="__GLX_VENDOR_LIBRARY_NAME", value="nvidia")

    robo_description = get_package_share_directory("piper_description")
    piper_world = get_package_share_directory("piper_bringup")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                robo_description, "urdf", "piper.urdf.xacro"
            ),
        description="Path to robot urdf file"
    )

    # Pass the same value to both launch files, e.g.:
    #   ros2 launch piper_bringup gazebo_launch.py object_name:=ice_cream
    #   ros2 launch mtc_tutorial pickplace_launch.py object_name:=ice_cream
    object_name_arg = DeclareLaunchArgument(
        name="object_name",
        default_value="burger",
        description="Which object's world to load (worlds/<object_name>.sdf must exist in "
                     "piper_bringup), e.g. burger, ice_cream, battery, magnet, carrot, "
                     "traffic_light, big_carrot, e_stop, small_tree, hammer, big_tree."
    )

    world_path = PathJoinSubstitution([
            piper_world,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("object_name"), "'", " + '.sdf'"])
        ]
    )

    model_path = str(Path(piper_world).parent.resolve())
    model_path += pathsep + "/home/sathwik/R_CUP-ARM/rcup_assembly/src/piper_bringup/models"

    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("piper_description"), "urdf", "piper.urdf.xacro"]
            ),
            " ",
            "sim_gazebo:=true",
            " ",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "piper",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        )]
    )

    arm_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller"],
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

    piper_moveit_config_dir = get_package_share_directory('piper_moveit_config')

    robot_description_semantic = ParameterValue(
        Command([
            'xacro ', 
            os.path.join(piper_moveit_config_dir, 'config', 'piper.srdf.xacro')
        ]),
        value_type=str
    )

    # 4. YAML Parameter Files
    ompl_planning_yaml = os.path.join(piper_moveit_config_dir, 'config', 'ompl_planning.yaml')
    kinematics_yaml = os.path.join(piper_moveit_config_dir, 'config', 'kinematics.yaml')
    joint_limits_yaml = os.path.join(piper_moveit_config_dir, 'config', 'joint_limits.yaml')
    moveit_controllers_yaml = os.path.join(piper_moveit_config_dir, 'config', 'moveit_controllers.yaml')
    rviz_config_file = os.path.join(piper_moveit_config_dir, 'config', 'moveit.rviz')

    # 5. Nodes
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            ompl_planning_yaml,
            kinematics_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
            {
                'robot_description_semantic': robot_description_semantic,
                'publish_robot_description_semantic': True,
                'use_sim_time': True,
                'capabilities': 'move_group/ExecuteTaskSolutionCapability',
                'disable_capabilities': ''
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            ompl_planning_yaml,
            kinematics_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
            {
                'use_sim_time': True
            }
        ]
    )

    point_cloud_processor = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/image_rect_color', '/rgbd_camera/image'),
                    ('depth_registered/image_rect', '/rgbd_camera/depth_image'),
                    ('rgb/camera_info', '/rgbd_camera/camera_info'),
                    ('points', '/rgbd_camera/points')
                ],
                # CRITICAL: You must include approximate_sync and queue_size here
                parameters=[{
                    'use_sim_time': True, 
                    'approximate_sync': True,
                    'queue_size': 20
                }]
            ),
        ],
        output='screen',
    )
    
    inference_node = Node(
        package="gazebo_perception",
        executable="inference_node",
        name="inference_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    pcl_node = Node(
        package="pcl_geometry",
        executable="pcl_geometry_node",
        name="pcl_geometry_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        gpu_offload,
        glx_vendor,
        model_arg,
        object_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        point_cloud_processor,
        inference_node,
        pcl_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
    ])
