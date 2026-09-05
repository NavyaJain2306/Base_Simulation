import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localisation.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items()
    )

    wait_for_localisation = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_share, 'lib', 'robot', 'wait_for_active.py'),
            'amcl',
        ],
        output='screen',
        name='wait_for_localisation_active'
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items()
    )

    wait_for_nav = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_share, 'lib', 'robot', 'wait_for_active.py'),
            'velocity_smoother',
        ],
        output='screen',
        name='wait_for_nav_active'
    )

    destination_node = Node(
        package='robot',
        executable='destination.py',
        name='go_to_shelf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    start_nav_after_localisation = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_localisation,
            on_exit=[nav_launch, wait_for_nav],
        )
    )

    start_destination_after_nav = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav,
            on_exit=[destination_node],
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(start_nav_after_localisation)
    ld.add_action(start_destination_after_nav)

    ld.add_action(localisation_launch)
    ld.add_action(wait_for_localisation)

    return ld
