from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():

    task_planner_node = Node(
        package='robocup_planner',
        executable='task_planner',
        name='task_planner_node',
        output='screen',
    )

    temp_warehouse_node = Node(
        package='robocup_planner',
        executable='temp_warehouse',
        name='temp_warehouse_node',
        output='screen',
    )

    start_warehouse_after_planner = RegisterEventHandler(
        OnProcessStart(
            target_action=task_planner_node,
            on_start=[temp_warehouse_node],
        )
    )

    ld = LaunchDescription()

    ld.add_action(start_warehouse_after_planner)
    ld.add_action(task_planner_node)

    return ld
