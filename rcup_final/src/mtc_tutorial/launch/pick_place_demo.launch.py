from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mecanum_arm_robot",
                 package_name="robot_moveit_config").to_dict()
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_demo])
