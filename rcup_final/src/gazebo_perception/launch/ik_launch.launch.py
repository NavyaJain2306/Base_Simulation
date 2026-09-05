from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Start the simulator viewer (using your perception node)
    perception_node = Node(
        package="mujoco_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
    )

    # 2. Start the isolated IK math engine
    ik_node = Node(
        package="pick_and_place", executable="piper_ik_node", name="piper_ik_node", output="screen"
    )

    return LaunchDescription([perception_node, ik_node])
