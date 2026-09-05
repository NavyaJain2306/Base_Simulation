import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    object_name_arg = DeclareLaunchArgument(
        name="object_name",
        default_value="burger",
        description="Which object to assemble"
    )

    assembly_config_arg = DeclareLaunchArgument(
        name="assembly_config_path",
        default_value="",
        description="Optional override path to assembly_recipes.yaml."
    )

    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("piper_moveit_config"),
                "config",
                "kinematics.yaml"
            ),
            os.path.join(
                get_package_share_directory("piper_moveit_config"),
                "config",
                "ompl_planning.yaml"
            ),
            os.path.join(
                get_package_share_directory("piper_moveit_config"),
                "config",
                "joint_limits.yaml"
            ),
            {
                "use_sim_time": True,
                "object_name": LaunchConfiguration("object_name"),
                "assembly_config_path": LaunchConfiguration("assembly_config_path"),
            }
        ],
    )

    return LaunchDescription([
        object_name_arg,
        assembly_config_arg,
        mtc_node,
    ])