import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_pkg_share = get_package_share_directory('robot')
    mtc_pkg_share = get_package_share_directory('mtc_tutorial')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg_share, 'launch', 'gazebo.launch.py'))
    )

    pick_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mtc_pkg_share, 'launch', 'pick_place_demo.launch.py'))
    )

    delayed_pick_place = TimerAction(
        period=35.0,
        actions=[pick_place_launch]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(delayed_pick_place)

    return ld
