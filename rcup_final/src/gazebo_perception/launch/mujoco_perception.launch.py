from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    gpu_offload = SetEnvironmentVariable(name="__NV_PRIME_RENDER_OFFLOAD", value="1")
    glx_vendor = SetEnvironmentVariable(name="__GLX_VENDOR_LIBRARY_NAME", value="nvidia")

    perception_node = Node(
        package="mujoco_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
    )

    inference_node = Node(
        package="mujoco_perception",
        executable="inference_node",
        name="inference_node",
        output="screen",
    )

    pcl_node = Node(
        package="pcl_geometry",
        executable="pcl_geometry_node",
        name="pcl_geometry_node",
        output="screen",
    )

    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz", output="log")

    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--x",
            "0.0",
            "--y",
            "-0.9505",
            "--z",
            "0.9505",
            "--yaw",
            "0.0",
            "--pitch",
            "0.0",
            "--roll",
            "-2.35619",
            "--frame-id",
            "map",
            "--child-frame-id",
            "camera_optical_frame",
        ],
    )

    return LaunchDescription(
        [
            gpu_offload,
            glx_vendor,
            perception_node,
            inference_node,
            # pcl_node,
            rviz_node,
            tf_node,
        ]
    )
