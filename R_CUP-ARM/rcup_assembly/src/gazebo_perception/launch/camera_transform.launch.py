from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    gpu_offload = SetEnvironmentVariable(name="__NV_PRIME_RENDER_OFFLOAD", value="1")
    glx_vendor = SetEnvironmentVariable(name="__GLX_VENDOR_LIBRARY_NAME", value="nvidia")

    inference_node = Node(
        package="mujoco_perception",
        executable="inference_node",
        name="inference_node",
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
            "0.0",
            "--z",
            "0.57",
            "--yaw",
            "0.0",
            "--pitch",
            "1.01229",  # 32 degrees in radians
            "--roll",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "zed_camera_link",
        ],
    )

    return LaunchDescription(
        [
            gpu_offload,
            glx_vendor,
            # perception_node,
            inference_node,
            # pcl_node,
            rviz_node,
            tf_node,
        ]
    )
