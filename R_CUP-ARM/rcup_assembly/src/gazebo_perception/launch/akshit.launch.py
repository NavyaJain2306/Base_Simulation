from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node


def generate_launch_description():

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
                    'queue_size': 5
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
        point_cloud_processor,
        inference_node,
        pcl_node,
    ])