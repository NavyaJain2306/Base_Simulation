from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rgbd_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock', 
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
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
                parameters=[{'use_sim_time': True}]
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
        # bridge_node,
        point_cloud_processor,
        inference_node,
        pcl_node
    ])
