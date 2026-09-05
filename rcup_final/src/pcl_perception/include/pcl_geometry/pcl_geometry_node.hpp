// Copyright 2026 Akshit Bhaskara

#ifndef PCL_GEOMETRY__PCL_GEOMETRY_NODE_HPP_
#define PCL_GEOMETRY__PCL_GEOMETRY_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <atomic> 

#include <geometry_msgs/msg/pose_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <perception_msgs/msg/perception_msg.hpp> 
class PclGeometryNode : public rclcpp::Node
{
public:
  PclGeometryNode();

private:
  using PointT = pcl::PointXYZRGB;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, vision_msgs::msg::Detection2DArray>;

  void syncedCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg);

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  geometry_msgs::msg::Pose estimateHybridOrientation(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & brick_cloud, float theta,
    Eigen::Vector3f surface_normal);
  void handleToggle(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response
  );

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ransac_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<perception_msgs::msg::PerceptionMsg>::SharedPtr block_data_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_srv_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double ransac_distance_threshold_;
  double cluster_tolerance_;
  bool active_; 
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif  // PCL_GEOMETRY__PCL_GEOMETRY_NODE_HPP_
