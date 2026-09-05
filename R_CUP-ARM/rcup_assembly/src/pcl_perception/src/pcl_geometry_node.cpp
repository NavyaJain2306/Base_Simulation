// Copyright 2026 Akshit Bhaskara

#include "pcl_geometry/pcl_geometry_node.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <limits>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <perception_msgs/msg/perception_msg.hpp> 

PclGeometryNode::PclGeometryNode() : Node("pcl_geometry_node")
{
  rclcpp::QoS pointcloud_qos(10);
  pointcloud_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); 
  pointcloud_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  
  pc_sub_.subscribe(this, "/rgbd_camera/points", pointcloud_qos.get_rmw_qos_profile());
  det_sub_.subscribe(this, "/vision/detected_objects");

  sync_ =
    std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), pc_sub_, det_sub_);
  sync_->registerCallback(
    std::bind(
      &PclGeometryNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/perception/brick_poses", 10);
  block_data_pub_ = this->create_publisher<perception_msgs::msg::PerceptionMsg>("/perception/brick_data", 10);
  debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/points", 10);
  ransac_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/ransac", 10);
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/obb_boxes", 10);
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "PCL Geometry Node Initialized. Waiting for data...");

  ransac_distance_threshold_ = this->declare_parameter("ransac_distance_threshold", 0.005);
  cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.01);

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PclGeometryNode::parametersCallback, this, std::placeholders::_1));

  toggle_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "/perception/set_active",
    std::bind(&PclGeometryNode::handleToggle,this,std::placeholders::_1,std::placeholders::_2));
    active_ = true; 

    RCLCPP_INFO(this->get_logger(), "GAZEBO_WS VERSION");
}
rcl_interfaces::msg::SetParametersResult PclGeometryNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters)
  {
    if (param.get_name() == "ransac_distance_threshold")
    {
      ransac_distance_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated RANSAC Threshold: %f", ransac_distance_threshold_);
    }
    else if (param.get_name() == "cluster_tolerance")
    {
      cluster_tolerance_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated Cluster Tolerance: %f", cluster_tolerance_);
    }
  }
  return result;
}

geometry_msgs::msg::Pose PclGeometryNode::estimateHybridOrientation(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & brick_cloud, float theta,
  Eigen::Vector3f surface_normal)
{
  geometry_msgs::msg::Pose pose;

  // Calculate centroid of brick_cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*brick_cloud, centroid);
  Eigen::Vector3f V_z = surface_normal;

  Eigen::Vector3f V_x_rough(std::cos(theta), std::sin(theta), 0);
  V_z.normalize();

  Eigen::Vector3f V_x = V_x_rough - (V_x_rough.dot(V_z)) * V_z;
  if (V_x.norm() < 1e-4)
  {
    V_x = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  }

  V_x.normalize();

  if (V_x.x() < 0.0)
  {
    V_x = -V_x;
  }

  Eigen::Vector3f V_y = V_z.cross(V_x);
  V_y.normalize();

  // Convert rotation matrix to quaternion and change into pose

  Eigen::Matrix3f R;
  R.col(0) = V_x;
  R.col(1) = V_y;
  R.col(2) = V_z;

  Eigen::Quaternionf q(R);

  pose.position.x = centroid(0);
  pose.position.y = centroid(1);
  pose.position.z = centroid(2);

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}

void PclGeometryNode::syncedCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg)
{ if(!active_) return; 
  RCLCPP_INFO(this->get_logger(), "Callback fired");

  if (det_msg->detections.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Empty detections");
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr optical_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc_msg, *optical_cloud);

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "base_link";
  pose_array.header.stamp = pc_msg->header.stamp;

  perception_msgs::msg::PerceptionMsg perception_msg;
  perception_msg.block_poses.header.frame_id = "base_link";
  perception_msg.block_poses.header.stamp = pc_msg->header.stamp;  

  visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::TransformStamped optical_to_base_link;
  try
  {
    optical_to_base_link =
      tf_buffer_->lookupTransform("base_link", pc_msg->header.frame_id, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_INFO(this->get_logger(), "Couldn't find transform: %s", ex.what());
    return;
  }
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  int marker_id = 0;
  pcl::PointCloud<PointT>::Ptr debug_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr ransac_cloud(new pcl::PointCloud<PointT>);

  for (const auto & det : det_msg->detections)
  {
    // float scale_x = static_cast<float>(pc_msg->width) / 1280.0f;
    // float scale_y = static_cast<float>(pc_msg->height) / 720.0f;
    float center_x = det.bbox.center.position.x * 1;
    float center_y = det.bbox.center.position.y * 1;

    float size_x = det.bbox.size_x * 1;  // width basically(along x)
    float size_y = det.bbox.size_y * 1;  // height basically(along y)

    float angle = det.bbox.center.theta * (180.0f / CV_PI);

    cv::RotatedRect rotated_rect(
      cv::Point2f(center_x, center_y), cv::Size2f(size_x, size_y), angle);

    // Expanded rect: adds a margin of table pixels around the brick footprint so
    // RANSAC has actual table points to find (a tight crop is almost all brick-top,
    // which makes RANSAC lock onto the brick's own top face instead of the table).
    const float margin_px = 8.0f;  // we have to tune this based on res
    cv::RotatedRect expanded_rect = rotated_rect;
    expanded_rect.size.width += 2 * margin_px;
    expanded_rect.size.height += 2 * margin_px;

    cv::Mat mask = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_8UC1);
    cv::Mat mask_tight = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_8UC1);

    cv::Point2f vertices_f[4];
    expanded_rect.points(vertices_f);

    cv::Point vertices_i[4];
    for (int i = 0; i < 4; i++)
    {
      vertices_i[i] = vertices_f[i];
    }

    cv::fillConvexPoly(mask, vertices_i, 4, cv::Scalar(255));

    // Tight mask, used only to get a reference point near the brick's
    // true center for cluster selection later, not for RANSAC.
    cv::Point2f vertices_f_tight[4];
    rotated_rect.points(vertices_f_tight);
    cv::Point vertices_i_tight[4];
    for (int i = 0; i < 4; i++)
    {
      vertices_i_tight[i] = vertices_f_tight[i];
    }
    cv::fillConvexPoly(mask_tight, vertices_i_tight, 4, cv::Scalar(255));

    cv::Rect bounding_box =
      expanded_rect.boundingRect();  // upright bounding box surrounding the expanded rotated rect

    int min_u = std::max(0, bounding_box.x);
    int max_u = std::min(mask.cols - 1, bounding_box.x + bounding_box.width);
    int min_v = std::max(0, bounding_box.y);
    int max_v = std::min(mask.rows - 1, bounding_box.y + bounding_box.height);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rough_optical_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tight_optical_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int v = min_v; v <= max_v; v++)
    {
      for (int u = min_u; u <= max_u; u++)
      {
        if (mask.at<uchar>(v, u) == 255)
        {
          int memory_index = v * pc_msg->width + u;
          pcl::PointXYZRGB point = optical_cloud->points[memory_index];

          if (!std::isnan(point.z))
          {
            rough_optical_cloud->push_back(point);
            if (mask_tight.at<uchar>(v, u) == 255)
            {
              tight_optical_cloud->push_back(point);
            }
          }
        }
      }
    }

    if (rough_optical_cloud->points.size() < 10)
    {
      RCLCPP_WARN(this->get_logger(), "Point Cloud too small");
      continue;
    }
    // Transform to World Space
    // Apply TF2 transform to isolated points
    Eigen::Affine3d eigen_transform = tf2::transformToEigen(optical_to_base_link);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*rough_optical_cloud, *transformed_cloud, eigen_transform);

    // Reference centroid near the true brick center (from the tight, unexpanded crop),
    // used below to pick the correct cluster when the expanded crop grabs a neighboring
    // brick's points too.
    Eigen::Vector3f reference_point(0.0f, 0.0f, 0.0f);
    bool have_reference_point = false;
    if (!tight_optical_cloud->points.empty())
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_tight_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud(*tight_optical_cloud, *transformed_tight_cloud, eigen_transform);
      Eigen::Vector4f tight_centroid;
      pcl::compute3DCentroid(*transformed_tight_cloud, tight_centroid);
      reference_point = tight_centroid.head<3>();
      have_reference_point = true;
    }

    // Geometric Filtering
    // Run RANSAC to remove table
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(ransac_distance_threshold_);

    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(2.0f * (M_PI / 180.0f));

    seg.setInputCloud(transformed_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty() || coefficients->values.size() < 3)
    {
      RCLCPP_WARN(this->get_logger(), "RANSAC failed! Skipping this brick.");
      continue;
    }

    pcl::ExtractIndices<PointT> extract;
    pcl::PointCloud<PointT>::Ptr rough_brick_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr ransac_brick_cloud(new pcl::PointCloud<PointT>);

    extract.setInputCloud(transformed_cloud);
    extract.setIndices(inliers);

    extract.setNegative(true);
    extract.filter(*rough_brick_cloud);

    extract.setNegative(false);
    extract.filter(*ransac_brick_cloud);

    *ransac_cloud += *ransac_brick_cloud;

    if (rough_brick_cloud->points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Cloud empty after table removal! Skipping.");
      continue;
    }

    // Run Euclidean Clustering to isolate the pure brick
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(rough_brick_cloud);

    std::vector<pcl::PointIndices> indices_cluster;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setInputCloud(rough_brick_cloud);
    ec.setSearchMethod(tree);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(25000);
    ec.setClusterTolerance(cluster_tolerance_);

    ec.extract(indices_cluster);

    if (indices_cluster.empty())
    {
      continue;
    }

    // Pick the cluster closest to the reference point (tight-crop centroid) rather
    // than blindly taking the largest one. This matters because the expanded crop
    // used for RANSAC can pull in a sliver of a neighboring brick; that sliver could,
    // in principle, form a larger cluster than the true (partially-occluded) brick.
    int best_cluster_idx = 0;
    if (have_reference_point)
    {
      float best_dist_sq = std::numeric_limits<float>::max();
      for (size_t c = 0; c < indices_cluster.size(); ++c)
      {
        Eigen::Vector3f sum(0.0f, 0.0f, 0.0f);
        for (const auto & idx : indices_cluster[c].indices)
        {
          const auto & p = rough_brick_cloud->points[idx];
          sum += Eigen::Vector3f(p.x, p.y, p.z);
        }
        Eigen::Vector3f cluster_centroid =
          sum / static_cast<float>(indices_cluster[c].indices.size());
        float dist_sq = (cluster_centroid - reference_point).squaredNorm();
        if (dist_sq < best_dist_sq)
        {
          best_dist_sq = dist_sq;
          best_cluster_idx = static_cast<int>(c);
        }
      }
    }

    pcl::PointCloud<PointT>::Ptr brick_cloud(new pcl::PointCloud<PointT>);
    for (const auto & idx : indices_cluster[best_cluster_idx].indices)
    {
      brick_cloud->push_back(rough_brick_cloud->points[idx]);
    }
    // Math Engine & Pose
    // Call estimateHybridOrientation()
    Eigen::Vector3f table_normal{
      coefficients->values[0], coefficients->values[1], coefficients->values[2]};

    float theta = det.bbox.center.theta;
    if (table_normal(2) < 0)
    {
      table_normal = -table_normal;
    }

    RCLCPP_INFO(
      this->get_logger(), "table_normal: [%f, %f, %f]", table_normal.x(), table_normal.y(),
      table_normal.z());

    *debug_cloud += *brick_cloud;

    geometry_msgs::msg::Pose final_pose =
      estimateHybridOrientation(brick_cloud, theta, table_normal);
    std::string combined_id_string = det.id; 
    size_t dotPos = combined_id_string.find('.');
      perception_msg.block_classes.push_back(combined_id_string.substr(0, dotPos));
      perception_msg.block_ids.push_back(combined_id_string.substr(dotPos + 1));
      perception_msg.block_poses.poses.push_back(final_pose);
      pose_array.poses.push_back(final_pose);
    // Oriented Bounding Box (OBB)
    // Un-rotate the cloud, get AABB(Axis-Aligned Bounding Boxes) dimensions, generate RViz Marker
    Eigen::Quaternionf q_final(
      final_pose.orientation.w, final_pose.orientation.x, final_pose.orientation.y,
      final_pose.orientation.z);
    Eigen::Matrix3f rotation_matrix = q_final.toRotationMatrix();
    Eigen::Matrix4f inv_transform = Eigen::Matrix4f::Identity();
    inv_transform.block<3, 3>(0, 0) = rotation_matrix.transpose();
    Eigen::Vector3f centroid(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    inv_transform.block<3, 1>(0, 3) = -rotation_matrix.transpose() * centroid;

    pcl::PointCloud<PointT>::Ptr unrotated_cloud(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*brick_cloud, *unrotated_cloud, inv_transform);

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*unrotated_cloud, min_pt, max_pt);

    visualization_msgs::msg::Marker obb_marker;
    obb_marker.header.frame_id = "base_link";
    obb_marker.header.stamp = pc_msg->header.stamp;
    obb_marker.ns = "obb_boxes";
    obb_marker.id = marker_id++;
    obb_marker.type = visualization_msgs::msg::Marker::CUBE;
    obb_marker.action = visualization_msgs::msg::Marker::ADD;
    obb_marker.pose = final_pose;

    obb_marker.scale.x = std::max(0.001f, max_pt.x - min_pt.x);
    obb_marker.scale.y = std::max(0.001f, max_pt.y - min_pt.y);
    obb_marker.scale.z = std::max(0.001f, max_pt.z - min_pt.z);
    obb_marker.color.r = 0.0;
    obb_marker.color.g = 1.0;
    obb_marker.color.b = 1.0;
    obb_marker.color.a = 0.5;
    marker_array.markers.push_back(obb_marker);
    
    shape_msgs::msg::SolidPrimitive shape; 
    shape.type = shape_msgs::msg::SolidPrimitive::BOX; 
    shape.dimensions.resize(3);
    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = obb_marker.scale.x;
    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = obb_marker.scale.y;
    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = obb_marker.scale.z;

    perception_msg.block_dims.push_back(shape); 
  }

  // Publish outputs
  if (!pose_array.poses.empty())
  {
    pose_pub_->publish(pose_array);
    block_data_pub_->publish(perception_msg);
  }

  marker_pub_->publish(marker_array);

  sensor_msgs::msg::PointCloud2 debug_msg;
  pcl::toROSMsg(*debug_cloud, debug_msg);

  debug_msg.header.frame_id = "base_link";
  debug_msg.header.stamp = pc_msg->header.stamp;

  debug_cloud_pub_->publish(debug_msg);

  sensor_msgs::msg::PointCloud2 ransac_msg;
  pcl::toROSMsg(*ransac_cloud, ransac_msg);

  ransac_msg.header.frame_id = "base_link";
  ransac_msg.header.stamp = pc_msg->header.stamp;

  ransac_cloud_pub_->publish(ransac_msg);
}
void PclGeometryNode::handleToggle(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    active_ = request->data; 

    if(active_){
      RCLCPP_INFO(this->get_logger(), "PCL Node Waking Up");
    }
    else
      RCLCPP_INFO(this->get_logger(), "PCL Node Going to Sleep"); 

    response->success = true; 
    response->message = active_? "Activated" : "Deactivated"; 
  }


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclGeometryNode>());
  rclcpp::shutdown();
  return 0;
}
