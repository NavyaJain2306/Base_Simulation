cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(pcl_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search kdtree)

add_executable(pcl_geometry_node src/pcl_geometry_node.cpp)

# Include external library headers
target_include_directories(pcl_geometry_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Link ROS 2 dependencies
ament_target_dependencies(pcl_geometry_node
  rclcpp
  sensor_msgs
  vision_msgs
  geometry_msgs
  visualization_msgs
  shape_msgs
  std_srvs
  perception_msgs
  message_filters
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

# Link external C++ libraries directly
target_link_libraries(pcl_geometry_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
  pcl_geometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
