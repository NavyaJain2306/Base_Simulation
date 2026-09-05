cmake_minimum_required(VERSION 3.8)

project(link_attatcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-common4 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Attach.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_library(link_attatcher_plugin SHARED
  src/link_attatcher.cc
)

target_include_directories(link_attatcher_plugin PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(link_attatcher_plugin
  rclcpp
)

target_link_libraries(link_attatcher_plugin
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
  ignition-common4::ignition-common4
)

# --- UPDATED SECTION START ---
# Fetch the target name for the cpp typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Link the generated typesupport library to your plugin
target_link_libraries(link_attatcher_plugin
  "${cpp_typesupport_target}"
)
# --- UPDATED SECTION END ---

install(
  TARGETS link_attatcher_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()