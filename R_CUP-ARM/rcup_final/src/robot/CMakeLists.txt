cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)


install(DIRECTORY config meshes urdf worlds models launch maps DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS
  scripts/destination.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/align.py
  scripts/destination.py
  scripts/gazebo_station_relay.py
  scripts/wait_for_active.py
  DESTINATION lib/${PROJECT_NAME}
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
