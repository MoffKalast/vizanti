cmake_minimum_required(VERSION 3.20)
project(vizanti_demos)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vizanti_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

install(DIRECTORY
  launch
  assets
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/_test_button.py
  scripts/_test_grid_cells.py
  scripts/_test_line_list.py
  scripts/_test_occupancy_grid.py
  scripts/_test_marker_array.py
  scripts/_test_parameters.py
  scripts/_test_sphere_list.py
  scripts/_test_tf.py
  scripts/particle_cloud_to_pose_array.py
  scripts/path_to_nav2poses.py
  scripts/waypoints_to_simple_goals.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
