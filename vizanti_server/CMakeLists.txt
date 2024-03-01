cmake_minimum_required(VERSION 3.5)
project(vizanti_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(rosbridge_suite REQUIRED)

install(DIRECTORY
  launch
  public
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/server.py
  scripts/service_handler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
