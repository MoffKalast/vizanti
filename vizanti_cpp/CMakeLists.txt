cmake_minimum_required(VERSION 3.22)
project(vizanti_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf_consolidator src/tf_consolidator.cpp)

target_link_libraries(tf_consolidator
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

install(TARGETS
  tf_consolidator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
