cmake_minimum_required(VERSION 3.5)
project(vizanti_msgs)

set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# List of service files
set(srv_files
    "srv/GetNodeParameters.srv"
    "srv/SetNodeParameter.srv"
    "srv/SaveMap.srv"
    "srv/LoadMap.srv"
    "srv/RecordRosbag.srv"
    "srv/ManageNode.srv"
    "srv/ListPackages.srv"
    "srv/ListExecutables.srv"
    "srv/ListLifecycles.srv"
)

# Generate messages / actions /  services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs nav_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
