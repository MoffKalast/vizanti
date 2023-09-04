cmake_minimum_required(VERSION 3.0.2)
project(vizanti)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
    actionlib_msgs
    move_base_msgs
    std_msgs
    message_generation
)

add_service_files(
    FILES
    GetNodeParameters.srv
    SaveMap.srv
    LoadMap.srv
    RecordRosbag.srv
    ManageNode.srv
    ListPackages.srv
    ListExecutables.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)


catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure rospy actionlib_msgs geometry_msgs move_base_msgs std_msgs message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

file(
    GLOB PYTHON_SCRIPTS_DIR "scripts/*.py"
)
catkin_install_python(
    PROGRAMS ${PYTHON_SCRIPTS_DIR}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY public launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)