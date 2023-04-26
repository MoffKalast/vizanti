cmake_minimum_required(VERSION 3.0.2)
project(outdooros)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
    actionlib_msgs
    move_base_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure rospy actionlib_msgs geometry_msgs move_base_msgs std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    src/server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)