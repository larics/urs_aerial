cmake_minimum_required(VERSION 2.8.3)
project(urs_optitrack)

find_package(catkin REQUIRED
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES urs_optitrack
#  CATKIN_DEPENDS rospy vrpn_client_ros
#  DEPENDS system_lib
)