cmake_minimum_required(VERSION 2.8.3)
project(urs_solution)

find_package(catkin REQUIRED message_generation std_msgs dynamic_reconfigure)

add_message_files(
    FILES
    PIDController.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

generate_dynamic_reconfigure_options(
	cfg/MavAttitudeCtlParams.cfg
	cfg/MavZCtlParams.cfg
	cfg/MavXYCtlParams.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES urs_solution
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)