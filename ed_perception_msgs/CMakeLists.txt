cmake_minimum_required(VERSION 2.8.3)
project(ed_perception_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

add_message_files(
  FILES
    CategoricalDistribution.msg
)

add_service_files(
  FILES
    Classify.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
