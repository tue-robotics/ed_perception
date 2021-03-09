cmake_minimum_required(VERSION 3.0.2)
project(ed_perception_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
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
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
