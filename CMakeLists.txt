cmake_minimum_required(VERSION 2.8.3)
project(ed_perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ed
  ed_object_models
  ed_sensor_integration
  genmsg
  geolib2
  message_generation
  image_recognition_msgs
  rgbd
  roscpp
  roslib
  tue_config
  tue_filesystem
  std_srvs
)

#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

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
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

# ------------------------------------------------------------------------------------------------
#                                                PLUGIN
# ------------------------------------------------------------------------------------------------

add_library(ed_perception_plugin_image_recognition src/perception_plugin_image_recognition.cpp)
target_link_libraries(ed_perception_plugin_image_recognition ${catkin_LIBRARIES})
add_dependencies(ed_perception_plugin_image_recognition ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_library(train-and-test-lib src/image_crawler.cpp src/annotated_image.cpp)
target_link_libraries(train-and-test-lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(store_segments src/store_segments.cpp)
target_link_libraries(store_segments train-and-test-lib)

add_executable(annotation-gui src/annotation_gui.cpp)
target_link_libraries(annotation-gui train-and-test-lib)

