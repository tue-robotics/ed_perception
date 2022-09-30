cmake_minimum_required(VERSION 3.0.2)
project(ed_perception)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ed
  ed_object_models
  ed_perception_msgs
  ed_sensor_integration
  geolib2
  image_recognition_msgs
  rgbd
  roscpp
  rospack
  tue_config
  tue_filesystem
  std_srvs
)

#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  SYSTEM
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
add_dependencies(ed_perception_plugin_image_recognition ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_library(train-and-test-lib src/image_crawler.cpp src/annotated_image.cpp)
target_link_libraries(train-and-test-lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(store_segments src/store_segments.cpp)
target_link_libraries(store_segments train-and-test-lib)

add_executable(annotation-gui src/annotation_gui.cpp)
target_link_libraries(annotation-gui train-and-test-lib)

