cmake_minimum_required(VERSION 2.8.3)
project(ed_perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  class_loader
  cv_bridge
  ed_object_models
  geolib2
  geometry_msgs
  #libsiftfast
  rgbd
  pcl_conversions
  roscpp
  roslib
  std_srvs
  tue_config
  tue_filesystem
  tue_serialization
  vocabulary_tree
  message_generation
  visualization_msgs
  nav_msgs
  ann2
  ed
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------
#                                          PERCEPTION MODULES
# ------------------------------------------------------------------------------------------------

add_library(type_aggregator
    plugins/type_aggregator.cpp
    plugins/type_aggregator.h
)
target_link_libraries(type_aggregator ed_perception)

add_library(size_matcher
    plugins/size_matcher.cpp
    plugins/size_matcher.h
)
target_link_libraries(size_matcher ed_perception)

add_library(human_contour_matcher
    plugins/human_contour_matcher.cpp
    plugins/human_contour_matcher.h
    plugins/human_classifier.cpp
    plugins/human_classifier.h
    plugins/human_classifier_structs.h
#    plugins/opencl_toolbox.cpp
#    plugins/opencl_toolbox.h
)
target_link_libraries(human_contour_matcher ed_perception)

add_library(face_detector
    plugins/face_detector.cpp
    plugins/face_detector.h
)
target_link_libraries(face_detector ed_perception)

add_library(ed_odu_finder
    plugins/odu_finder/odu_finder_module.cpp
    plugins/odu_finder/odu_finder_module.h
    plugins/odu_finder/odu_finder.cpp
    plugins/odu_finder/odu_finder.h
)
target_link_libraries(ed_odu_finder ed_perception /usr/local/lib/libsiftfast.so)

add_library(qr_detector
    plugins/qr_detector.cpp
    plugins/qr_detector.h
    plugins/qr_detector_zbar/qr_detector_zbar.cpp
    plugins/qr_detector_zbar/qr_detector_zbar.h
)
target_link_libraries(qr_detector ed_perception)
target_link_libraries(qr_detector /usr/lib/libzbar.so)


add_library(color_matcher
    plugins/color_matcher/color_matcher.cpp
    plugins/color_matcher/color_matcher.h
    plugins/color_matcher/color_name_table.cpp
    plugins/color_matcher/color_name_table.h
)
target_link_libraries(color_matcher ed_perception)

add_library(odu_finder_learner
    tools/odu_finder_learner.cpp
)
target_link_libraries(odu_finder_learner ed_perception /usr/local/lib/libsiftfast.so)

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(view-measurement tools/view_measurement.cpp)
target_link_libraries(view-measurement ed_core ed_io ${OpenCV_LIBRARIES})

add_executable(test-perception tools/test_perception.cpp)
target_link_libraries(test-perception ed_core ed_perception ed_io ${OpenCV_LIBRARIES})

add_executable(create-rgbd-measurement tools/create_rgbd_measurement.cpp)
target_link_libraries(create-rgbd-measurement ed_core ed_io ${OpenCV_LIBRARIES})

add_executable(odu-finder-db-builder tools/odu_finder_db_builder.cpp)
target_link_libraries(odu-finder-db-builder ${OpenCV_LIBRARIES} /usr/local/lib/libsiftfast.so ${catkin_LIBRARIES})

add_executable(ed-learning-tool tools/ed_learning_tool.cpp)
target_link_libraries(ed-learning-tool ed_core ed_perception ed_io odu_finder_learner ${OpenCV_LIBRARIES})

