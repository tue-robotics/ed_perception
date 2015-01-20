cmake_minimum_required(VERSION 2.8.3)
project(ed_perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ed_object_models
  geolib2
  rgbd
  roscpp
  roslib
  tue_config
  tue_filesystem
  vocabulary_tree
  message_generation
  ann2
  ed
)

#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

add_service_files(
   FILES
   LearnPerson.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)





###########
## Build ##
###########

include_directories(
#    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------
#                                          PERCEPTION MODULES
# ------------------------------------------------------------------------------------------------

add_library(type_aggregator
    plugins/type_aggregator.cpp
    plugins/type_aggregator.h
)
target_link_libraries(type_aggregator ${catkin_LIBRARIES})

add_library(size_matcher
    plugins/size_matcher.cpp
    plugins/size_matcher.h
)
target_link_libraries(size_matcher ${catkin_LIBRARIES})

add_library(human_contour_matcher
    plugins/human_contour_matcher.cpp
    plugins/human_contour_matcher.h
    plugins/human_classifier.cpp
    plugins/human_classifier.h
    plugins/human_classifier_structs.h
#    plugins/opencl_toolbox.cpp
#    plugins/opencl_toolbox.h
)
target_link_libraries(human_contour_matcher ${catkin_LIBRARIES})

add_library(face_detector
    plugins/face_detector.cpp
    plugins/face_detector.h
)
target_link_libraries(face_detector ${catkin_LIBRARIES})

add_library(face_recognition_ed
    plugins/face_recognition.cpp
    plugins/face_recognition.h
)
target_link_libraries(face_recognition_ed ${catkin_LIBRARIES})

add_library(ed_odu_finder
    plugins/odu_finder/odu_finder_module.cpp
    plugins/odu_finder/odu_finder_module.h
    plugins/odu_finder/odu_finder.cpp
    plugins/odu_finder/odu_finder.h
)
target_link_libraries(ed_odu_finder ${catkin_LIBRARIES} /usr/local/lib/libsiftfast.so)

add_library(qr_detector
    plugins/qr_detector.cpp
    plugins/qr_detector.h
    plugins/qr_detector_zbar/qr_detector_zbar.cpp
    plugins/qr_detector_zbar/qr_detector_zbar.h
)
target_link_libraries(qr_detector ${catkin_LIBRARIES} /usr/lib/libzbar.so)


add_library(color_matcher
    plugins/color_matcher/color_matcher.cpp
    plugins/color_matcher/color_matcher.h
    plugins/color_matcher/color_name_table.cpp
    plugins/color_matcher/color_name_table.h
)
target_link_libraries(color_matcher ${catkin_LIBRARIES})

add_library(odu_finder_db_builder
    tools/odu_finder_db_builder.cpp
)
target_link_libraries(odu_finder_db_builder ${catkin_LIBRARIES} /usr/local/lib/libsiftfast.so)

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(view-measurement tools/view_measurement.cpp)
target_link_libraries(view-measurement ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test-perception tools/test_perception.cpp)
target_link_libraries(test-perception ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(create-rgbd-measurement tools/create_rgbd_measurement.cpp)
target_link_libraries(create-rgbd-measurement ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ed-learning-tool tools/ed_learning_tool.cpp)
target_link_libraries(ed-learning-tool odu_finder_db_builder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

