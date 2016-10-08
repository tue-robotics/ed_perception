cmake_minimum_required(VERSION 2.8.3)
project(ed_perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  ann2
  ed
  ed_object_models
  ed_sensor_integration
  genmsg
  geolib2
  message_generation
  rgbd
  roscpp
  roslib
  tue_config
  tue_filesystem
  vocabulary_tree
  std_srvs
)

#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# Qt GUI
#find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#include(${QT_USE_FILE})

###################################
## catkin specific configuration ##
###################################

add_message_files(
  FILES
    CategoricalDistribution.msg
    PersonDetection.msg
)

add_service_files(
  FILES
    LearnPerson.srv
    Classify.srv
    AddTrainingInstance.srv
    LearnPerson.srv
    RecognizePerson.srv
)

add_action_files(
  DIRECTORY
    action
  FILES
    FaceLearning.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs
)

catkin_package()


###########
## Build ##
###########

# ------------------------------------------------------------------------------------------------

add_library(ed_perception
  include/ed/perception/categorical_distribution.h
  include/ed/perception/module.h
  include/ed/perception/aggregator.h

  src/categorical_distribution.cpp
  src/aggregator.cpp
)
target_link_libraries(ed_perception ${catkin_LIBRARIES})
add_dependencies(ed_perception ${PROJECT_NAME}_generate_messages_cpp)

# ------------------------------------------------------------------------------------------------

add_library(ed_perception_plugin src/perception_plugin.cpp)
target_link_libraries(ed_perception_plugin ${catkin_LIBRARIES} ed_perception)

# ------------------------------------------------------------------------------------------------

add_library(ed_perception_plugin_tensorflow src/perception_plugin_tensorflow.cpp)
target_link_libraries(ed_perception_plugin_tensorflow ${catkin_LIBRARIES} ed_perception)

# ------------------------------------------------------------------------------------------------

include_directories(
  include
  # ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  3rdparty/people_detector/PeopleDetector
  3rdparty/people_detector/common
  3rdparty/people_detector/features
  3rdparty/people_detector/linalg
)


# ----- Build Oscar Martinez Mozos people detector -----
file(GLOB_RECURSE PEOPLE_DETECTOR_SRC
  3rdparty/people_detector/PeopleDetector/*.cpp
  3rdparty/people_detector/common/*.cpp
  3rdparty/people_detector/features/*.cpp
  3rdparty/people_detector/linalg/*.cc
)

add_library(people_detector ${PEOPLE_DETECTOR_SRC})
target_link_libraries(people_detector gsl gslcblas m)

# ------------------------------------------------------------------------------------------------
#                                          PERCEPTION MODULES
# ------------------------------------------------------------------------------------------------

add_library(shared_methods
  plugins/shared_methods.cpp
  plugins/shared_methods.h
)
target_link_libraries(shared_methods ${catkin_LIBRARIES})

add_library(size_matcher
  plugins/size_matcher.cpp
  plugins/size_matcher.h
)
target_link_libraries(size_matcher shared_methods ${catkin_LIBRARIES})

#add_library(human_contour_matcher
#    plugins/human_contour_matcher.cpp
#    plugins/human_contour_matcher.h
#    plugins/human_classifier.cpp
#    plugins/human_classifier.h
#    plugins/human_classifier_structs.h
##    plugins/opencl_toolbox.cpp
##    plugins/opencl_toolbox.h
#)
#target_link_libraries(human_contour_matcher shared_methods ${catkin_LIBRARIES})

add_library(face_detector
    plugins/face_detector.cpp
    plugins/face_detector.h
)
target_link_libraries(face_detector shared_methods ${catkin_LIBRARIES})

#add_library(face_recognition_ed
#    plugins/face_recognition.cpp
#    plugins/face_recognition.h
#)
#target_link_libraries(face_recognition_ed shared_methods shared_methods ${catkin_LIBRARIES})

#add_library(open_biometrics_ed
#    plugins/open_biometrics_ed.cpp
#    plugins/open_biometrics_ed.h
#)
#target_link_libraries(open_biometrics_ed shared_methods ${catkin_LIBRARIES} ${Qt5_LIBRARIES})
# target_link_libraries(open_biometrics_ed ${catkin_LIBRARIES} ${Qt5_LIBRARIES} /usr/local/lib/libopenbr.so)

#add_library(ed_odu_finder
#    plugins/odu_finder/odu_finder_module.cpp
#    plugins/odu_finder/odu_finder_module.h
#    plugins/odu_finder/odu_finder.cpp
#    plugins/odu_finder/odu_finder.h
#)
#target_link_libraries(ed_odu_finder shared_methods ${catkin_LIBRARIES} /usr/local/lib/libsiftfast.so)

#add_library(qr_detector
#    plugins/qr_detector.cpp
#    plugins/qr_detector.h
#    plugins/qr_detector_zbar/qr_detector_zbar.cpp
#    plugins/qr_detector_zbar/qr_detector_zbar.h
#)
#target_link_libraries(qr_detector ${catkin_LIBRARIES} /usr/lib/libzbar.so)

add_library(color_matcher
  plugins/color_matcher/color_matcher.cpp
  plugins/color_matcher/color_matcher.h
  plugins/color_matcher/color_name_table.cpp
  plugins/color_matcher/color_name_table.h
)
target_link_libraries(color_matcher shared_methods ${catkin_LIBRARIES})

#add_library(laser_human_chest_classifier
#    plugins/laser_human_classifier/laser_human_chest_classifier.cpp
#    plugins/laser_human_classifier/laser_human_chest_classifier.h
#)
#target_link_libraries(laser_human_chest_classifier people_detector shared_methods ${catkin_LIBRARIES})

#add_library(odu_finder_db_builder tools/odu_finder_db_builder.cpp)
#target_link_libraries(odu_finder_db_builder ${catkin_LIBRARIES} /usr/local/lib/libsiftfast.so)

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_library(train-and-test-lib tools/image_crawler.cpp tools/annotated_image.cpp)
target_link_libraries(train-and-test-lib ed_perception ${catkin_LIBRARIES})

add_executable(train-perception tools/train_perception.cpp)
target_link_libraries(train-perception train-and-test-lib ed_perception)

add_executable(test-perception tools/test_perception.cpp tools/confusionmatrix.cpp)
target_link_libraries(test-perception train-and-test-lib ed_perception ${OpenCV_LIBRARIES})

add_executable(annotation-gui tools/annotation_gui.cpp)
target_link_libraries(annotation-gui train-and-test-lib ed_perception ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                                NODES
# ------------------------------------------------------------------------------------------------

add_executable(face_recognition
    nodes/face_recognition.cpp
    nodes/face_recognizer.cpp
    nodes/face_recognizer.h
)
target_link_libraries(face_recognition ${catkin_LIBRARIES})

# Qt Package
# find_package(Qt4 COMPONENTS QtCore QtGui QtApplication)
# include(${QT_USE_FILE})
# Create Moc files
# QT4_WRAP_CPP(QT4ENTITYVIEWERGUI tools/entity_live_viewer/viewer_ui.h)
#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)


