cmake_minimum_required(VERSION 2.8.3)
project(ed_perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ed_object_models
  ed_sensor_integration
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
  pein_srvs
  genmsg
  actionlib_msgs
  actionlib
)

#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# Qt GUI
#find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#include(${QT_USE_FILE})

###################################
## catkin specific configuration ##
###################################

add_service_files(
   FILES
   LearnPerson.srv
   Classify.srv
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
)

catkin_package()


###########
## Build ##
###########

# ------------------------------------------------------------------------------------------------

add_library(ed_perception2
    include/ed/perception/categorical_distribution.h
    include/ed/perception/worker_input.h
    include/ed/perception/worker_output.h
    include/ed/perception/module.h
    include/ed/perception/worker.h
    include/ed/perception/aggregator.h

    src/categorical_distribution.cpp
    src/module.cpp
    src/worker.cpp
    src/aggregator.cpp
)
target_link_libraries(ed_perception2 ${catkin_LIBRARIES})
add_dependencies(ed_perception2 ${PROJECT_NAME}_generate_messages_cpp)

# ------------------------------------------------------------------------------------------------

add_library(ed_perception_plugin
    src/perception_plugin.cpp
)
target_link_libraries(ed_perception_plugin ${catkin_LIBRARIES} ed_perception2)

# ------------------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------------------

include_directories(
    include
#    ${PCL_INCLUDE_DIRS}
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

add_library(type_aggregator
    plugins/type_aggregator.cpp
    plugins/type_aggregator.h
)
target_link_libraries(type_aggregator shared_methods ${catkin_LIBRARIES})

add_library(size_matcher
    plugins/size_matcher.cpp
    plugins/size_matcher.h
)
target_link_libraries(size_matcher shared_methods ${catkin_LIBRARIES})

add_library(human_contour_matcher
    plugins/human_contour_matcher.cpp
    plugins/human_contour_matcher.h
    plugins/human_classifier.cpp
    plugins/human_classifier.h
    plugins/human_classifier_structs.h
#    plugins/opencl_toolbox.cpp
#    plugins/opencl_toolbox.h
)
target_link_libraries(human_contour_matcher shared_methods ${catkin_LIBRARIES})

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

add_library(open_biometrics_ed
    plugins/open_biometrics_ed.cpp
    plugins/open_biometrics_ed.h
)
target_link_libraries(open_biometrics_ed shared_methods ${catkin_LIBRARIES} ${Qt5_LIBRARIES})
# target_link_libraries(open_biometrics_ed ${catkin_LIBRARIES} ${Qt5_LIBRARIES} /usr/local/lib/libopenbr.so)

add_library(ed_odu_finder
    plugins/odu_finder/odu_finder_module.cpp
    plugins/odu_finder/odu_finder_module.h
    plugins/odu_finder/odu_finder.cpp
    plugins/odu_finder/odu_finder.h
)
target_link_libraries(ed_odu_finder shared_methods ${catkin_LIBRARIES} /usr/local/lib/libsiftfast.so)

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
target_link_libraries(color_matcher shared_methods ${catkin_LIBRARIES})

add_library(laser_human_chest_classifier
    plugins/laser_human_classifier/laser_human_chest_classifier.cpp
    plugins/laser_human_classifier/laser_human_chest_classifier.h
)
target_link_libraries(laser_human_chest_classifier people_detector shared_methods ${catkin_LIBRARIES})

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
target_link_libraries(test-perception ed_perception2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test-perception2 tools/test_perception2.cpp)
target_link_libraries(test-perception2 ed_perception2 ed_perception_plugin ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(test-perception3 tools/test_perception3.cpp)
#target_link_libraries(test-perception3 ed_perception2 ed_perception_plugin ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(create-rgbd-measurement tools/create_rgbd_measurement.cpp)
target_link_libraries(create-rgbd-measurement ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# --------------

add_library(train-and-test-lib
    tools/train_and_test/image_crawler.cpp
    tools/train_and_test/annotated_image.cpp
)
target_link_libraries(train-and-test-lib ed_perception2 ${catkin_LIBRARIES})

add_executable(train-perception tools/train_and_test/train.cpp)
target_link_libraries(train-perception train-and-test-lib ed_perception2)

add_executable(test-perception-new tools/train_and_test/test.cpp tools/confusionmatrix.cpp)
target_link_libraries(test-perception-new train-and-test-lib ed_perception2 ${OpenCV_LIBRARIES})

# -------------- ED LEARNING TOOL --------------
add_executable(ed-learning-tool tools/ed_learning_tool.cpp)
target_link_libraries(ed-learning-tool ed_perception2 odu_finder_db_builder ed_perception_plugin shared_methods ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# -------------- ENTITY VIEWER OPEN CV --------------
add_executable(entity_live_viewer_cv
    tools/entity_live_viewer/entity_live_viewer_cv.cpp
)
target_link_libraries(entity_live_viewer_cv entity_viewer_common shared_methods ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# -------------- ENTITY VIEWER COMMON --------------

add_library(entity_viewer_common
    tools/entity_live_viewer/entity_viewer_common.cpp
    tools/entity_live_viewer/entity_viewer_common.h
)
target_link_libraries(entity_viewer_common ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



# -------------- ENTITY VIEWER PROBE --------------
add_library(entity_viewer_probe
    tools/entity_live_viewer/entity_viewer_probe.cpp
    tools/entity_live_viewer/entity_viewer_probe.h
)
target_link_libraries(entity_viewer_probe shared_methods entity_viewer_common ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# -------------- ENTITY VIEWER GUI --------------

# # # # # # # # # # # # # # # # # # # # # # # # # # #
#
#                         GUI
#
# # # # # # # # # # # # # # # # # # # # # # # # # # #

# Qt Package
find_package(Qt4 COMPONENTS QtCore QtGui QtApplication)
include(${QT_USE_FILE})

# Create Moc files
QT4_WRAP_CPP(QT4ENTITYVIEWERGUI tools/entity_live_viewer/viewer_ui.h)

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
#ADD_DEFINITIONS(${QT_DEFINITIONS})

#add_library(entity_viewer_gui
#    tools/entity_live_viewer/viewer_ui.cpp
#    tools/entity_live_viewer/viewer_ui.h
#)
#target_link_libraries(entity_viewer_gui
#    tools/entity_live_viewer/ui_entity_live_viewer_form.h
#    ${QT4ENTITYVIEWERGUI}
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${QT_LIBRARIES}
#)


add_executable(annotation-gui-new tools/train_and_test/annotation_gui.cpp)
target_link_libraries(annotation-gui-new train-and-test-lib ed_perception2 ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# -------------- ENTITY VIEWER CLIENT GUI --------------

#message("${QT_LIBRARIES} BLAAAAAAAAAA")

#add_executable(test_ui tools/entity_live_viewer/test_ui.cpp)

#target_link_libraries(test_ui
#    entity_viewer_gui
#    entity_viewer_common
#    ${catkin_LIBRARIES}
#    tools/entity_live_viewer/viewer_ui.h
#    tools/entity_live_viewer/viewer_ui.cpp
#    ${QT4ENTITYVIEWERGUI}
#    ${QT_LIBRARIES}
#)

# -------------- ANNOTATION GUI --------------
add_executable(annotation_gui tools/annotation_gui/src/annotation_gui.cpp)
target_link_libraries(annotation_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



