cmake_minimum_required(VERSION 3.0.2)
project(ed_perception)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ed
  ${PROJECT_NAME}_msgs
  ed_sensor_integration
  geolib2
  image_recognition_msgs
  rgbd
  rosconsole_bridge
  roscpp
  tue_config
  tue_filesystem
)

#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_console_bridge
  # Foward geolib2 and tue_filesystem logging to rosconsole
  src/rosconsole_bridge.cpp
)

# ------------------------------------------------------------------------------------------------
#                                                PLUGIN
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_plugin_image_recognition src/perception_plugin_image_recognition.cpp)
target_link_libraries(${PROJECT_NAME}_plugin_image_recognition ${PROJECT_NAME}_console_bridge ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_plugin_image_recognition ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_library(train-and-test-lib src/annotated_image.cpp src/image_crawler.cpp)
target_link_libraries(train-and-test-lib ${PROJECT_NAME}_console_bridge ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(store_segments src/store_segments.cpp)
target_link_libraries(store_segments ${PROJECT_NAME}_console_bridge train-and-test-lib)

add_executable(annotation-gui src/annotation_gui.cpp)
target_link_libraries(annotation-gui ${PROJECT_NAME}_console_bridge train-and-test-lib)

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
    scripts/face_recognizer_dummy_node
    scripts/load_object_types
    scripts/object_recognition_dummy_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}_console_bridge
    ${PROJECT_NAME}_plugin_image_recognition
    train-and-test-lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    annotation-gui
    store_segments
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2")
endif()

