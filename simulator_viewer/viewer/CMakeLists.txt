## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 3.0.2)
project(viewer)

find_package(catkin REQUIRED COMPONENTS
  rviz
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  xycar_data
)
find_package(OpenCV REQUIRED)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})


## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)


## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/viewer2.cpp
  src/viewer1.cpp
  src/main.cpp
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(viewer ${SRC_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(viewer ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
## END_TUTORIAL

## Install
install(TARGETS viewer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_dependencies(viewer ${catkin_EXPORTED_TARGETS})
