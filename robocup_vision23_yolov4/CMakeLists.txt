##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(robocup_vision23_yolov4)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp rospy cv_bridge message_runtime message_generation image_transport std_msgs roslib sensor_msgs image_transport msg_generate)
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

set(OpenCV_INClUDE_DIRS /usr/local/include/opencv4/opencv2)

set(Opencv_LIBS
    /usr/local/lib/libopencv_highgui.so
    /usr/local/lib/libopencv_videoio.so
    /usr/local/lib/libopencv_features2d.so
    /usr/local/lib/libopencv_flann.so
    /usr/local/lib/libopencv_calib3d.so
    /usr/local/lib/libopencv_dnn.so
    )

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/robocup_vision23_yolov4/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp include/pid_control_float/*.cpp include/robit_vision/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(robocup_vision23_yolov4 ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(robocup_vision23_yolov4 ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Opencv_LIBS})
install(TARGETS robocup_vision23_yolov4 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

