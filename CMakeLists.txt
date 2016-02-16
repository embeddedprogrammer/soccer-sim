cmake_minimum_required(VERSION 2.8.3)
project(walle)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation cv_bridge genmsg image_transport sensor_msgs)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES SoccerPoses.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare C++ executables
add_executable(vision src/vision.cpp)
find_package( OpenCV REQUIRED )
target_link_libraries(vision ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(home src/home.cpp src/motorControl.cpp)
target_link_libraries(home ${catkin_LIBRARIES})