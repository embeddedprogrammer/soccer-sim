cmake_minimum_required(VERSION 2.8.3)
project(walle)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation cv_bridge genmsg image_transport sensor_msgs)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Num.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(talker src/Vision/talker.cpp)
find_package( OpenCV REQUIRED )
target_link_libraries(talker ${OpenCV_LIBS})
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/Robot/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)