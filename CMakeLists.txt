cmake_minimum_required(VERSION 2.8.3)
project(walle)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros rospy std_msgs geometry_msgs message_generation cv_bridge genmsg image_transport sensor_msgs)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

# Declare ROS messages and services
add_message_files(DIRECTORY msg FILES SoccerPoses.msg)

# Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime roscpp gazebo_ros)

# Specify additional locations of header files
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Declare C++ executables
add_executable(vision src/vision.cpp)
find_package( OpenCV REQUIRED )
target_link_libraries(vision ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(home src/home.cpp)
target_link_libraries(home ${catkin_LIBRARIES})

# Add plugins
add_library(model_push2 plugins/model_push.cc)
target_link_libraries(model_push2 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
