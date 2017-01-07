cmake_minimum_required(VERSION 2.8.3)
project(soccersim)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    gazebo_ros
    std_msgs
    std_srvs
    geometry_msgs
    message_generation
    cv_bridge
    genmsg
    image_transport
    sensor_msgs
    xacro
    soccerref
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

# Declare ROS messages
#add_message_files(DIRECTORY msg FILES)

# Generate added messages
#generate_messages(DEPENDENCIES std_msgs std_srvs geometry_msgs)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs message_runtime gazebo_ros soccerref)

# Specify additional locations of header files
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Declare C++ executables
add_executable(vision_sim src/vision.cpp)
#add_dependencies(vision_sim ${PROJECT_NAME}_generate_messages_cpp)
find_package( OpenCV REQUIRED )
target_link_libraries(vision_sim ${OpenCV_LIBS} ${catkin_LIBRARIES})

# Add plugins
add_library(soccer_drive plugins/soccer_drive.cpp)
add_dependencies(soccer_drive soccerref_generate_messages_cpp)
target_link_libraries(soccer_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(soccer_ball plugins/soccer_ball.cpp)
add_dependencies(soccer_ball soccerref_generate_messages_cpp)
target_link_libraries(soccer_ball ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Generate .world files from .world.xacro files
file(GLOB xacro_files worlds/*.world.xacro)
xacro_add_files(${xacro_files} TARGET media_files INORDER)