cmake_minimum_required(VERSION 3.0.2)
project(visual_odometry)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV 3 REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(opencv_test src/opencv_test.cpp)
target_link_libraries(opencv_test 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

#include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})