cmake_minimum_required(VERSION 3.0.2)
project(visual_odometry)

#set(OpenCV_DIR /usr/local/include/opencv4)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

find_package(OpenCV 4 REQUIRED) 

catkin_package()

include_directories(
# include  #-> Da scommentare per librerie
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(visual_Odometry src/visual_Odometry.cpp)
target_link_libraries(visual_Odometry 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})