cmake_minimum_required(VERSION 3.0.2)
project(visual_odometry)

#set(OpenCV_DIR /usr/local/include/opencv4)
#SET(cv_bridge_DIR /home/catkin_ws/devel/share/cv_bridge/cmake/)
#set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  pcl_ros
  marta_msgs
)

find_package(OpenCV 4 REQUIRED) 

#Generate messages in the msg folder
add_message_files(
  FILES
    vo_results.msg
    fail_check.msg
)

#Generate added messages with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()

include_directories(
  include  #-> Da scommentare per librerie
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(visual_Odometry src/visual_Odometry.cpp)
add_dependencies(visual_Odometry visual_odometry_generate_messages_cpp)
add_dependencies(visual_Odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_Odometry 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})