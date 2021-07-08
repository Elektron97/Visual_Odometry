cmake_minimum_required(VERSION 3.0.2)
project(visual_odometry)

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

find_package(OpenCV 3 REQUIRED)

catkin_package()

include_directories(
# include  #-> Da scommentare per librerie
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(show_image src/show_image.cpp)
target_link_libraries(show_image 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

add_executable(show_video src/show_video.cpp)
target_link_libraries(show_video 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

add_executable(visual_Odometry src/visual_Odometry.cpp)
target_link_libraries(visual_Odometry 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})