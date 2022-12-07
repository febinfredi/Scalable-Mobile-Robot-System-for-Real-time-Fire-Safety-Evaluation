cmake_minimum_required(VERSION 3.0.2)
project(map_score_detector)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  )

add_service_files(
  DIRECTORY
  srv
  FILES
  calculateChange.srv 
  )

catkin_package(
  LIBRARIES map_score_detector
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
  )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  )

catkin_install_python(PROGRAMS
   scripts/detect_changes.py
   scripts/map_score_server.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )