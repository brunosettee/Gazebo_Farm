cmake_minimum_required(VERSION 3.0.2)
project(data_collection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
