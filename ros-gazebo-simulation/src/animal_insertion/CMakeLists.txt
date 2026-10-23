cmake_minimum_required(VERSION 3.0.2)
project(animal_insertion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/spawn_mesh.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
