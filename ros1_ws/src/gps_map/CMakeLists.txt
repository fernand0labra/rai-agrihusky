cmake_minimum_required(VERSION 3.0.2)
project(gps_map)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  ublox_msg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# No compilation needed for pure Python node
catkin_install_python(PROGRAMS
  src/visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
