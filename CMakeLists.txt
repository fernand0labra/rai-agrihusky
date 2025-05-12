cmake_minimum_required(VERSION 3.14.4)
project(rai_agrihusky)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

#############
## Install ##
#############

# Install Python executables
install(PROGRAMS
  src/fake_odometry_pub.py
  src/fake_waypoint_pub.py
  src/controller.py
  src/visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()