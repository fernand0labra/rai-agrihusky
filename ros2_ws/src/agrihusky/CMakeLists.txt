cmake_minimum_required(VERSION 3.14.4)
project(agrihusky)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ublox_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ProbeRequest.srv"
  "srv/WaypointRequest.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

#############
## Install ##
#############

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# Install Python executables
install(PROGRAMS
  src/simulation/fake_odometry_pub.py
  src/simulation/fake_gps_pub.py
  src/simulation/visualizer.py
  src/tf/tf_gps_pub.py
  src/tf/tf_imu_pub.py
  src/probe_interface.py
  src/waypoint_pub.py
  src/controller.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()