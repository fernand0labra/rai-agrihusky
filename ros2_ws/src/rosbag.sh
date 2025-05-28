ros2 launch innovusion ivu_pc2.py  # Lidar

ros2 launch depthai_ros_driver pointcloud.launch.py  # Camera

cd sensor_ws && sr install/
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py  # IMU

# CHECK: Change path in gnss_rtk_exec.sh
cd sensor_ws && sr install/
bash /home/renam/sensors_ws/src/Ericsson-GNSS-RTK-ROS-package/src/gnss_rtk_exec.sh  # GPS

# CHECK: Velocity values
# CHECK: Velocity command topic
ros2 run agrihusky controller.py  # GPS Controller

---

# Simulated to Real tf

/home/renam/colcon_ws/src/python/pcl_pub2_no_tf.py
/home/renam/colcon_ws/src/python/imu_pub2.py

---

/lidar_points
/oak/rgb/rect
/imu/data/repub
/ublox_client