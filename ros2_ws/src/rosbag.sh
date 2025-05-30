# Camera ********************************************************************************
ros2 launch depthai_ros_driver pointcloud.launch.py  

# IMU ***********************************************************************************
cd sensor_ws && sr install/
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py  
python3 /home/renam/colcon_ws/src/python/imu_pub2.py

# GPS ***********************************************************************************
cd agri_ws/rai-agrihusky/ros2_ws && sr install/
bash src/gnss_rtk_exec.sh  

# GPS Controller ************************************************************************
ros2 run agrihusky controller.py  

# Lidar *********************************************************************************
ros2 launch innovusion ivu_pc2.py 
python3 /home/renam/colcon_ws/src/python/pcl_pub2_no_tf.py

---

ros2 bag record /tf /tf_static /lidar_points /oak/rgb/raw /imu/data/repub /ublox_client

ros2 bag record /ublox_client /imu/data/repub /husky_planner/odom /husky_planner/waypoint

---

ros2 bag play --rate 1. --start-offset 280 rosbag2_2025_05_30-11_47_47_0.db3