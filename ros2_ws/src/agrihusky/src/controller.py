#!/usr/bin/env python3

import rclpy, serial, math as m, numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from agrihusky.srv import ProbeRequest
from ublox_msg.msg import UbxNavPvt
from geometry_msgs.msg import Twist, Point, Quaternion, PointStamped
# from tf_transformations import euler_from_quaternion

###

inSimulation = True
robotName = 'husky'

# https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
# Length in km of 1° of latitude = always 111.32 km
# Length in km of 1° of longitude = 40075 km * cos( latitude ) / 360
lat_degree_to_meter = 111320
lon_degree_to_meter = lambda latitude : 40075000 * m.cos(m.radians(latitude)) / 360  

###

def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    :param quat: a list or tuple with 4 elements (x, y, z, w)
    :return: a tuple (roll, pitch, yaw)
    """
    x, y, z, w = quat

    # Roll (rotation around X-axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = m.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around Y-axis)
    sinp = 2 * (w * y - z * x)
    pitch = m.asin(sinp) if abs(sinp) <= 1 else m.copysign(m.pi / 2, sinp)

    # Yaw (rotation around Z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = m.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

###

class HuskyController(Node):
    
    def __init__(self, robotName, rate=10, maxSpeed=0.5, maxAngVel=0.5, goalDistance=0.2, slowDownDistance=0.7, turnInPlaceAngle=0.5):
        super().__init__('husky_controller')

        self.robotName = robotName
        self.rate = rate
        self.path = []

        self.x = 0.0
        self.y = 0.0

        self.x_offset = 0.0  # Initial GPS position in meters
        self.y_offset = 0.0

        self.ubxNavPvt = None
        self.longitude = None
        self.latitude  = None

        self.newWaypointData = False
        self.newPositionData = False
        self.newOrientationData = False
        
        self.maxSpeed = maxSpeed
        self.maxAngVel = maxAngVel

        self.blockingProbeRequest = False
        self.request = ProbeRequest.Request()

        self.goalDistance = goalDistance
        self.slowDownDistance = slowDownDistance
        self.turnInPlaceAngle = turnInPlaceAngle

        ###

        self.cmdVelPub = self.create_publisher(Twist, '/husky_planner/cmd_vel', self.rate)                              # Velocity Command 

        self.client = self.create_client(ProbeRequest, 'probe_request')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Probe service not available, waiting...')

        self.gpsTopic = '/husky/gps' if inSimulation else '/ublox_client'
        self.gpsMessage = PointStamped if inSimulation else UbxNavPvt
        self.poisitionSub = self.create_subscription(self.gpsMessage, self.gpsTopic, self.gpsCallback, self.rate)       # GPS Callback

        self.orientationSub = self.create_subscription(Odometry, '/husky/odom', self.odometryCallback, self.rate) if inSimulation else \
                              self.create_subscription(Quaternion, '/imu/data', self.imuCallback, self.rate)            # Odomemtry/IMU Callback
        
        self.waypointSub = self.create_subscription(Point, '/husky_planner/waypoint', self.waypointCallback, self.rate) # Waypoint Callback

        self.timer = self.create_timer(1.0 / self.rate, self.control)  # Controller loop
        
        self.get_logger().info(f"Controller initialized for {self.robotName} at {self.rate}Hz")


    def gpsCallback(self, msg):  # https://github.com/KumarRobotics/ublox/blob/ros2/ublox_msgs/msg/NavPVT.msg
        if self.blockingProbeRequest:  return
        
        self.ubxNavPvt = msg

        if self.latitude is None and self.longitude is None:
            self.latitude  = msg.point.y if inSimulation else msg.lat
            self.longitude = msg.point.x if inSimulation else msg.lon

            self.y_offset = self.latitude  * lat_degree_to_meter
            self.x_offset = self.longitude * lon_degree_to_meter(self.latitude)

        latitude  = msg.point.y if inSimulation else msg.lat
        longitude = msg.point.x if inSimulation else msg.lon

        # Calculate delta in meters (Global to local coordinates)
        lat_meters = (latitude  - self.latitude)  * lat_degree_to_meter
        lon_meters = (longitude - self.longitude) * lon_degree_to_meter(latitude  - self.latitude)

        self.y = self.y + lat_meters  # Positive latitude  is North, negative is South
        self.x = self.x + lon_meters  # Positive longitude is East,  negative is West

        self.latitude = latitude
        self.longitude = longitude

        self.newPositionData = True


    def odometryCallback(self, msg):  # https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        q = msg.pose.pose.orientation
        (_, _, self.angle) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.linVel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
        self.angVel = msg.twist.twist.angular.z

        self.newOrientationData = True


    def imuCallback(self, msg):  # https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
        q = msg.orientation
        (_, _, self.angle) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        dt = 1/self.rate # Every 10Hz
        self.linVel = np.array([msg.linear_acceleration.x * dt, msg.linear_acceleration.y * dt])
        self.angVel = msg.angular_velocity.z

        self.newOrientationData = True


    def waypointCallback(self, msg):  # https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html
        self.get_logger().info(f"Next waypoint: ({msg.x:.2f}, {msg.y:.2f})")
        self.segmentStart = np.array([self.x, self.y])
        
        # Transform waypoint to local frame (based on initial GPS position)
        lat_meters = msg.y * lat_degree_to_meter
        lon_meters = msg.x * lon_degree_to_meter(msg.y)       
        self.segmentEnd = np.array([lon_meters - self.x_offset, lat_meters - self.y_offset])

        self.newWaypointData = True


    def probeRequest(self):
        self.request.start = b'\x00'
        response = self.client.call(self.request)

        if   response.status == b'\xFF':  
             result = response.data;  message = "Successful Reading"
        elif response.status == b'\x0F':  
             result = None;           message = "ERROR: Unsuccesful Reading (NULL)"
        elif response.status == b'\xEF':  
             result = None;           message = "ERROR: No initial pot contact. Retracting..."
        elif response.status == b'\xDF':  
             result = None;           message = "ERROR: Insertion stalled. Retracting..."

        self.get_logger().info(f"Probe Status: ({response.status}, {message})")
        return result


    def control(self):
        if self.newWaypointData and self.newPositionData and self.newOrientationData:
            self.newPositionData = False;  self.newOrientationData = False

            self.pose = np.array([[m.cos(self.angle), -m.sin(self.angle), self.x],
                                  [m.sin(self.angle),  m.cos(self.angle), self.y],
                                  [0, 0, 1]])

            self.get_logger().info(f"Position: ({self.x:.2f}, {self.y:.2f}) | Angle: {self.angle:.2f}")
            # self.get_logger().info(f"Speed: {self.linVel} | Yawrate: {self.angVel:.2f}")

            [xRefTrans, yRefTrans, _] = np.linalg.solve(self.pose, np.append(self.segmentEnd, 1))
            self.get_logger().info(f"Transformed waypoint: ({xRefTrans:.2f}, {yRefTrans:.2f})")
            
            distance = np.linalg.norm([xRefTrans, yRefTrans])
            cmdVel = Twist()

            if distance < self.goalDistance:
                self.newWaypointData = False
                self.get_logger().info(f"Waypoint reached! ({self.segmentEnd[0]:.2f}, {self.segmentEnd[0]:.2f})")

                if not inSimulation:
                    self.blockingProbeRequest = True  # Block GPS readings to impede changes in the GPS message
                    result = self.probeRequest()
                    if result != None:  self.get_logger().info(f"New data reading ({result})")  ## TODO Save data to file
                    else:               pass                                                    ## TODO Call service on waypoint_pub to create new waypoint one meter forward   
                    self.blockingProbeRequest = False # Reactivate GPS readings
            else:
                refAngle = m.atan2(yRefTrans, xRefTrans)
                if abs(refAngle) > self.turnInPlaceAngle:
                    cmdVel.angular.z = np.sign(refAngle) * self.maxAngVel
                else:       
                    cmdVel.linear.x = min(self.maxSpeed, self.maxSpeed * distance / self.slowDownDistance)
                    cmdVel.angular.z = self.maxAngVel * (abs(refAngle) / self.turnInPlaceAngle) * np.sign(refAngle)
                                        
            self.cmdVelPub.publish(cmdVel)

        if not rclpy.ok():
            self.get_logger().info('ROS is shutting down!')
            self.destroy_node()

###

def main(args=None):
    try:
        rclpy.init(args=args)
        huskyController = HuskyController(robotName, rate=25, maxSpeed=0.7, maxAngVel=1.2, goalDistance=0.1, slowDownDistance=0.5, turnInPlaceAngle=0.6)    
        rclpy.spin(huskyController)
    except KeyboardInterrupt:
        pass
    finally:
        huskyController.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()