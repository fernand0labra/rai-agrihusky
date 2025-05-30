#!/usr/bin/env python3

import rclpy, math as m, numpy as np

from rclpy.node import Node
from datetime import datetime
from nav_msgs.msg import Odometry
from agrihusky.srv import ProbeRequest, WaypointRequest
from ublox_msg.msg import UbxNavPvt
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point, PointStamped, Quaternion
# from tf_transformations import euler_from_quaternion

###

# https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
# Length in km of 1° of latitude = always 111.32 km
# Length in km of 1° of longitude = 40075 km * cos( latitude ) / 360
lat_degree_to_meter = 111320
lon_degree_to_meter = lambda latitude : 40075000 * m.cos(m.radians(latitude)) / 360  

meter_to_lat_degree = lambda meters: meters / 111320
meter_to_lon_degree = lambda meters, latitude: meters / (lon_degree_to_meter(latitude))

###

def euler_to_quaternion(angle):
    q = Quaternion()  # Assuming the angle is around the Z-axis (yaw), this is the simplest case.
    q.x = 0.0
    q.y = 0.0
    q.z = m.sin(angle / 2.0)
    q.w = m.cos(angle / 2.0)
    return q

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

def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions.
    q1 and q2 are in [x, y, z, w] format.
    Returns [x, y, z, w]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return [x, y, z, w]

###

class HuskyController(Node):
    
    def __init__(self, rate=10, maxSpeed=0.5, maxAngVel=0.5, goalDistance=0.2, slowDownDistance=0.7, turnInPlaceAngle=0.5):
        super().__init__('husky_controller')

        self.robotName = 'Husky'
        self.rate = rate
        self.path = []

        self.x = 0.0
        self.y = 0.0

        self.angle = 0.0

        self.x_offset = 0.0  # Initial GPS position in meters
        self.y_offset = 0.0

        self.ubxNavPvt = None

        self.longitude = None
        self.latitude  = None
        self.latitudeRef = None

        self.newWaypointData = False
        self.newPositionData = False
        self.newOrientationData = False

        self.withProbe = False     # Activate probe logic
        self.inSimulation = False  # Activate fake sensors
        
        self.maxSpeed = maxSpeed
        self.maxAngVel = maxAngVel

        self.probeMsgRequest = ProbeRequest.Request()
        self.waypointMsgRequest = WaypointRequest.Request()

        self.goalDistance = goalDistance
        self.slowDownDistance = slowDownDistance
        self.turnInPlaceAngle = turnInPlaceAngle

        ###

        self.gpsTopic = '/husky_planner/cmd_vel' if self.inSimulation else '/cmd_vel'
        self.cmdVelPub = self.create_publisher(Twist, self.gpsTopic, self.rate)                               # Velocity Command 
        self.odometryPub = self.create_publisher(Odometry, '/husky_planner/odom', self.rate)                  # Local Transformed Odometry

        if not self.inSimulation and self.withProbe:  # Services for probe handling
            self.probeData = open(f"probe-data-{datetime.now():%Y%m%d-%H%M}.txt")
            self.probeClient = self.create_client(ProbeRequest, 'probe_request')
            while not self.probeClient.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Probe service not available, waiting...')

            self.waypointClient = self.create_client(WaypointRequest, 'waypoint_request')
            while not self.waypointClient.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waypoint service not available, waiting...')

        self.gpsTopic = '/husky/gps' if self.inSimulation else '/ublox_client'
        self.gpsMessage = PointStamped if self.inSimulation else UbxNavPvt
        self.positionSub = self.create_subscription(self.gpsMessage, self.gpsTopic, self.gpsCallback, self.rate)             # GPS Callback

        self.orientationSub = self.create_subscription(Odometry, '/husky/odom',     self.odometryCallback, self.rate) if self.inSimulation else \
                              self.create_subscription(Imu,      '/imu/data/repub', self.imuCallback,      self.rate)        # Odomemtry/IMU Callback
        
        self.waypointSub = self.create_subscription(Point, '/husky_planner/waypoint', self.waypointCallback, self.rate)      # Waypoint Callback

        self.timer = self.create_timer(1.0 / self.rate, self.control)  # Controller loop
        
        self.get_logger().info(f"Controller initialized for {self.robotName} at {self.rate}Hz")


    def gpsCallback(self, msg):  # https://github.com/KumarRobotics/ublox/blob/ros2/ublox_msgs/msg/NavPVT.msg
        self.ubxNavPvt = msg

        if self.latitude is None and self.longitude is None:  # When first latitude/longitude obtained
            # Update global reference
            self.latitude    = msg.point.y if self.inSimulation else msg.lat
            self.longitude   = msg.point.x if self.inSimulation else msg.lon

            self.latitudeRef = msg.point.y if self.inSimulation else msg.lat

            # Compute local offset in meters
            self.y_offset = self.latitude  * lat_degree_to_meter
            self.x_offset = self.longitude * lon_degree_to_meter(self.latitudeRef)

            self.get_logger().info(f"Husky at coordinates [{self.latitude}, {self.longitude}]")

        latitude  = msg.point.y if self.inSimulation else msg.lat
        longitude = msg.point.x if self.inSimulation else msg.lon

        # Calculate delta in meters (Global to local coordinates)
        lat_meters = (latitude  - self.latitude)  * lat_degree_to_meter
        lon_meters = (longitude - self.longitude) * lon_degree_to_meter(self.latitudeRef)

        # Update local reference from latitude/longitude delta
        self.y = self.y + lat_meters # Positive latitude  is North, negative is South
        self.x = self.x + lon_meters # Positive longitude is East,  negative is West

        # Update global reference
        self.latitude = latitude
        self.longitude = longitude

        # Publish local position
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(self.angle)  # Convert yaw to Quaternion

        self.odometryPub.publish(odom)

        self.newPositionData = True


    def odometryCallback(self, msg):  # https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        q = msg.pose.pose.orientation
        (_, _, self.angle) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.newOrientationData = True


    def imuCallback(self, msg):  # https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
        q = msg.orientation
        rot_correction = euler_to_quaternion(0, 0, -1.5708)    # Define the frame rotation: -90 degrees (clockwise) around Z
        rotated_quat = quaternion_multiply(rot_correction, q)  # Apply the rotation correction: q_new = q_rot * q_imu
        (_, _, self.angle) = euler_from_quaternion(rotated_quat)

        self.newOrientationData = True


    def waypointCallback(self, msg):  # https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html
        self.get_logger().info(f"Next waypoint: ({msg.x:.2f}, {msg.y:.2f})")
        self.segmentStart = np.array([self.x, self.y])
        
        # Transform waypoint to meters
        lat_meters = msg.y * lat_degree_to_meter
        lon_meters = msg.x * lon_degree_to_meter(self.latitudeRef)     

        # Transform waypoint to local frame as a position from starting point (offset)
        self.segmentEnd = np.array([lon_meters - self.x_offset, lat_meters - self.y_offset])

        self.newWaypointData = True


    def probeRequest(self):
        self.probeMsgRequest.start = b'\x00'
        response = self.probeClient.call(self.probeMsgRequest)

        if   response.status == b'\xFF':  
             result = response.data;  message = "Successful Reading"
        elif response.status == b'\x0F':  
             result = None;           message = "ERROR: Unsuccesful Reading (NULL)"
        elif response.status == b'\xEF':  
             result = None;           message = "ERROR: No initial pot contact. Retracting..."
        elif response.status == b'\xDF':  
             result = None;           message = "ERROR: Insertion stalled. Retracting..."

        self.get_logger().info(f"Probe Service Status: ({response.status}, {message})")
        return result


    def waypointRequest(self):
        # Request new waypoint one meter forward in latitude
        latitude = meter_to_lat_degree(self.y + 0.1 * np.sign(self.y))
        longitude = meter_to_lon_degree(self.x, self.latitudeRef)

        # Build request message
        self.waypointMsgRequest.waypoint.z = 0.0
        self.waypointMsgRequest.waypoint.y = latitude
        self.waypointMsgRequest.waypoint.x = longitude

        # Notify waypoint publisher
        response = self.waypointClient.call(self.waypointMsgRequest)
        self.get_logger().info(f"Waypoint Service Status: ({response.status})")
    

    def control(self):
        if self.newWaypointData and self.newPositionData and self.newOrientationData:
            self.newPositionData = False;  self.newOrientationData = False

            self.pose = np.array([[m.cos(self.angle), -m.sin(self.angle), self.x],
                                  [m.sin(self.angle),  m.cos(self.angle), self.y],
                                  [0, 0, 1]])  # Transformation matrix

            self.get_logger().info(f"Position: ({self.x:.2f}, {self.y:.2f}) | Angle: {self.angle:.2f}")

            [xRefTrans, yRefTrans, _] = np.linalg.solve(self.pose, np.append(self.segmentEnd, 1))  # Coordinate frame transform
            self.get_logger().info(f"Transformed waypoint: ({xRefTrans:.2f}, {yRefTrans:.2f})")
            
            distance = np.linalg.norm([xRefTrans, yRefTrans])  # Distance from origin
            cmdVel = Twist()

            if distance < self.goalDistance:  # When distance is close enough to the goal
                self.newWaypointData = False
                self.get_logger().info(f"Waypoint reached! ({self.segmentEnd[0]:.2f}, {self.segmentEnd[1]:.2f})")
                self.get_logger().info(f"Husky at coordinates [{self.latitude}, {self.longitude}]")

                if not self.inSimulation and self.withProbe:  # Probe logic when indicated
                    result = self.probeRequest()
                    if result != None:  self.probeData.write(result)  # Write data if available
                    else:               self.waypointRequest()        # Otherwise adjust current waypoint

            else:                             
                refAngle = m.atan2(yRefTrans, xRefTrans)   # Compute angle of the robot
                if abs(refAngle) > self.turnInPlaceAngle:  # When robot requires turning in place
                    cmdVel.angular.z = np.sign(refAngle) * self.maxAngVel  # Rotate at maxAngVel in the direction of refAngle
                else:       
                    # distance / self.slowDownDistance -> Ratio [0.0, 1.0]
                    # Smaller linear velocities when distance below slowDownDistance otherwise maxSpeed
                    cmdVel.linear.x = min(self.maxSpeed, self.maxSpeed * (distance / self.slowDownDistance))

                    # abs(refAngle) / self.turnInPlaceAngle -> Ratio [0.0, 1.0]
                    # Smaller angular velocities when refAngle below turnInPlaceAngle otherwise maxAngleVel
                    cmdVel.angular.z = self.maxAngVel * (abs(refAngle) / self.turnInPlaceAngle) * np.sign(refAngle)
                                        
            self.cmdVelPub.publish(cmdVel)

        if not rclpy.ok():
            self.get_logger().info('ROS is shutting down!')
            self.destroy_node()

###

def main(args=None):
    try:
        rclpy.init(args=args)
        huskyController = HuskyController(rate=25, maxSpeed=0.7, maxAngVel=0.6, goalDistance=0.01, slowDownDistance=0.5, turnInPlaceAngle=0.2)    
        rclpy.spin(huskyController)
    except KeyboardInterrupt:
        pass
    finally:
        huskyController.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()