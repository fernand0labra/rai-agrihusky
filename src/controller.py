#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math as m
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path, Odometry
from gazebo_msgs.msg import ModelStates
# from tf_transformations import euler_from_quaternion

###

inSimulation = False
robotName = 'husky'

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
    
    def __init__(self, robotName, rate=10, maxSpeed=0.5, maxAngVel=0.5, lookAheadDistance=0.5, goalDistance=0.2, slowDownDistance=0.7, 
                 turnInPlaceAngle=0.5):
        super().__init__('husky_controller')

        self.robotName = robotName
        self.rate = rate
        self.active = False
        self.newOdometryData = False
        self.goalDistance = goalDistance
        self.maxSpeed = maxSpeed
        self.maxAngVel = maxAngVel
        self.slowDownDistance = slowDownDistance
        self.turnInPlaceAngle = turnInPlaceAngle

        self.cmdVelPub = self.create_publisher(Twist, '/unitree/cmd_vel', 10)

        self.odometrySub = self.create_subscription(Odometry, '/pixy/vicon/' + robotName + '/' + robotName + '/odom', self.odometryCallback, 10)
        self.waypointSub = self.create_subscription(Point, '/unitree_planner/waypoint', self.waypointCallback, 10)
        self.pathSub = self.create_subscription(Path, '/unitree_planner/path', self.pathCallback, 10)

        self.timer = self.create_timer(1.0 / self.rate, self.control)


    def odometryCallback(self, msg):  # TODO Update position from GPS
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y       
        q = msg.pose.pose.orientation
        (_, _, self.angle) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.linVel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
        self.angVel = msg.twist.twist.angular.z

        self.pose = np.array([[m.cos(self.angle), -m.sin(self.angle), self.x],
                            [m.sin(self.angle), m.cos(self.angle), self.y],
                            [0, 0, 1]])

        self.newOdometryData = True


    def waypointCallback(self, msg):
        print(msg)
        self.segmentStart = np.array([self.x, self.y])
        self.segmentEnd = np.array([msg.x, msg.y])
        self.segmentIndex = 0
        self.fracIndex = 0
        self.numPathPoints = 2
        self.active = True


    def pathCallback(self, msg):  # TODO Change callback to individual waypoints from path list
        return
        self.numPathPoints = len(msg.poses)
        self.pathXY = np.zeros((self.numPathPoints, 2))
        self.pathHeading = np.zeros(self.numPathPoints)

        for i in range(self.numPathPoints):
            pos = msg.poses[i].pose.position
            self.pathXY[i, 0] = pos.x
            self.pathXY[i, 1] = pos.y
            q = msg.poses[i].pose.orientation
            (_, _, self.pathHeading[i]) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.segmentStart = self.pathXY[0]
        self.segmentEnd = self.pathXY[1]
        self.segmentIndex = 0
        self.fracIndex = 0
        # self.lookAheadPoint = self.segmentStart
        self.active = True    


    def control(self):
        if self.active and self.newOdometryData:
            self.newOdometryData = False
            print(f"Position: ({self.x:.2f}, {self.y:.2f}) | Angle: {self.angle:.2f}")
            print(f"Speed: {self.linVel} | Yawrate: {self.angVel:.2f}")

            [xRefTrans, yRefTrans, _] = np.linalg.solve(self.pose, np.append(self.segmentEnd, 1))
            print(f"Transformed waypoint: ({xRefTrans:.2f}, {yRefTrans:.2f})")
            distance = np.linalg.norm([xRefTrans, yRefTrans])
            cmdVel = Twist()
            if self.segmentIndex == self.numPathPoints - 2 and distance < self.goalDistance:
                self.active = False
            else:
                refAngle = m.atan2(yRefTrans, xRefTrans)
                if abs(refAngle) > self.turnInPlaceAngle:
                    cmdVel.angular.z = np.sign(refAngle) * self.maxAngVel
                else:       
                    cmdVel.linear.x = min(self.maxSpeed, self.maxSpeed * distance / self.slowDownDistance)
                    curvature = 2 * yRefTrans / distance**2
                    cmdVel.angular.z = min(curvature * cmdVel.linear.x, self.maxAngVel)
                                        
            self.cmdVelPub.publish(cmdVel)

        if not rclpy.ok():
            self.get_logger().info('ROS is shutting down!')
            self.destroy_node()

###

def main(args=None):
    try:
        rclpy.init(args=args)
        huskyController = HuskyController(robotName, rate=25, maxSpeed=0.7, maxAngVel=1.2, goalDistance=0.1,
                                                    slowDownDistance=0.5, turnInPlaceAngle=0.6, lookAheadDistance=0.6)    
        rclpy.spin(huskyController)
    except KeyboardInterrupt:
        pass
    finally:
        huskyController.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()