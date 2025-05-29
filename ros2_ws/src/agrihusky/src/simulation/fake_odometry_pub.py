#!/usr/bin/env python3

import math, rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion

###

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odometry_pub')

        self.odom_pub = self.create_publisher(Odometry, '/husky/odom', 10)                                          # Publisher for odometry
        self.cmd_vel_sub = self.create_subscription(Twist, '/husky_planner/cmd_vel', self.cmd_vel_callback, 10)     # Subscriber for cmd_vel (velocity commands)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_odometry)  # Timer to publish odometry at 10 Hz

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0                # Orientation (yaw)
        self.linear_velocity = 0.0      # m/s
        self.angular_velocity = 0.0     # rad/s

        self.last_time = self.get_clock().now()


    def cmd_vel_callback(self, msg):
        """Callback to update velocities based on cmd_vel message."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z


    def publish_odometry(self):
        """Publish the robot's odometry based on its current state."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Time step in seconds
        self.last_time = current_time

        # Simulate movement using the velocities from cmd_vel
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        q = self.quaternion_from_euler(0, 0, self.theta)

        # Create and publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])  # Convert yaw to Quaternion
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        self.odom_pub.publish(odom)


    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles (roll, pitch, yaw) to quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return (qx, qy, qz, qw)

###

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
