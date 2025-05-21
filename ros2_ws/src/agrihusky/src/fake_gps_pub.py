#!/usr/bin/env python3

import math, rclpy

from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, PointStamped

###

class FakeGPSPublisher(Node):
    def __init__(self):
        super().__init__('fake_gps_pub')

        self.gps_pub = self.create_publisher(PointStamped, '/husky/gps', 10)                                        # Publisher for simulated GPS coordinates
        self.cmd_vel_sub = self.create_subscription(Twist, '/husky_planner/cmd_vel', self.cmd_vel_callback, 10)     # Subscriber for velocity commands
        
        timer_period = 0.1  # Seconds
        self.timer = self.create_timer(timer_period, self.publish_gps)  # Timer to publish at 10 Hz

        self.heading = 0.0                  # Radians (yaw) :: True North is 0, East is pi/2, South is pi, West is -pi/2
        self.latitude  = 65.6166129
        self.longitude = 22.1373941
        self.linear_velocity = 0.0          # m/s
        self.angular_velocity = 0.0         # rad/s

        self.last_time = self.get_clock().now()


    def cmd_vel_callback(self, msg):
        """Update velocities from Twist message."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z


    def publish_gps(self):
        """Simulate and publish GPS based on velocity and orientation."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        R = 6378137.0  # Earth's radius in meters
        # Δlat in radians = distance / Earth's radius
        delta_lat = (self.linear_velocity * dt * math.sin(self.heading)) / R
        # Δlon in radians = distance / (Earth's radius * cos(latitude))
        delta_lon = (self.linear_velocity * dt * math.cos(self.heading)) / (R * math.cos(math.radians(self.latitude)))

        # Update global position in degrees
        self.latitude  += math.degrees(delta_lat)
        self.longitude += math.degrees(delta_lon)
        self.heading   += self.angular_velocity * dt

        # Create and publish PointStamped as fake GPS
        gps_msg = PointStamped()
        gps_msg.header = Header()
        gps_msg.header.stamp = current_time.to_msg()
        gps_msg.header.frame_id = 'gps'

        gps_msg.point.x = self.longitude
        gps_msg.point.y = self.latitude
        gps_msg.point.z = 0.0 

        self.gps_pub.publish(gps_msg)

###

def main(args=None):
    rclpy.init(args=args)
    node = FakeGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
