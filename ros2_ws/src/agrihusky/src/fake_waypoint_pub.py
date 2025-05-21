#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random  # For generating random waypoints (you can customize this)

class FakeWaypointPublisher(Node):
    def __init__(self):
        super().__init__('fake_waypoint_pub')

        # Create the publisher
        self.waypoint_pub = self.create_publisher(Point, '/unitree_planner/waypoint', 10)

        # Set the timer to periodically publish waypoints
        self.timer = self.create_timer(15.0, self.publish_waypoint)  # Publish every x seconds

    def publish_waypoint(self):
        # Create a Point message
        waypoint = Point()

        # For demonstration, generate random x, y, z coordinates (you can set these manually)
        waypoint.x = random.uniform(-10.0, 10.0)
        waypoint.y = random.uniform(-10.0, 10.0)
        waypoint.z = 0.0  # Assuming we're working in a 2D plane (z = 0)

        # Log and publish the waypoint
        self.get_logger().info(f'Publishing waypoint: ({waypoint.x}, {waypoint.y}, {waypoint.z})')
        self.waypoint_pub.publish(waypoint)

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin to handle callbacks
    waypoint_publisher = FakeWaypointPublisher()

    try:
        rclpy.spin(waypoint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
