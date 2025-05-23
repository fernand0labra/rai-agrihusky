#!/usr/bin/env python3

import rclpy, random

from rclpy.node import Node
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory

###

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_pub')

        self.withData = True
        self.path = []
        if self.withData:  # Read every 60th GPS coordinate as waypoint
            with open(get_package_share_directory('agrihusky') + '/data/gps-2025-05-20-13-37-14.txt', 'r') as file:
                for ldx, line in enumerate(file.readlines()):
                    if ldx%2 == 0:  
                        self.path.append(line.split('\t'))  # [Latitude, Longitude]

        self.waypoint_pub = self.create_publisher(Point, '/husky_planner/waypoint', 10)

        self.timer = self.create_timer(2, self.publish_waypoint)


    def publish_waypoint(self):
        waypoint = Point()

        if self.withData:  # In global frame
            if len(self.path) > 0:  coordinates = self.path.pop(0)
            else:                   return
            
            waypoint.x = float(coordinates[1])
            waypoint.y = float(coordinates[0])
            waypoint.z = 0.0

        else:  # In local frame
            waypoint.x = random.uniform(-10.0, 10.0)
            waypoint.y = random.uniform(-10.0, 10.0)
            waypoint.z = 0.0

        self.get_logger().info(f'Publishing waypoint: ({waypoint.x}, {waypoint.y}, {waypoint.z})')
        self.waypoint_pub.publish(waypoint)

###

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()

    try:
        rclpy.spin(waypoint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
