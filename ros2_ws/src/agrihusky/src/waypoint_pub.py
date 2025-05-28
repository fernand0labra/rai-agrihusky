#!/usr/bin/env python3

import rclpy, random

from rclpy.node import Node
from geometry_msgs.msg import Point
from agrihusky.srv import WaypointRequest
from ament_index_python.packages import get_package_share_directory

###

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_pub')

        self.withData = False  # Read every 60th GPS coordinate as waypoint
        self.path = []
        self.filename = '/data/gps-2025-05-20-13-37-14.txt' if self.withData else '/data/new-waypoints.txt'
        with open(get_package_share_directory('agrihusky') + self.filename, 'r') as file:
            for ldx, line in enumerate(file.readlines()):
                # if ldx%2 == 0:  
                self.path.append(line.split('\t'))  # [Latitude, Longitude]

        self.waypoint_pub = self.create_publisher(Point, '/husky_planner/waypoint', 10)
        self.srv = self.create_service(WaypointRequest, 'waypoint_request', self.waypointCallback)

        self.timer = self.create_timer(2, self.publish_waypoint)  # TODO Communication regarding waypoint arrival between controller and publisher


    def waypointCallback(self, request, response):
        self.get_logger().info(f'Adjusting waypoint: ({request.waypoint.x}, {request.waypoint.y}, {request.waypoint.z})')
        self.waypoint_pub.publish(request.waypoint)

        response.status = b'\xFF'
        return response


    def publish_waypoint(self):
        waypoint = Point()

        if len(self.path) > 0:  coordinates = self.path.pop(0)
        else:                   return

        waypoint.y = float(coordinates[0])
        waypoint.x = float(coordinates[1])
        waypoint.z = 0.0

        self.get_logger().info(f'Publishing waypoint: ({waypoint.x}, {waypoint.y}, {waypoint.z})')
        self.waypoint_pub.publish(waypoint)

###

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()

    try:
        # waypoint_publisher.publish_waypoint()
        rclpy.spin(waypoint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
