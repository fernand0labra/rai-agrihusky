#!/usr/bin/env python3

import rclpy, random, math as m

from rclpy.node import Node
from geometry_msgs.msg import Point
from ublox_msg.msg import UbxNavPvt
from geometry_msgs.msg import PointStamped
from agrihusky.srv import WaypointRequest
from ament_index_python.packages import get_package_share_directory

###

lon_degree_to_meter = lambda latitude : 40075000 * m.cos(m.radians(latitude)) / 360  

meter_to_lat_degree = lambda meters: meters / 111320
meter_to_lon_degree = lambda meters, latitude: meters / (lon_degree_to_meter(latitude))


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_pub')

        self.path = []
        self.withData = False
        self.withInput = False
        self.inSimulation = False

        if self.withData and not self.withInput:
            with open(get_package_share_directory('agrihusky') + '/data/new-waypoints-gps.txt', 'r') as file:
                for ldx, line in enumerate(file.readlines()):
                    self.path.append(line.split('\t'))  # [Latitude, Longitude]

        if not self.withData and not self.withInput:
            self.gpsTopic = '/husky/gps' if self.inSimulation else '/ublox_client'
            self.gpsMessage = PointStamped if self.inSimulation else UbxNavPvt
            self.poisitionSub = self.create_subscription(self.gpsMessage, self.gpsTopic, self.gpsCallback, 10)

        self.waypoint_pub = self.create_publisher(Point, '/husky_planner/waypoint', 10)
        self.srv = self.create_service(WaypointRequest, 'waypoint_request', self.waypointCallback)
        
        self.origin = None
        self.timer = self.create_timer(2, self.publish_waypoint)  # TODO Communication regarding waypoint arrival between controller and publisher


    def gpsCallback(self, msg):
        if self.origin is None:
            self.origin = [msg.point.y if self.inSimulation else msg.lat, 
                           msg.point.x if self.inSimulation else msg.lon]
            meters = 3

            # Vertex (1, 0) * meters
            vertex_1_0 = self.origin.copy()
            vertex_1_0[1] = vertex_1_0[1] + meter_to_lon_degree(meters, self.origin[0])
            self.path.append(vertex_1_0)
        
            # Vertex (1, 1) * meters
            vertex_1_1 = vertex_1_0.copy()
            vertex_1_1[0] = vertex_1_1[0] + meter_to_lat_degree(meters)
            self.path.append(vertex_1_1)

            # Vertex (0, 1) * meters
            vertex_0_1 = vertex_1_1.copy()
            vertex_0_1[1] = vertex_0_1[1] - meter_to_lon_degree(meters, self.origin[0])
            self.path.append(vertex_0_1)

            # Vertex (0, 0) :: Origin
            vertex_0_0 = vertex_0_1.copy()
            vertex_0_0[0] = vertex_0_0[0] - meter_to_lat_degree(meters)
            self.path.append(vertex_0_0)

        else: return


    def waypointCallback(self, request, response):
        self.get_logger().info(f'Adjusting waypoint: ({request.waypoint.x}, {request.waypoint.y}, {request.waypoint.z})')
        self.waypoint_pub.publish(request.waypoint)

        response.status = b'\xFF'
        return response


    def publish_waypoint(self):
        waypoint = Point()

        if self.withData or not self.withInput:
            if len(self.path) > 0:  coordinates = self.path.pop(0)
            else:                   exit(0)

        else:
            coordinates = [0, 0]
            try:
                coordinates[0] = float(input("Indicate objective latitude: "))
                coordinates[1] = float(input("Indicate objective longitude: "))
    
            except Exception as exception:
                self.get_logger().info(f'Error from input: {exception}')
                return

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
