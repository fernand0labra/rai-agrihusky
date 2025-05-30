#!/usr/bin/env python3

import rclpy, math as m
from rclpy.node import Node
from ublox_msg.msg import UbxNavPvt
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time
from datetime import datetime, timezone

###

lat_degree_to_meter = 111320
lon_degree_to_meter = lambda latitude : 40075000 * m.cos(m.radians(latitude)) / 360  

###

class NavPVTProcessor(Node):
    def __init__(self):
        super().__init__('navpvt_processor')
        self.subscription = self.create_subscription(
            UbxNavPvt,
            '/ublox_client',
            self.navpvt_callback,
            10
        )
        self.publisher = self.create_publisher(PointStamped, 'husky_planner/gps', 10)
        
        self.x = 0.0
        self.y = 0.0

        self.latitude = None
        self.longitude = None

    def navpvt_callback(self, msg: UbxNavPvt):
        # Convert UTC fields to ROS time
        try:
            utc_time = datetime(
                year=msg.year,
                month=msg.month,
                day=msg.day,
                hour=msg.hour,
                minute=msg.min,
                second=msg.sec,
                microsecond=int(msg.nano / 1000),  # ns to µs
                tzinfo=timezone.utc
            )
        except Exception as e:
            self.get_logger().error(f"Failed to create UTC timestamp: {e}")
            return

        ros_time = Time()
        ros_time.sec = int(utc_time.timestamp())
        ros_time.nanosec = int((utc_time.timestamp() % 1) * 1e9)

        # Create and populate PointStamped
        point = PointStamped()
        point.header.stamp = ros_time
        point.header.frame_id = "base_link"

        if self.latitude is None and self.longitude is None:  # When first latitude/longitude obtained
            # Update global reference
            self.latitude, self.longitude = msg.lat, msg.lon

            # Compute local offset in meters
            self.y_offset = self.latitude  * lat_degree_to_meter
            self.x_offset = self.longitude * lon_degree_to_meter(self.latitude)

        latitude, longitude = msg.lat, msg.lon

        # Calculate delta in meters (Global to local coordinates)
        lat_meters = (latitude  - self.latitude)  * lat_degree_to_meter
        lon_meters = (longitude - self.longitude) * lon_degree_to_meter(self.latitude)

        # Update local reference from latitude/longitude delta
        self.y = self.y - lat_meters # Positive latitude  is North, negative is South
        self.x = self.x - lon_meters # Positive longitude is East,  negative is West

        # Update global reference
        self.latitude = latitude
        self.longitude = longitude

        point.point.x = self.x  # Convert from int to float degrees
        point.point.y = self.y
        point.point.z = 0.0  # Optional: could use height

        self.publisher.publish(point)
        self.get_logger().info(f"Published point: lat={point.point.x}, lon={point.point.y}")


def main(args=None):
    rclpy.init(args=args)
    node = NavPVTProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()