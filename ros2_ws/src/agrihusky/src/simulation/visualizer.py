#!/usr/bin/env python3

import math, rclpy, pygame, numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

###

lat_degree_to_meter = 111320
lon_degree_to_meter = lambda latitude : 40075000 * math.cos(math.radians(latitude)) / 360  

SCALE = 0.5

class HuskyVisualizer(Node):
    def __init__(self):
        super().__init__('husky_visualizer')

        self.gpsSub = self.create_subscription(Odometry, '/husky/gps', self.gpsCallback, 10)
        self.waypointSub = self.create_subscription(Point, '/waypoint', self.waypointCallback, 10)
        self.odometrySub = self.create_subscription(Odometry, '/husky/odom', self.odometryCallback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation (yaw angle)

        self.latitude = None
        self.longitude = None

        self.x_offset = 0.0
        self.y_offset = 0.0

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.waypoint = (0.0, 0.0)
        self.path_history = []

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((800, 800))
        pygame.display.set_caption("Husky Visualizer")
        self.clock = pygame.time.Clock()

        # Initialize robot image (as a surface to be rotated)
        self.robot_width = 50 * SCALE
        self.robot_height = 50 * SCALE
        self.robot_surface = pygame.Surface((self.robot_width, self.robot_height), pygame.SRCALPHA)
        self.robot_surface.fill((0, 0, 255))  # Fill surface with the robot's color

        self.timer = self.create_timer(0.1, self.update_visualization)  # Update every 0.1 seconds


    def odometryCallback(self, msg):
        _, _, self.theta = self.quaternion_to_euler(msg.pose.pose.orientation)

        # Update linear and angular velocity from the twist message
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z


    def gpsCallback(self, msg):  # https://github.com/KumarRobotics/ublox/blob/ros2/ublox_msgs/msg/NavPVT.msg
        if self.latitude is None and self.longitude is None:
            self.latitude  = msg.point.y
            self.longitude = msg.point.x

            self.y_offset = self.latitude  * lat_degree_to_meter
            self.x_offset = self.longitude * lon_degree_to_meter(self.latitude)

        latitude  = msg.point.y
        longitude = msg.point.x

        # Calculate delta in meters (Global to local coordinates)
        lat_meters = (latitude  - self.latitude)  * lat_degree_to_meter
        lon_meters = (longitude - self.longitude) * lon_degree_to_meter(latitude  - self.latitude)

        self.y = self.y + lat_meters  # Positive latitude  is North, negative is South
        self.x = self.x + lon_meters  # Positive longitude is East,  negative is West

        self.latitude = latitude
        self.longitude = longitude

        self.newPositionData = True


    def waypointCallback(self, msg):
        self.waypoint[1] = (msg.y * lat_degree_to_meter) - self.y_offset
        self.waypoint[0] = (msg.x * lon_degree_to_meter(msg.y)) - self.x_offset


    def update_visualization(self):
        # Move the robot based on its velocity
        self.x += self.linear_velocity * np.cos(self.theta) * 0.1
        self.y += self.linear_velocity * np.sin(self.theta) * 0.1
        self.theta += self.angular_velocity * 0.1  # Update orientation (yaw)

        # print(f"Robot position: ({self.x:.2f}, {self.y:.2f}), velocity: ({self.linear_velocity:.2f}, {self.angular_velocity:.2f})")
        
        self.screen.fill((255, 255, 255))  # Clear the screen

        rotated_robot = pygame.transform.rotate(self.robot_surface, -math.degrees(self.theta))  # Rotate the robot surface based on theta
        rotated_rect = rotated_robot.get_rect(center=(self.x * 20 * SCALE + 400, self.y * SCALE * 20 + 400))    # Get the new rectangle of the rotated surface

        self.screen.blit(rotated_robot, rotated_rect.topleft)  # Draw the rotated robot
        pygame.draw.circle(self.screen, (0, 255, 0), (int(self.waypoint[0] * SCALE * 20 + 400), int(self.waypoint[1] * SCALE * 20 + 400)), 10 * SCALE)  # Draw the waypoint (green dot)

        for i in range(1, len(self.path_history)):  # Draw the path (red line)
            pygame.draw.line(self.screen, (255, 0, 0),
                             (int(self.path_history[i - 1][0] * SCALE * 20 + 400),  int(self.path_history[i - 1][1] * SCALE * 20 + 400)),
                             (int(self.path_history[i][0]     * SCALE * 20 + 400 ), int(self.path_history[i][1]     * SCALE * 20 + 400)), 2)
            
        pygame.display.flip()  # Update the display
        
        self.path_history.append((self.x, self.y))  # Add current robot position to the path history
        self.clock.tick(10)                         # Limit the frame rate


    def quaternion_to_euler(self, quaternion):
        """
        Convert quaternion to euler angles (roll, pitch, yaw)
        Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        """
        x = quaternion.x;  y = quaternion.y
        z = quaternion.z;  w = quaternion.w
        
        roll_x  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_y = math.asin(2.0 * (w * y - z * x))
        yaw_z   = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
        return roll_x, pitch_y, yaw_z

###

def main(args=None):
    rclpy.init(args=args)
    visualizer = HuskyVisualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
