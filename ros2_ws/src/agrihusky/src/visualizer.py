#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import numpy as np
import pygame
import math

class HuskyVisualizer(Node):
    def __init__(self):
        super().__init__('husky_visualizer')

        # Variables to store robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation (yaw angle)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.waypoint = Point()

        # ROS 2 Subscribers
        self.odometrySub = self.create_subscription(Odometry, '/pixy/vicon/husky/husky/odom', self.odometryCallback, 10)
        self.waypointSub = self.create_subscription(Point, '/unitree_planner/waypoint', self.waypointCallback, 10)

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((800, 800))
        pygame.display.set_caption("Husky Visualizer")
        self.clock = pygame.time.Clock()

        # Colors
        self.robot_color = (0, 0, 255)  # Blue
        self.waypoint_color = (0, 255, 0)  # Green
        self.path_color = (255, 0, 0)  # Red

        # Path history (for drawing the path)
        self.path_history = []

        self.timer = self.create_timer(0.1, self.update_visualization)  # Update every 0.1 seconds

        # Initialize robot image (as a surface to be rotated)
        self.robot_width = 50
        self.robot_height = 50
        self.robot_surface = pygame.Surface((self.robot_width, self.robot_height), pygame.SRCALPHA)
        self.robot_surface.fill(self.robot_color)  # Fill surface with the robot's color

    def odometryCallback(self, msg):
        # Extract robot position and orientation (yaw)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = self.quaternion_to_euler(q)

        # Update linear and angular velocity from the twist message
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def waypointCallback(self, msg):
        # Update the current waypoint
        self.waypoint = msg

    def update_visualization(self):
        # Move the robot based on its velocity (simple kinematics)
        delta_x = self.linear_velocity * np.cos(self.theta) * 0.1  # Assume 0.1s time step
        delta_y = self.linear_velocity * np.sin(self.theta) * 0.1
        self.x += delta_x
        self.y += delta_y
        self.theta += self.angular_velocity * 0.1  # Update orientation (yaw)

        # Debugging: Print robot's position and velocity
        print(f"Robot position: ({self.x:.2f}, {self.y:.2f}), velocity: ({self.linear_velocity:.2f}, {self.angular_velocity:.2f})")

        # Clear the screen
        self.screen.fill((255, 255, 255))  # White background

        # Rotate the robot surface based on theta
        rotated_robot = pygame.transform.rotate(self.robot_surface, -math.degrees(self.theta))  # Convert rad to degrees

        # Get the new rectangle of the rotated surface
        rotated_rect = rotated_robot.get_rect(center=(self.x * 20 + 400, self.y * 20 + 400))

        # Draw the rotated robot
        self.screen.blit(rotated_robot, rotated_rect.topleft)

        # Draw the waypoint (green dot)
        pygame.draw.circle(self.screen, self.waypoint_color, (int(self.waypoint.x * 20 + 400), int(self.waypoint.y * 20 + 400)), 10)

        # Draw the path (red line)
        for i in range(1, len(self.path_history)):
            pygame.draw.line(self.screen, self.path_color,
                             (int(self.path_history[i - 1][0] * 20 + 400), int(self.path_history[i - 1][1] * 20 + 400)),
                             (int(self.path_history[i][0] * 20 + 400), int(self.path_history[i][1] * 20 + 400)),
                             2)

        # Update the display
        pygame.display.flip()

        # Add current robot position to the path history
        self.path_history.append((self.x, self.y))

        # Limit the frame rate
        self.clock.tick(10)

    def quaternion_to_euler(self, quaternion):
        """
        Convert quaternion to euler angles (roll, pitch, yaw)
        Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        roll_x = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_y = math.asin(2.0 * (w * y - z * x))
        yaw_z = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)

    # Initialize the HuskyVisualizer node
    visualizer = HuskyVisualizer()

    # Run the ROS 2 node
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
