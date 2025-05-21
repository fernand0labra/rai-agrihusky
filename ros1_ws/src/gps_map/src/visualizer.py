#!/usr/bin/env python

import rospy
import pygame
import numpy as np
from ublox_msg.msg import UbxNavPvt  # Standard GPS message
import os

# rosbag play --start=130 --duration=70 data/gps-2025-05-20-13-37-14/gps-2025-05-20-13-37-14.bag

class GPSVisualizer:
    def __init__(self):
        rospy.init_node('gps_visualizer')

        self.lat_degree_to_meter = 111320  # approx. meters per degree latitude
        self.lon_degree_to_meter = lambda lat: 40075000 * np.cos(np.radians(lat)) / 360

        # Load background image
        self.bg_image = pygame.image.load('/home/fernand0labra/rai-agrihusky/src/gps_map/imgs/map_background.jpg')  # Replace with your image filename
        self.bg_width, self.bg_height = self.bg_image.get_size()

        # Image and real-world GPS corner mappings
        self.top_left = (65.617324, 22.135317)
        self.top_right = (65.617324, 22.140283)
        self.bottom_left = (65.616380, 22.135317)
        self.bottom_right = (65.616380, 22.140283)

        # Top-left and bottom-left coordinates for lat span
        lat_span = self.top_left[0] - self.bottom_left[0]  # degrees
        lon_span = self.bottom_right[1] - self.bottom_left[1]  # degrees

        # Meters across the image
        height_m = lat_span * self.lat_degree_to_meter
        width_m = lon_span * self.lon_degree_to_meter((self.top_left[0] + self.bottom_left[0]) / 2)

        # Meters per pixel
        self.mpp_x = width_m / self.bg_width
        self.mpp_y = height_m / self.bg_height

        # Latitude: 65.6166129, Longitude: 22.1373941
        self.x = (22.1373941 - self.bottom_left[1]) * self.lon_degree_to_meter(self.bottom_left[0])
        self.y = (65.6166129 - self.bottom_left[0]) * self.lat_degree_to_meter

        # Set up Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.bg_width, self.bg_height))
        pygame.display.set_caption("GPS Robot Visualizer")
        self.clock = pygame.time.Clock()

        # Robot path
        self.path = []

        # Current GPS position
        self.latitude = None
        self.longitude = None

        # Subscribe to GPS data
        rospy.Subscriber("/ublox_client", UbxNavPvt, self.gps_callback)

    def gps_callback(self, msg):
        if self.latitude is None and self.longitude is None:
            self.latitude  = msg.lat
            self.longitude = msg.lon

        latitude = msg.lat
        longitude = msg.lon

        # Calculate delta in meters (Global to local coordinates)
        lat_meters  = (latitude  - self.latitude)  * self.lat_degree_to_meter
        long_meters = (longitude - self.longitude) * self.lon_degree_to_meter(latitude)

        self.y = self.y + lat_meters  # Positive latitude  is North, negative is South
        self.x = self.x + long_meters # Positive longitude is East,  negative is West

        self.latitude = latitude
        self.longitude = longitude

        # print(f"Latitude: {self.latitude}, Longitude: {self.longitude}")
        print(f"X: {self.x}, Y: {self.y}")

    def latlon_to_pixel(self, lat, lon):
        # Interpolate pixel location from GPS coordinates
        lat_min = self.bottom_left[0]
        lat_max = self.top_left[0]
        lon_min = self.top_left[1]
        lon_max = self.top_right[1]

        # Normalize
        x_ratio = (lon - lon_min) / (lon_max - lon_min)
        y_ratio = (lat_max - lat) / (lat_max - lat_min)

        x_pixel = int(x_ratio * self.bg_width)
        y_pixel = int(y_ratio * self.bg_height)

        return x_pixel, y_pixel

    def run(self):
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

            self.screen.blit(self.bg_image, (0, 0))

            if self.latitude and self.longitude:

                # Convert x, y in meters to pixels relative to the center or origin
                pixel_x = int(self.x / self.mpp_x)
                pixel_y = int(self.y / self.mpp_y)

                # Flip y-axis if needed (since Pygame's y increases downward)
                pixel_y = self.bg_height - pixel_y

                pixel = (pixel_x, pixel_y)
                # pixel = self.latlon_to_pixel(self.latitude, self.longitude)

                self.path.append(pixel)

                # Draw path
                if len(self.path) > 1:
                    pygame.draw.lines(self.screen, (255, 0, 0), False, self.path, 2)

                # Draw current robot position
                pygame.draw.circle(self.screen, (0, 0, 255), pixel, 8)

            pygame.display.flip()
            self.clock.tick(10)


if __name__ == "__main__":
    try:
        visualizer = GPSVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
