#!/usr/bin/env python

import os, rospy, rospkg, pygame, numpy as np

from ublox_msg.msg import UbxNavPvt 

###

class GPSVisualizer:
    def __init__(self):
        rospy.init_node('gps_visualizer')

        self.path = []
        self.latitude = None
        self.longitude = None

        # Image and real-world GPS corner mappings
        self.top_left =     (65.617324, 22.135317)
        self.top_right =    (65.617324, 22.140283)
        self.bottom_left =  (65.616380, 22.135317)
        self.bottom_right = (65.616380, 22.140283)
        # Convert GPS coordinates to meters
        self.lat_degree_to_meter = 111320
        self.lon_degree_to_meter = lambda lat: 40075000 * np.cos(np.radians(lat)) / 360
        # Initial position in meters
        self.x = (22.1373941 - self.bottom_left[1]) * self.lon_degree_to_meter(self.bottom_left[0])
        self.y = (65.6166129 - self.bottom_left[0]) * self.lat_degree_to_meter        


        # Load background image
        self.img = pygame.image.load(rospkg.RosPack().get_path('gps_map') + '/imgs/map_background.jpg')  # Replace with your image filename
        self.width_p, self.height_p = self.img.get_size()
        # Meters across the image
        width_m  = (self.bottom_right[1] - self.bottom_left[1])  * self.lon_degree_to_meter((self.top_left[0] + self.bottom_left[0]) / 2)
        height_m = (self.top_left[0] - self.bottom_left[0])      * self.lat_degree_to_meter
        # Meters per pixel
        self.mpp_x = width_m / self.width_p
        self.mpp_y = height_m / self.height_p

        # Set up Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.width_p, self.height_p))
        pygame.display.set_caption("GPS Robot Visualizer")
        self.clock = pygame.time.Clock()

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


    def run(self):
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

            self.screen.blit(self.img, (0, 0))

            if self.latitude and self.longitude:
                # Convert x, y in meters to pixels relative to the center or origin
                pixel_x = int(self.x / self.mpp_x);  pixel_y = int(self.y / self.mpp_y)

                # Flip y-axis if needed (since Pygame's y increases downward)
                pixel_y = self.height_p - pixel_y
                pixel = (pixel_x, pixel_y)
                self.path.append(pixel)

                if len(self.path) > 1:  # Draw path
                    pygame.draw.lines(self.screen, (255, 0, 0), False, self.path, 2)

                # Draw current robot position
                pygame.draw.circle(self.screen, (0, 0, 255), pixel, 8)

            pygame.display.flip()
            self.clock.tick(10)

###

if __name__ == "__main__":
    try:
        visualizer = GPSVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
