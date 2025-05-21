#!/bin/bash

source ../install/setup.bash
gnome-terminal -- "ros2 launch agrihusky simulation.launch.py"
gnome-terminal -- "ros2 run agrihusky fake_waypoint_pub.py"