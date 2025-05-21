#!/bin/bash

source ../install/setup.bash
bash -c "ros2 launch agrihusky simulation.launch.py" &
bash -c "ros2 run agrihusky fake_waypoint_pub.py" &