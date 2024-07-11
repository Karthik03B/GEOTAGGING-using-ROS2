#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/.bashrc
ros2 launch geocam geotag.launch.xml
