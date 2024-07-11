#!/bin/bash

cd
cd microros_ws
source install/local_setup.bash
cd
cd ardupilot/libraries/AP_DDS
ros2 run micro_ros_agent micro_ros_agent serial -b 115200  -r dds_xrce_profile.xml -D  /dev/ttyACM2


