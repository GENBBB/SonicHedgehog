#!/bin/bash
ros2 launch sonic_hedgehog nav_slam_to_point.launch.py use_sim:=false my_lidar:=true send_goal:=true goal_x:=1.3 goal_y:=1.3