#!/bin/bash

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: 'map1'}"