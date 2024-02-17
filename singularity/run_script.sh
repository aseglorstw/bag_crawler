#!/bin/bash

if [ -d "~/catkin_ws/src" ]; then
    python3 ~/catkin_ws/src/bag_crawler/scripts/Bag_Crawler.py "$1"
else
    source /opt/ros/noetic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    cd src
    git clone https://github.com/aseglorstw/bag_crawler.git
    cd ~/catkin_ws
    catkin_make
    python3 ~/catkin_ws/src/bag_crawler/scripts/Bag_Crawler.py "$1"
fi
