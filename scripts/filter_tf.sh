#!/bin/bash
bag_in="$1"
bag_out="$2"
rosbag filter "$bag_in" "$bag_out" "topic == '/tf' and m.transforms[0].child_frame_id == 'odom' and m.transforms[0].header.frame_id == 'map'"
