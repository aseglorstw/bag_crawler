import subprocess
import time
import os



def create_bag_with_transform_from_map(bag, path):
    ros_env = os.environ.copy()
    ros_env['ROS_PACKAGE_PATH'] = '/opt/ros/noetic/share'
    ros_env['ROS_MASTER_URI'] = 'http://localhost:11311'
    roscore_process = subprocess.Popen(["/opt/ros/noetic/bin/roscore"], env=ros_env)
    slam_process = subprocess.Popen(
        ["/opt/ros/noetic/bin/roslaunch", "bag_crawler", "slam.launch", "cloud:=points"], env=ros_env)
    record_process = subprocess.Popen(["/opt/ros/noetic/bin/rosbag", "record", "-O",
                                       "/home/robert/catkin_ws/src/bag_crawler/bagfiles/full.bag", "/tf",
                                       "/tf_static"], env=ros_env)
    play_process = subprocess.Popen(["/opt/ros/noetic/bin/roslaunch", "bag_crawler", "demo.launch",
                                     "bag:=/home/robert/catkin_ws/src/bag_crawler/bagfiles/husky_2022-10-27-15-33-57.bag"],
                                    env=ros_env)
    time.sleep(10)
    play_process.terminate()
    play_process.wait()
    record_process.terminate()
    record_process.wait()
    slam_process.terminate()
    slam_process.wait()
    roscore_process.terminate()
    roscore_process.wait()