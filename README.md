# Bag Crawler

Post-processing tool to analyze the ROS bag-files available at: [http://subtdata.felk.cvut.cz/robingas/data/](http://subtdata.felk.cvut.cz/robingas/data/).

The script recursively goes through all folders starting from the root folder in search of bag files. For each bag-file being processed the following data is generated:

 <br>
-The trajectory of the robot's movement based on all possible topics and the point cloud along a selected path of the robot.

<p align="center">
  <img src="https://i.imgur.com/yRb9LH6.png" alt="Alt текст">
  <br><sub> Trajectories and point cloud along the trajectory given by icp topics for the bag file "husky_2022-09-09-27-15-15-01-44.bag".</sub>
</p>

Usually the point cloud is built along the path given by the icp topic. If it does not exist, then one of the odometry topics is selected so that the first topic containing GPS in its name is found. If there is no such topic, then a random one is taken from all odometry topics.

 <br>
-Graphs of the change in each individual coordinate and the distance traveled by the robot.









The post-processed data for each bag-file is saved in a separate folder.
For example:
TODO

## Installation

TODO

## Usage

TODO

## TODOs

Web-based application that monitors a directory for ROS bag files and visualizes their metadata.
