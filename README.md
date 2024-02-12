# Bag Crawler

Post-processing tool to analyze the ROS bag-files available at: [http://subtdata.felk.cvut.cz/robingas/data/](http://subtdata.felk.cvut.cz/robingas/data/).

The script recursively goes through all folders starting from the root folder in search of bag files. For each bag-file being processed the following data is generated:

- The trajectory of the robot's movement based on all possible topics and the point cloud along a selected path of the robot.

<p align="center">
  <img src="https://i.imgur.com/yRb9LH6.png" alt="Alt текст">
  <br><sub> Trajectories and point cloud along the trajectory given by icp topics for the bag file "husky_2022-09-09-27-15-15-01-44.bag".</sub>
</p>











- Point cloud map of the environment
- Video streams from all available cameras (RGB + Depth)
- Movement analysis: speed (mean, max)

The post-processed data for each bag-file is saved in a separate folder.
For example:
TODO

## Installation

TODO

## Usage

TODO

## TODOs

Web-based application that monitors a directory for ROS bag files and visualizes their metadata.
