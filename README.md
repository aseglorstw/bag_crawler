# Bag Crawler

Post-processing tool to analyze the ROS bag-files available at: [http://subtdata.felk.cvut.cz/robingas/data/](http://subtdata.felk.cvut.cz/robingas/data/).


An important part of the processing is finding the topic based on which the start and end of the robot's movement, average and maximum speeds, and similar metrics will be calculated. This topic will be referred to as the **selected topic** in this documentation. In the case where an ICP topic exists, it becomes the selected topic. When such a topic is not present, the chosen topic will be an odom topic containing GPS in its name. If neither of these topics exists, the selected topic will be a randomly available odom topic.


The script starting from the root folder searches for bag files to process. For each bag-file being processed the following data is generated:

- Robot motion trajectories based on all available topics and a point cloud along the trajectory given a selected topic.

<p align="center">
  <img src="https://i.imgur.com/yRb9LH6.png">
  <br><sub> Trajectories and point cloud along the trajectory given by topic /icp_odom for the bag file "/robingas/data/22-09-27-unhost/husky/husky_2022-09-09-27-15-15-01-44.bag".</sub>
</p>



- Graphs of the change in a single robot coordinate over time.

<p align="center">
  <img src="https://i.imgur.com/W0eKOjl.png" width="30%">
  <img src="https://i.imgur.com/uZCRINs.png" width="30%">
  <img src="https://i.imgur.com/fhpaeno.png" width="30%">
 <br><sub>Graphs for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag".</sub>
</p>

- Graphs of changes in the distance traveled by the robot from all available topics.

<p align="center">
  <img src="https://i.imgur.com/lr0ewq8.png">
   <br><sub>Graph for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag". <br> 
     The start and end times of the movement were calculated based on data from the selected topic - /icp_odom.</sub>
</p>


- Graph showing the marks of controlling the robot with the joystick. 

<p align="center">
  <img src="https://i.imgur.com/WQrwHTR.png">
   <br><sub>Graphfor bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag". <br> 
    The principle is that on top of the trajectory given by the selected topic, areas are drawn where the robot was controlled by the joystick. </sub> 
</p>























The post-processed data for each bag-file is saved in a separate folder.
For example:
TODO

## Installation

TODO

## Usage

TODO

## TODOs

Web-based application that monitors a directory for ROS bag files and visualizes their metadata.
