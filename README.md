# Bag Crawler

**Post-processing tool to analyze the ROS bag-files available at: [http://subtdata.felk.cvut.cz/robingas/data/](http://subtdata.felk.cvut.cz/robingas/data/).**


An important part of the processing is finding the topic based on which the start and end of the robot's movement, average and maximum speeds, and similar metrics will be calculated. This topic will be referred to as the **selected topic** in this documentation. In the case where an ICP topic exists, it becomes the selected topic. When such a topic is not present, the chosen topic will be an odom topic containing GPS in its name. If neither of these topics exists, the selected topic will be a randomly available odom topic. 


**The script starting from the root folder searches for bag files to process. For each bag-file being processed the following data is generated:**

- **Robot motion trajectories based on all available topics and a point cloud along the trajectory given a selected topic.**

<p align="center">
  <img src="https://i.imgur.com/WOxdYEp.png">
  <br><sub> Trajectories and point cloud along the trajectory given by topic /icp_odom for the bag file "/robingas/data/22-09-27-unhost/husky/husky_2022-09-09-27-15-15-01-44.bag".</sub>
</p>



- **Graphs of the change in a single robot coordinate over time.**

<p align="center">
  <img src="https://i.imgur.com/w9VXm3Q.png" width="30%">
  <img src="https://i.imgur.com/s2pus5G.png" width="30%">
  <img src="https://i.imgur.com/7Ux5AtC.png" width="30%">
 <br><sub>Graphs for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag".</sub>
</p>


- **Graphs of changes in the distance traveled by the robot from all available topics.**

<p align="center">
  <img src="https://i.imgur.com/lr0ewq8.png">
   <br><sub>Graph for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag". <br> 
     The start and end times of the movement were calculated based on data from the selected topic - /icp_odom.</sub>
</p>


- **Graph showing the marks of controlling the robot with the joystick.** 

<p align="center">
  <img src="https://i.imgur.com/AcNqizz.png">
   <br><sub>Graph for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-27-10-33-15.bag". <br> 
    The principle is that on top of the trajectory given by the selected topic - /husky_velocity_controller/odom, areas are drawn where the robot was controlled by the joystick. </sub> 
</p>


- **The video is 20 seconds long from every possible color, black and white, and depth cameras.**

<p align="center">
  <img src="https://i.imgur.com/EnJfNqI.gif" width="30%">
  <img src="https://i.imgur.com/7r34Yvu.gif" width="30%">  
  <img src="https://i.imgur.com/W7NMVqa.gif" width="30%">  
  <br><sub> Black and white video and depth camera video belongs to bag file "/robingas/data/22-10-20-unhost/spot_2022-10-20-12-39-17.bag".  <br> Color video from bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag".  
   </sub>
</p>

The script at the video processing stage checks the camera location based on the TF ROS library and if the camera is rotated, the program will flip the image for correct display. 

Depth camera image point filtering is also present. It works in such a way that all points lying above the 99th percentile and under the 2nd percentile are cut off.

If a bag file has a topic containing "omnicam" in its name, the script will split this panoramic image into 5 separate videos, i.e. front, right, back, left and top views, and then save them. 

- **Information about the bag file.**

Example of "bag_info.json" file for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag":

```json
{
    "distance": 71.85627031389019,
    "average_speed": 0.6973400679781954,
    "start of moving": 0.07594871520996094,
    "end of moving": 103.11931824684143,
    "start": "2022-09-27 15:01:44",
    "end": "2022-09-27 15:03:27",
    "duration": 103.311347,
    "size": 4682860168,
    "messages": 242458
}
``` 
Thus this file contains information about the beginning and end of bag file recording, its duration, size, total number of messages in all topics. And also based on the data from the selected topic, information about the average speed, distance traveled, start and end of the robot's movement.

- **Information about the bag file topics.**

So, for example, for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag" the script generates the file "topics_info.json", which for each topic contains information about the number of messages in this topic, maximum and average waiting time between messages:


```json 
 "/camera_front/camera_info": {
        "msg type": "sensor_msgs/CameraInfo",
        "message count": 510,
        "average time delay": 0.202,
        "max time delay": 0.414
  }
```

- **Information about the moving parts of the robot.**

Based on the topic "/joint_states" the script saves information about moving parts of the robot to the file "moving_joints_info.json". So for "/robingas/data/22-10-20-unhost/spot_2022-10-20-20-12-39-17.bag" this file will look as follows:
```json 
[
    "front_left_hip_x",
    "front_left_knee",
    "front_left_hip_y",
    "rear_right_hip_y",
    "rear_right_hip_x",
    "rear_right_knee",
    "front_right_hip_x",
    "front_right_hip_y",
    "front_right_knee",
    "rear_left_knee",
    "rear_left_hip_x",
    "rear_left_hip_y"
]
```

- **A brief description of what the robot did when writing this bag file based on the data from the selected topic.**

For bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag", as you can see from the video excerpt above the robot overcomes a small obstacle, so in the file "movement_tag.txt" is written:  
```
overcame_obstacle
```


- **Information about how the robot was controlled.**

In order for the "controller_info.json" file to contain information about which ways the robot was controlled when this bag file was written, it is necessary to add possible ways of control for this robot to the config file. Then, based on the configuration, the script checks for the presence of the corresponding topic in the bag file. 

The post-processed data for each bag-file is saved in a separate folder.
For example:
TODO

## Installation

TODO

## Usage

TODO

## TODOs

Web-based application that monitors a directory for ROS bag files and visualizes their metadata.
