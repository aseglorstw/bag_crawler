# Bag Crawler

**Post-processing tool to analyze the ROS bag-files available at: [http://subtdata.felk.cvut.cz/robingas/data/](http://subtdata.felk.cvut.cz/robingas/data/).**


An important part of the processing is finding the topic based on which the start and end of the robot's movement, average and maximum speeds, and similar metrics will be calculated. This topic will be referred to as the **selected topic** in this documentation. In the case where an ICP topic exists, it becomes the selected topic. When such a topic is not present, the chosen topic will be an odom topic containing GPS in its name. If neither of these topics exists, the selected topic will be a randomly available odom topic. 


**The script starting from the root folder searches for bag files to process. For each bag-file being processed the following data is generated:**

- **Robot motion trajectories based on all available topics and a point cloud along the trajectory given a selected topic.**

<p align="center">
  <img src="https://i.imgur.com/WOxdYEp.png">
  <br><sub> Trajectories and point cloud for the bag file "/robingas/data/22-09-27-unhost/husky/husky_2022-09-09-27-15-15-01-44.bag" with selected topic /icp_odom.</sub>
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
   <br><sub>Graph for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-09-09-09-27-15-15-01-44.bag" with selected_topic /icp_odom.</sub>
</p>

On the chart, you can see the start and end marks of the robot's movement, which are important points of the file and are made based on the data from the selected topic.

- **The trajectories of this selected topic with joystick control marks.** 

<p align="center">
  <img src="https://i.imgur.com/AcNqizz.png">
   <br><sub>Graph for bag file "/robingas/data/22-09-27-unhost/husky/husky/husky_2022-09-27-10-33-15.bag" with selected topic - /husky_velocity_controller/odom.</sub> 
</p>

The principle is that on top of the trajectory given by selected topic areas are drawn where the robot was controlled by the joystick. The script checks the configuration file, then will try to make marks based on the topic from the configuration file. If it doesn't exist, it will try to find a topic that contains "joy" and "cmd_vel" in its name. 

- **The video is 20 seconds long from every possible color, black and white, and depth cameras.**

<p align="center">
  <img src="https://i.imgur.com/ffSW6FZ.gif" width="30%">
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

Based on the configuration file, the script checks for the presence of topics that characterize one of the control methods and if there were any messages in this topic, then this control method is written to the "controller_info.json" file.
This file might look like this:
```json 
[
    "gamepad_PC"
]
```


**The post-processed data for each bag-file is saved in a separate folder.**

<p align="center">
  <img src="https://i.imgur.com/cqT7kOV.png">
  <br><sub>A folder named ".web_server_spot_2022-10-20-12-39-17.bag" for the bag file "/robingas/data/22-10-20-unhost/spot_2022-10-20-12-39-17.bag".</sub>
</p>

**In addition to the data described above, several other files can be noticed:**


- Several images with suffix "demo" are made on the middle bag of the file and serve to briefly demonstrate where this file was recorded.

- A logging file ".data_availability.json", which contains information about which parts of the bag file were processed. In this case it looks like this:
  ```json 
    {
      "icp": false,
      "odom": true,
      "point_cloud": true,
      "joy": true,
      "video": true,
      "graphs": true,
      "bag_info": true
    }
  ```
Based on this information we can say that the icp topic was not processed or was not found. 

- The .npz and joy.json files are used to log the icp, odom, and topix reads to which commands are sent to control the robot with the joystick. 



## Installation
1. **Creating a .sif file.**

```
wget https://raw.githubusercontent.com/aseglorstw/bag_crawler/master/singularity/build.sh
wget https://raw.githubusercontent.com/aseglorstw/bag_crawler/master/singularity/recepie.def
chmod +x build.sh
./build.sh
```
Create a folder in a convenient location and build a container in it using the instructions above.

2. **Copy the image to the server.**

```
scp bag_crawler.sif username@login3.rci.cvut.cz:/mnt/personal/username/conteiners/
```
Use this command to copy an image from your local disk to a folder on the server, such as the "containers" folder.

3. **Assembling the project.**

```
singularity shell bag_crawler.sif
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/aseglorstw/bag_crawler.git
cd ..
catkin_make
chmod +x ~/catkin_ws/src/bag_crawler/singularity/run_script.sh
exit
```

Then you need to go to the container and build the project inside it, you can see more details about building in the [ROS documentation](https://wiki.ros.org/ROS/Tutorials/BuildingPackages).

## Usage

- **Configuration file.**

The configuration file can, but should not, be used. The situations where a configuration file is used are described above. There are three types of configuration file:
1. The global configuration file ".bag_crawler_global_config.json"  must be located in the root folder.
2. The local configuration file ".bag_crawler_local_config.json" will be used for bag files that are in the same folder as the configuration file.
3. The configuration file for one particular bag file must be in the same folder and contain the suffix ".config.json" instead of the suffix ".bag".  

<p align="center">
  <img src="https://i.imgur.com/xipDMeE.png">
  <br><sub>Example of a configuration file for a folder with one bag file. This configuration file will be used only for bag files in this folder. </sub>
</p>

The configuration file might look something like this:
```json 
{
    "elements_of_control": {
        "/local_joy/joy": "gamepad_PC" 
    }
}
```
You can add other topics to the "elements_of_control" values depending on the robot and bind the control method to them. The script at the stage of plotting the trajectory graph with joystick control marks will try to find in the configuration file the tops to which control types "robot_gamepad" or "gamepad_PC" are mapped.

- **Re-processing of bag files.**

 In case the script is called again on a folder that has already been processed, the program will check the logging file ".data_availability.json". The following are the situations that may occur:
 1. "icp": false or "odom": false or "point_cloud": false or "joy": false - the script will try to read the missing topic again, load the rest from the .npz or .json files if they were read successfully, and generate the graphs and the rest of the data once more. 
 2. "video": false - the script will try again to create the video.
 3. "graphs": false - script will load all the counted tops and create graphs.
 4. "bag_info": false - The script will load saved data from topics and, based on it, will generate files - bag_info.json, topics_info.json and other files of this kind.

- **Ignoring bag files.**

In a situation where you want to process or reprocess only part of the bag files from the entire folder, you can go two ways:

1. In the folder you want to process, create a file ".ignore.json" and write the files that you want to skip into it. For example:
```
[
    "ugv_2022-05-20-14-51-44.bag"
]
```
2. Use the "Bag_Ignore.py" module. To do this you will need the absolute path to the bag file. 

   To add a bag file to the ignore list, use the command:


- **Running the script.**

You can run the script for processing using the following command:
```
 singularity exec --bind /root_directory bag_crawler.sif ~/catkin_ws/src/bag_crawler/singularity/run_script.sh /root_directory 
```

Here "/root_directory" is the folder from which the script will start looking for bag files.  

