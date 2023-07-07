from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.point_cloud2 import read_points
import numpy as np


"""
Every 10 seconds I save the coordinates of the points of the cloud, the color changes depending on the height
"""


def save_cloud(bag):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/point_clouds/point_cloud_"
    number_of_cloud = 0
    saved_times = set()
    for topic, msg, time in bag.read_messages(topics=['/points']):
        time = rospy.Time.from_sec(time.to_sec())
        time_sec = time.to_sec()
        start_time = bag.get_start_time()
        time_sec = time_sec - start_time
        save_time = int(time_sec)
        if save_time % 10 == 0 and save_time not in saved_times:
            saved_times.add(save_time)
            msg = PointCloud2(*slots(msg))
            cloud = np.array(list(read_points(msg)))
            colors = cloud[:, 2]
            marker_size = 0.5
            fig, ax = plt.subplots()
            ax.scatter(cloud[:, 0], cloud[:, 1], s=marker_size, c=colors, cmap='winter', alpha=1)
            plt.xlim(-10, 10)
            plt.ylim(-10, 10)
            plt.xlabel('X-coordinate')
            plt.ylabel('Y-coordinate')
            plt.title('Point Cloud')
            plt.savefig(output_path + str(number_of_cloud) + ".png")
            number_of_cloud += 1
            plt.close()


def slots(msg):
    return [getattr(msg, var) for var in msg.__slots__]
