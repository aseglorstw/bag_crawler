import rospy
import tf2_ros
from rosbag import ROSBagException
from tqdm import tqdm
import numpy as np
from math import sqrt
import yaml


def slots(msg):
    return [getattr(msg, var) for var in msg.__slots__]


def load_buffer(bag):
    tf_topics = ['/tf', '/tf_static', 'points']
    buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600))
    try:
        for topic, msg, stamp in tqdm(bag.read_messages(topics=tf_topics),
                                      total=bag.get_message_count(topic_filters=tf_topics)):
            if topic == '/tf':
                for tf in msg.transforms:
                    buffer.set_transform(tf, 'bag')
            elif topic == '/tf_static':
                for tf in msg.transforms:
                    buffer.set_transform_static(tf, 'bag')
    except ROSBagException:
        print('Could not read')
    return buffer


def transform_z_coordinates_to_color(coord_z):
    coord_z += abs(np.min(coord_z))
    colors = np.log1p(coord_z)
    return colors


def move_coordinates_to_the_origin(coordinates):
    return np.array(coordinates) - coordinates[0]


def get_distances(coord):
    distances_one_period = np.abs(coord[1:] - coord[:-1])
    distances_xyz = [[0, 0, 0]]
    distances = [0]
    for distance in distances_one_period:
        distances_xyz.append(distances_xyz[-1] + distance)
        distances.append(sqrt(pow(distances_xyz[-1][0], 2) + pow(distances_xyz[-1][1], 2) +
                              pow(distances_xyz[-1][2], 2)))
    return distances


def find_start_and_end_of_moving(speeds, saved_times):
    start_of_moving = -1
    end_of_moving = -1
    for i in range(len(speeds)):
        if speeds[i] > 0.2 and start_of_moving == -1:
            start_of_moving = saved_times[i]
        elif speeds[i] < 0.2 or (i == len(speeds) - 1 and speeds[i] > 0.2):
            end_of_moving = saved_times[i]
    return start_of_moving, end_of_moving


def get_speeds_one_period(icp, saved_times):
    distances_one_period = np.abs(icp[1:] - icp[:-1])
    saved_times = np.array(saved_times)
    times_one_period = saved_times[1:] - saved_times[:-1]
    speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
    speeds = []
    for speed in speeds_xyz:
        speeds.append(sqrt(pow(speed[0], 2) + pow(speed[1], 2) + pow(speed[2], 2)))
    return speeds


def find_joy_topic(bag):
    topics_info = bag.get_type_and_topic_info()[1]
    for topic_name, topics_info in topics_info.items():
        if "joy" in topic_name:
            return topic_name
    return -1


def create_time_array(bag):
    info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.Loader)
    time_array = np.arange(0, info_dict['duration'], 1, dtype=int)
    return time_array


def create_array_of_binary_control_joy(time_array, saved_times):
    control_joy = []
    for time in time_array:
        if time in saved_times:
            control_joy.append(1)
        else:
            control_joy.append(0)
    return control_joy
