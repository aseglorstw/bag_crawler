from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.point_cloud2 import read_points
import numpy as np
from rosbag import ROSBagException
from tqdm import tqdm
import tf2_ros
from tf2_ros import ExtrapolationException
from ros_numpy import numpify


def save_shared_cloud_with_graph(bag):
    rospy.init_node('tf_listener')
    buffer = load_buffer(bag)
    coord_x_base_link = []
    coord_y_base_link = []
    cloud_combined = []
    saved_times = set()
    for topic, msg, time in bag.read_messages(topics=['/points']):
        time = rospy.Time.from_sec(time.to_sec())
        time_sec = time.to_sec()
        start_time = bag.get_start_time()
        time_sec = time_sec - start_time
        save_time = int(time_sec)
        if save_time % 5 == 0:
            msg = PointCloud2(*slots(msg))
            cloud = np.array(list(read_points(msg)))
            try:
                transform_map_os_sensor = buffer.lookup_transform_full("map", time, "os_sensor", time, "map",
                                                                       rospy.Duration(1))
                matrix = numpify(transform_map_os_sensor.transform)
                vectors = np.array([cloud[::400, 0], cloud[::400, 1], cloud[::400, 2]])
                transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4]
                cloud_combined.append(transformed_vectors)
                transform_map_base_link = buffer.lookup_transform_full("map", time, "base_link", time, "map",
                                                                       rospy.Duration(1))
                coord_x_base_link.append(transform_map_base_link.transform.translation.x)
                coord_y_base_link.append(transform_map_base_link.transform.translation.y)
                saved_times.add(save_time)
            except ExtrapolationException:
                continue
    if len(cloud_combined) > 0:
        save_png(cloud_combined, coord_x_base_link, coord_y_base_link)
        

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


def save_png(cloud_combined, coord_x_base_link, coord_y_base_link):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/shared_point_cloud.png"
    fig, ax = plt.subplots()
    marker_size = 0.5
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("Shared Point Cloud with Coordinate Graph (Base Link relative to Map)")
    combined_points = np.concatenate(cloud_combined, axis=1)
    colors = combined_points[2, :] * 2
    ax.scatter(combined_points[0, :], combined_points[1, :], s=marker_size, c=colors, cmap='winter', alpha=1)
    ax.plot(coord_x_base_link, coord_y_base_link, color='red')
    plt.savefig(output_path)
    plt.close()

