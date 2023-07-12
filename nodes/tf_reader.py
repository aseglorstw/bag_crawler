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


def create_graphs(bag):
    rospy.init_node('tf_listener')
    buffer = load_buffer(bag)
    icp_x = []
    icp_y = []
    icp_z = []
    cloud_combined = []
    saved_times = list()
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
                transform_icp = buffer.lookup_transform_full("map", time, "base_link", time, "map", rospy.Duration(1))
                icp_x.append(transform_icp.transform.translation.x)
                icp_y.append(transform_icp.transform.translation.y)
                icp_z.append(transform_icp.transform.translation.z)
                saved_times.append(save_time)
                transform_map_os_sensor = buffer.lookup_transform_full("map", time, "os_sensor", time, "map",
                                                                       rospy.Duration(1))
                matrix = numpify(transform_map_os_sensor.transform)
                vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4]
                cloud_combined.append(transformed_vectors)
            except ExtrapolationException:
                continue
    if len(cloud_combined) > 0:
        create_graph_tf_and_point_cloud(cloud_combined, icp_x, icp_y)
        create_graph_x_coord_and_time(icp_x, saved_times)
        create_graph_y_coord_and_time(icp_y, saved_times)
        create_graph_z_coord_and_time(icp_z, saved_times)

        
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


def create_graph_tf_and_point_cloud(cloud_combined, coord_x_robot, coord_y_robot):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/shared_point_cloud.png"
    fig, ax = plt.subplots()
    marker_size = 0.5
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("Shared Point Cloud with Coordinate Graph (Base Link relative to Map)")
    combined_points = np.concatenate(cloud_combined, axis=1)
    colors = combined_points[2, :]
    colors += abs(np.min(colors))
    transformed_colors = np.log1p(colors)
    ax.scatter(combined_points[0, :], combined_points[1, :], s=marker_size, c=transformed_colors, cmap='Blues')
    ax.plot(coord_x_robot, coord_y_robot, color='red')
    plt.savefig(output_path)
    plt.close()


def create_graph_x_coord_and_time(coord_x_robot, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_x_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('Time')
    plt.ylabel('X-coordinate')
    plt.title("Change of coordinate X with time")
    ax.plot(saved_times, coord_x_robot, color='blue')
    plt.savefig(output_path)
    plt.close()


def create_graph_y_coord_and_time(coord_y_robot, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_y_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('Time')
    plt.ylabel('Y-coordinate')
    plt.title("Change of coordinate Y with time")
    ax.plot(saved_times, coord_y_robot, color='blue')
    plt.savefig(output_path)
    plt.close()


def create_graph_z_coord_and_time(coord_z_robot, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_z_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('Time')
    plt.ylabel('Z-coordinate')
    plt.title("Change of coordinate Z with time")
    ax.plot(saved_times, coord_z_robot, color='blue')
    plt.savefig(output_path)
    plt.close()
