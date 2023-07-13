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
from math import sqrt


def create_graphs(bag):
    rospy.init_node('tf_listener')
    buffer = load_buffer(bag)
    icp_x = []
    icp_y = []
    icp_z = []
    imu_x = []
    imu_y = []
    imu_z = []
    cloud_combined = []
    saved_times = []
    start_time = bag.get_start_time()
    for idx, (topic, msg, time) in enumerate(bag.read_messages(topics=['/points'])):
        time = rospy.Time.from_sec(time.to_sec())
        save_time = time.to_sec() - start_time
        try:
            transform_icp = buffer.lookup_transform_full("map", time, "base_link", time, "map", rospy.Duration(1))
            icp_x.append(transform_icp.transform.translation.x)
            icp_y.append(transform_icp.transform.translation.y)
            icp_z.append(transform_icp.transform.translation.z)

            transform_imu = buffer.lookup_transform_full("odom", time, "base_link", time, "odom", rospy.Duration(1))
            imu_x.append(transform_imu.transform.translation.x)
            imu_y.append(transform_imu.transform.translation.y)
            imu_z.append(transform_imu.transform.translation.z)

            saved_times.append(save_time)
            if idx % 10 == 0:
                msg = PointCloud2(*slots(msg))
                cloud = np.array(list(read_points(msg)))
                transform_map_os_sensor = buffer.lookup_transform_full("map", time, "os_sensor", time, "map",
                                                                       rospy.Duration(1))
                matrix = numpify(transform_map_os_sensor.transform)
                vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4]
                cloud_combined.append(transformed_vectors)
        except ExtrapolationException:
            continue
    if len(cloud_combined) > 0:
        create_graph_tf_and_point_cloud(cloud_combined, icp_x, icp_y, imu_x, imu_y)
        create_graph_x_coord_and_time(icp_x, imu_x, saved_times)
        create_graph_y_coord_and_time(icp_y, imu_y, saved_times)
        create_graph_z_coord_and_time(icp_z, imu_z, saved_times)
        create_graph_distance_and_time(icp_x, icp_y, icp_z, imu_x, imu_y, imu_z, saved_times)

        
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


def create_graph_tf_and_point_cloud(cloud_combined, icp_x, icp_y, imu_x, imu_y):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/shared_point_cloud.png"
    fig, ax = plt.subplots()
    marker_size = 0.5
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement")
    combined_points = np.concatenate(cloud_combined, axis=1)
    colors = z_coord_transform_for_color(combined_points[2, :])
    ax.scatter(combined_points[0, :], combined_points[1, :], s=marker_size, c=colors, cmap='Greens')
    ax.plot(imu_x, imu_y, color='blue', label='imu_odom')
    ax.plot(icp_x, icp_y, color='red', linestyle='--', label='icp_odom')
    plt.legend()
    plt.savefig(output_path)
    plt.close()


def create_graph_x_coord_and_time(icp_x, imu_x, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_x_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in X direction")
    ax.plot(saved_times, move_coordinates_to_the_origin(imu_x), color='blue')
    ax.plot(saved_times, move_coordinates_to_the_origin(icp_x), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_y_coord_and_time(icp_y, imu_y, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_y_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Y direction")
    ax.plot(saved_times, move_coordinates_to_the_origin(imu_y), color='blue')
    ax.plot(saved_times, move_coordinates_to_the_origin(icp_y), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_z_coord_and_time(icp_z, imu_z, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_z_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Z direction")
    ax.plot(saved_times, move_coordinates_to_the_origin(imu_z), color='blue')
    ax.plot(saved_times, move_coordinates_to_the_origin(icp_z), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_distance_and_time(icp_x, icp_y, icp_z, imu_x, imu_y, imu_z, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/distance_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's travelled distance over time")
    icp_x = np.array(icp_x)
    icp_distances_x = np.abs(icp_x[1:] - icp_x[:-1])
    icp_movement_x = [0]
    for distance in icp_distances_x:
        icp_movement_x.append(distance + icp_movement_x[-1])
    icp_y = np.array(icp_y)
    icp_distances_y = np.abs(icp_y[1:] - icp_y[:-1])
    icp_movement_y = [0]
    for distance in icp_distances_y:
        icp_movement_y.append(distance + icp_movement_y[-1])
    icp_z = np.array(icp_z)
    icp_distances_z = np.abs(icp_z[1:] - icp_z[:-1])
    icp_movement_z = [0]
    for distance in icp_distances_z:
        icp_movement_z.append(distance + icp_movement_z[-1])
    icp_movement = []
    for i in range(len(icp_movement_x)):
        icp_movement.append(sqrt(pow(icp_movement_x[i], 2) + pow(icp_movement_y[i], 2) + pow(icp_movement_z[i], 2)))
    ax.plot(saved_times, icp_movement, color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def z_coord_transform_for_color(coord_z):
    coord_z += abs(np.min(coord_z))
    colors = np.log1p(coord_z)
    return colors


def move_coordinates_to_the_origin(coordinates):
    return np.array(coordinates) - coordinates[0]
