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
    odom_x = []
    odom_y = []
    odom_z = []
    cloud_combined = []
    saved_times = []
    start_time = bag.get_start_time()
    for msg_number, (topic, msg, time) in enumerate(bag.read_messages(topics=['/points'])):
        time = rospy.Time.from_sec(time.to_sec())
        save_time = time.to_sec() - start_time
        try:
            transform_icp = buffer.lookup_transform_full("map", time, "base_link", time, "map", rospy.Duration(1))
            icp_x.append(transform_icp.transform.translation.x)
            icp_y.append(transform_icp.transform.translation.y)
            icp_z.append(transform_icp.transform.translation.z)
            transform_imu = buffer.lookup_transform_full("odom", time, "base_link", time, "odom", rospy.Duration(1))
            odom_x.append(transform_imu.transform.translation.x)
            odom_y.append(transform_imu.transform.translation.y)
            odom_z.append(transform_imu.transform.translation.z)
            saved_times.append(save_time)
            if msg_number % 50 == 0:
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
    icp = move_coordinates_to_the_origin(np.vstack([icp_x, icp_y, icp_z]).T)
    if len(cloud_combined) > 0:
        speeds = get_speeds_one_period(icp[:, 0], icp[:, 1], icp[:, 2], saved_times)
        start_of_moving, end_of_moving = find_start_and_end_of_moving(speeds, saved_times)
        create_graph_xy_and_point_cloud(cloud_combined, icp[:, 0], icp[:, 1], odom_x, odom_y)
        create_graph_x_over_time(icp[:, 0], odom_x, saved_times)
        create_graph_y_over_time(icp[:, 1], odom_y, saved_times)
        create_graph_z_over_time(icp[:, 2], odom_z, saved_times)
        create_graph_distance_over_time(icp[:, 0], icp[:, 1], icp[:, 2], odom_x, odom_y, odom_z, saved_times,
                                        start_of_moving, end_of_moving)
        write_info_to_file(get_distances(icp[:, 0], icp[:, 1], icp[:, 2]), start_of_moving, end_of_moving, speeds)
        

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


def create_graph_xy_and_point_cloud(cloud_combined, icp_x, icp_y, imu_x, imu_y):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/shared_point_cloud.png"
    fig, ax = plt.subplots()
    marker_size = 0.5
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement")
    combined_points = np.concatenate(cloud_combined, axis=1)
    colors = transform_z_coordinates_to_color(combined_points[2, :])
    ax.scatter(combined_points[0, :], combined_points[1, :], s=marker_size, c=colors, cmap='Greens')
    ax.plot(imu_x, imu_y, color='blue', label='imu_odom')
    ax.plot(icp_x, icp_y, color='red', linestyle='--', label='icp_odom')
    plt.legend()
    plt.savefig(output_path)
    plt.close()


def create_graph_x_over_time(icp_x, imu_x, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_x_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in X direction")
    ax.plot(saved_times, move_coordinates_to_the_origin(imu_x), color='blue')
    ax.plot(saved_times, move_coordinates_to_the_origin(icp_x), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_y_over_time(icp_y, imu_y, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_y_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Y direction")
    ax.plot(saved_times, move_coordinates_to_the_origin(imu_y), color='blue')
    ax.plot(saved_times, move_coordinates_to_the_origin(icp_y), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_z_over_time(icp_z, imu_z, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/coord_z_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Z direction")
    ax.plot(saved_times, move_coordinates_to_the_origin(imu_z), color='blue')
    ax.plot(saved_times, move_coordinates_to_the_origin(icp_z), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_distance_over_time(icp_x, icp_y, icp_z, imu_x, imu_y, imu_z, saved_times, start_of_moving,
                                    end_of_moving):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/distance_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's travelled distance over time")
    ax.plot(saved_times, get_distances(imu_x, imu_y, imu_z), color='blue')
    ax.plot(saved_times, get_distances(icp_x, icp_y, icp_z), color='red', linestyle='--')
    ax.axvline(start_of_moving, color='green', linestyle=':', label='start_of_moving')
    ax.axvline(end_of_moving, color='green', linestyle='--', label='end_of_moving')
    plt.legend()
    plt.savefig(output_path)
    plt.close()


def transform_z_coordinates_to_color(coord_z):
    coord_z += abs(np.min(coord_z))
    colors = np.log1p(coord_z)
    return colors


def move_coordinates_to_the_origin(coordinates):
    return np.array(coordinates) - coordinates[0]


def get_distances(coord_x, coord_y, coord_z):
    coord_x = np.array(coord_x)
    coord_y = np.array(coord_y)
    coord_z = np.array(coord_z)
    distances_one_period_x = np.abs(coord_x[1:] - coord_x[:-1])
    distances_one_period_y = np.abs(coord_y[1:] - coord_y[:-1])
    distances_one_period_z = np.abs(coord_z[1:] - coord_z[:-1])
    distances_x = [0]
    distances_y = [0]
    distances_z = [0]
    distances = [0]
    for i in range(len(distances_one_period_x)):
        distances_x.append(distances_one_period_x[i] + distances_x[-1])
        distances_y.append(distances_one_period_y[i] + distances_y[-1])
        distances_z.append(distances_one_period_z[i] + distances_z[-1])
        distances.append(sqrt(pow(distances_x[i + 1], 2) + pow(distances_y[i + 1], 2) + pow(distances_z[i + 1], 2)))
    return distances


def write_info_to_file(distances_icp,  start_of_moving, end_of_moving, speeds):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/bag_info.txt"
    speeds = np.array(speeds)
    average_speed_icp = np.sum(speeds)/len(speeds)
    with open(output_path, "w", encoding="utf-8") as file:
        file.write(f"{distances_icp[-1]}\n{average_speed_icp}\n{start_of_moving}\n{end_of_moving}\n")


def find_start_and_end_of_moving(speeds, saved_times):
    start_of_moving = -1
    end_of_moving = -1
    for i in range(len(speeds)):
        if speeds[i] > 0.1 and start_of_moving == -1:
            start_of_moving = saved_times[i]
        elif speeds[i] < 0.1 or (i == len(speeds) - 1 and speeds[i] > 0.1):
            end_of_moving = saved_times[i]
    return start_of_moving, end_of_moving


def get_speeds_one_period(icp_x, icp_y, icp_z, saved_times):
    coord_x = np.array(icp_x)
    coord_y = np.array(icp_y)
    coord_z = np.array(icp_z)
    saved_times = np.array(saved_times)
    distances_one_period_x = np.abs(coord_x[1:] - coord_x[:-1])
    distances_one_period_y = np.abs(coord_y[1:] - coord_y[:-1])
    distances_one_period_z = np.abs(coord_z[1:] - coord_z[:-1])
    times_one_period = saved_times[1:] - saved_times[:-1]
    speeds_x = distances_one_period_x / times_one_period
    speeds_y = distances_one_period_y / times_one_period
    speeds_z = distances_one_period_z / times_one_period
    speeds = []
    for i in range(len(speeds_x)):
        speeds.append(sqrt(pow(speeds_x[i], 2) + pow(speeds_y[i], 2) + pow(speeds_z[i], 2)))
    return speeds
