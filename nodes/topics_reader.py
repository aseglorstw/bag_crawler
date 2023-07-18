import utils
import graphs_creator
import writer_to_files
from sensor_msgs.msg import PointCloud2
import rospy
from sensor_msgs.point_cloud2 import read_points
import numpy as np
from tf2_ros import ExtrapolationException
from ros_numpy import numpify


def read_lidar_topic_and_icp_with_odom(bag):
    rospy.init_node('tf_listener')
    buffer = utils.load_buffer(bag)
    icp = []
    odom = []
    cloud_combined = []
    saved_times = []
    first_transform_lidar = []
    start_time = bag.get_start_time()
    for msg_number, (topic, msg, time) in enumerate(bag.read_messages(topics=['/points'])):
        time = rospy.Time.from_sec(time.to_sec())
        save_time = time.to_sec() - start_time
        try:
            transform_icp = buffer.lookup_transform_full("map", time, "base_link", time, "map", rospy.Duration(1))
            icp.append([transform_icp.transform.translation.x, transform_icp.transform.translation.y,
                        transform_icp.transform.translation.z])
            transform_imu = buffer.lookup_transform_full("odom", time, "base_link", time, "odom", rospy.Duration(1))
            odom.append([transform_imu.transform.translation.x, transform_imu.transform.translation.y,
                         transform_imu.transform.translation.z])
            saved_times.append(save_time)
            if msg_number % 20 == 0:
                msg = PointCloud2(*utils.slots(msg))
                cloud = np.array(list(read_points(msg)))
                transform_map_lidar = buffer.lookup_transform_full("map", time, msg.header.frame_id, time,
                                                                   "map", rospy.Duration(1))
                if len(first_transform_lidar) == 0:
                    first_transform_lidar = np.array([transform_map_lidar.transform.translation.x,
                            transform_map_lidar.transform.translation.y, transform_map_lidar.transform.translation.z])
                matrix = numpify(transform_map_lidar.transform)
                vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4] - first_transform_lidar.reshape(3, 1)
                cloud_combined.append(transformed_vectors)
        except ExtrapolationException:
            continue
    if len(cloud_combined) > 0:
        icp = utils.move_coordinates_to_the_origin(icp)
        odom = utils.move_coordinates_to_the_origin(odom)
        speeds = utils.get_speeds_one_period(icp, saved_times)
        start_of_moving, end_of_moving = utils.find_start_and_end_of_moving(speeds, saved_times)
        graphs_creator.create_graph_xy_and_point_cloud(cloud_combined, icp, odom)
        graphs_creator.create_graph_x_over_time(icp[:, 0], odom[:, 0], saved_times)
        graphs_creator.create_graph_y_over_time(icp[:, 1], odom[:, 1], saved_times)
        graphs_creator.create_graph_z_over_time(icp[:, 2], odom[:, 2], saved_times)
        graphs_creator.create_graph_distance_over_time(icp, odom,  saved_times, start_of_moving, end_of_moving)
        writer_to_files.write_bag_info(bag, utils.get_distances(icp), start_of_moving, end_of_moving, speeds)
        

def read_joy_topic_and_icp(bag):
    saved_times = []
    start_time = bag.get_start_time()
    joy_name = utils.find_joy_topic(bag)
    if joy_name != -1:
        for topic, msg, time in bag.read_messages(topics=[joy_name]):
            time = rospy.Time.from_sec(time.to_sec())
            save_time = int(time.to_sec() - start_time)
            if save_time not in saved_times:
                saved_times.append(save_time)
        time_array = utils.create_time_array(bag)
        control_joy = utils.create_array_of_binary_control_joy(time_array, saved_times)
    else:
        print("Topic joy not founded")

