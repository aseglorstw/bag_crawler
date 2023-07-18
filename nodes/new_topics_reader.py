import rospy
import tf2_ros
from rosbag import ROSBagException
from tqdm import tqdm
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
import numpy as np
from ros_numpy import numpify
from tf2_ros import ExtrapolationException


class Reader:

    def __init__(self, bag):
        self.bag = bag
        self.icp = []
        self.odom = []
        self.point_cloud = []
        self.saved_times = []
        self.buffer = []
        self.start_time = bag.get_start_time()
        rospy.init_node('tf_listener')

    def read_point_cloud(self):
        first_transform = []
        for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=['/points'])):
            if msg_number % 20 == 0:
                try:
                    msg = PointCloud2(*self.slots(msg))
                    cloud = np.array(list(read_points(msg)))
                    transform_map_lidar = self.buffer.lookup_transform_full("map", time, msg.header.frame_id, time,
                                                                            "map", rospy.Duration(1))
                    if len(first_transform) == 0:
                        first_transform = np.array([transform_map_lidar.transform.translation.x,
                                                    transform_map_lidar.transform.translation.y,
                                                    transform_map_lidar.transform.translation.z])
                    matrix = numpify(transform_map_lidar.transform)
                    vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                    transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4] - first_transform.reshape(3, 1)
                    self.point_cloud.append(transformed_vectors)
                except ExtrapolationException:
                    continue

    def read_icp_odom(self):
        for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=['/points'])):
            time = rospy.Time.from_sec(time.to_sec())
            save_time = time.to_sec() - self.start_time
            try:
                transform_icp = self.buffer.lookup_transform_full("map", time, "base_link", time, "map", rospy.Duration(1))
                self.icp.append([transform_icp.transform.translation.x, transform_icp.transform.translation.y,
                            transform_icp.transform.translation.z])
                transform_imu = self.buffer.lookup_transform_full("odom", time, "base_link", time, "odom", rospy.Duration(1))
                self.odom.append([transform_imu.transform.translation.x, transform_imu.transform.translation.y,
                             transform_imu.transform.translation.z])
                self.saved_times.append(save_time)
            except ExtrapolationException:
                continue

    def load_buffer(self):
        tf_topics = ['/tf', '/tf_static', 'points']
        self.buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600))
        try:
            for topic, msg, stamp in tqdm(self.bag.read_messages(topics=tf_topics),
                                          total=self.bag.get_message_count(topic_filters=tf_topics)):
                if topic == '/tf':
                    for tf in msg.transforms:
                        self.buffer.set_transform(tf, 'bag')
                elif topic == '/tf_static':
                    for tf in msg.transforms:
                        self.buffer.set_transform_static(tf, 'bag')
        except ROSBagException:
            print('Could not read')

    def slots(self, msg):
        return [getattr(msg, var) for var in msg.__slots__]
