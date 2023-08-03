import rospy
import tf2_ros
from rosbag import ROSBagException
from tqdm import tqdm
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
import numpy as np
from ros_numpy import numpify
from tf2_ros import ExtrapolationException
import cv2
import datetime
from sensor_msgs.msg import CompressedImage


class Reader:

    def __init__(self, bags):
        self.bags = bags
        self.buffer = None
        self.start_time = bags[0].get_start_time()
        self.rotation_matrix_icp = None
        self.rotation_matrix_odom = None
        self.topics_info = self.bags[0].get_type_and_topic_info()[1]
        rospy.init_node('tf_listener')
        self.load_buffer()

    def read_point_cloud(self):
        topic_name = self.find_points_topic()
        if topic_name is None:
            return None
        for msg_number, (topic, msg, time) in enumerate(self.bags[0].read_messages(topics=[topic_name])):
            if msg_number % 20 == 0:
                try:
                    msg = PointCloud2(*self.slots(msg))
                    cloud = np.array(list(read_points(msg)))
                    transform_map_lidar = self.buffer.lookup_transform_full("map", time, msg.header.frame_id, time,
                                                                            "map", rospy.Duration(1))
                    matrix = numpify(transform_map_lidar.transform)
                    vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                    transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4]
                    yield transformed_vectors
                except ExtrapolationException:
                    continue

    def read_icp_odom(self):
        icp = []
        odom = []
        saved_times = []
        topic_name = self.find_topic_for_icp_and_odom()
        if topic_name is None:
            return icp, odom, saved_times
        origin_icp, robot_center = self.find_transformation_from_base_link_to_map()
        origin_odom, robot_center = self.find_transformation_from_base_link_to_odom()
        for topic, msg, time in self.bags[0].read_messages(topics=[topic_name]):
            time = rospy.Time.from_sec(time.to_sec())
            save_time = time.to_sec() - self.start_time
            try:
                if origin_icp is not None and robot_center is not None:
                    transform_icp = self.buffer.lookup_transform_full(origin_icp, time, robot_center, time, origin_icp,
                                                                      rospy.Duration(1))
                    icp.append(np.array([[transform_icp.transform.translation.x], [transform_icp.transform.translation.y],
                               [transform_icp.transform.translation.z]]))
                    if self.rotation_matrix_icp is None:
                        self.rotation_matrix_icp = transform_icp.transform
                if origin_odom is not None and robot_center is not None:
                    transform_odom = self.buffer.lookup_transform_full(origin_odom, time, robot_center, time, origin_odom,
                                                                       rospy.Duration(1))
                    odom.append(np.array([[transform_odom.transform.translation.x], [transform_odom.transform.translation.y],
                                [transform_odom.transform.translation.z]]))
                    if self.rotation_matrix_odom is None:
                        self.rotation_matrix_odom = transform_odom.transform
                saved_times.append(save_time)
            except ExtrapolationException:
                continue
        return np.array(icp), np.array(odom), np.array(saved_times)

    def read_images_and_save_video(self, folder):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        fps = self.calculate_fps()
        video_out = cv2.VideoWriter(f"{folder}/video.avi", fourcc, fps, (1920, 1200), True)
        topic_name = self.find_camera_topic()
        if topic_name is None:
            return None
        for msg_number, (topic, msg, time) in enumerate(
                self.bags[0].read_messages(topics=[topic_name])):
            if msg_number % 5 == 0:
                time = rospy.Time.from_sec(time.to_sec())
                time_from_start = int(time.to_sec() - self.start_time)
                msg = CompressedImage(*self.slots(msg))
                np_arr = np.fromstring(msg.data, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                current_datetime = self.get_datetime(time_from_start)
                cv2.putText(image, current_datetime, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
                video_out.write(image)
        video_out.release()

    def read_joy_topic(self):
        joy_control_times = []
        topic_name = self.find_joy_topic()
        if topic_name is None:
            return None
        for topic, msg, time in self.bags[0].read_messages(topics=[topic_name]):
            time = rospy.Time.from_sec(time.to_sec())
            control_time = time.to_sec() - self.start_time
            if control_time not in joy_control_times:
                joy_control_times.append(control_time)
        return np.array(joy_control_times)

    def load_buffer(self):
        tf_topics = ['/tf', '/tf_static', 'points']
        self.buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600))
        for bag in self.bags:
            try:
                for topic, msg, time in tqdm(bag.read_messages(topics=tf_topics),
                                             total=bag.get_message_count(topic_filters=tf_topics)):
                    for tf in msg.transforms:
                        if topic == '/tf':
                            self.buffer.set_transform(tf, 'bag')
                        elif topic == '/tf_static':
                            self.buffer.set_transform_static(tf, 'bag')
            except ROSBagException:
                print('Could not read')

    def find_joy_topic(self):
        for topic_name, topics_info in self.topics_info.items():
            if "cmd_vel" in topic_name and "joy" in topic_name:
                return topic_name
        return None

    def find_points_topic(self):
        for topic_name, topics_info in self.topics_info.items():
            if "points" in topic_name:
                return topic_name
        print("The topic lidar posted to was not found")
        return None

    def find_camera_topic(self):
        for topic_name, topics_info in self.topics_info.items():
            if "image" in topic_name:
                return topic_name
        print("The topic in which messages from the camera are posted was not found")
        return None

    def find_transformation_from_base_link_to_map(self):
        return "map", "base_link"

    def find_transformation_from_base_link_to_odom(self):
        return "odom", "base_link"

    def find_topic_for_icp_and_odom(self):
        return "/points"

    def calculate_fps(self):
        video_duration = 20
        return self.bags[0].get_type_and_topic_info()[1]['/camera_front/image_color/compressed'].message_count / \
            (video_duration * 5)

    def get_datetime(self, time_from_start):
        return datetime.datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S') + "+" + \
                                                                str(datetime.timedelta(seconds=time_from_start))

    def get_first_rotation_matrices(self):
        return self.rotation_matrix_icp, self.rotation_matrix_odom

    @staticmethod
    def slots(msg):
        return [getattr(msg, var) for var in msg.__slots__]
