import rospy
import tf2_ros
from rosbag import ROSBagException
from tqdm import tqdm
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
import numpy as np
from ros_numpy import numpify
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
import cv2
import datetime
from sensor_msgs.msg import CompressedImage
from pyquaternion import Quaternion


class Reader:

    def __init__(self, bag, loc_file):
        self.bags = [bag, loc_file] if loc_file is not None else [bag]
        rospy.init_node('tf_listener')
        self.buffer = None
        self.load_buffer()
        self.start_time = bag.get_start_time()
        self.topics_info = self.bags[0].get_type_and_topic_info()[1]
        self.matrices_base_link_map = []
        self.saved_times_icp = []
        self.matrices_base_link_odom = []
        self.saved_times_odom = []
        self.data_availability = {"icp": True, "odom": True, "point_cloud": True, "video": True}

    def read_point_cloud(self):
        topic_name = self.find_points_topic()
        if topic_name is None:
            self.data_availability["point_cloud"] = False
            print("The topic lidar posted to was not found")
            return None
        save_interval = 20
        matrix_lidar_base_link = self.get_matrix_from_lidar_to_base_link(topic_name)
        for msg_number, (topic, msg, time) in enumerate(self.bags[0].read_messages(topics=[topic_name])):
            if msg_number % save_interval == 0:
                time = rospy.Time.from_sec(time.to_sec())
                save_time = time.to_sec() - self.start_time
                msg = PointCloud2(*self.slots(msg))
                cloud = np.array(list(read_points(msg)))
                vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                matrix_lidar_static_frame = self.get_matrix_from_lidar_to_static_frame(matrix_lidar_base_link, save_time)
                if matrix_lidar_static_frame is None:
                    continue
                transformed_vectors = matrix_lidar_static_frame[:3, :3] @ vectors + matrix_lidar_static_frame[:3, 3:4]
                print(f"Point cloud is saved. Time: {save_time}")
                yield transformed_vectors

    def read_odom(self):
        odom = []
        first_rotation_matrix_odom = None
        first_transform_odom = None
        topic_name = self.find_odom_topic()
        if topic_name is None:
            print("The topic imu_odom was not found")
            self.data_availability["odom"] = False
            return np.array(odom), np.array(self.saved_times_odom), first_rotation_matrix_odom
        for topic, msg, time in self.bags[0].read_messages(topics=[topic_name]):
            time = rospy.Time.from_sec(time.to_sec())
            save_time = time.to_sec() - self.start_time
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            quaternion = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
            self.add_matrix_to_matrices_base_link_static_frame(quaternion.rotation_matrix,
                                                               [position.x, position.y, position.z],
                                                               self.matrices_base_link_odom)
            odom.append(np.array([[position.x], [position.y], [position.z]]))
            if first_rotation_matrix_odom is None:
                first_rotation_matrix_odom = quaternion.rotation_matrix
                first_transform_odom = np.array([[position.x], [position.y], [position.z]])
            print(f"The Coordinates from frame 'base_link' to frame 'odom' are saved. Time: {save_time}")
            self.saved_times_odom.append(save_time)
        return np.array(odom), np.array(self.saved_times_odom), first_rotation_matrix_odom, first_transform_odom

    def read_icp(self):
        icp = []
        first_rotation_matrix_icp = None
        first_transform_icp = None
        topic_name = self.find_icp_topic()
        if topic_name is None:
            print("The topic icp_odom was not found")
            self.data_availability["icp"] = False
            return np.array(icp), np.array(self.saved_times_icp), first_rotation_matrix_icp, first_transform_icp
        for topic, msg, time in self.bags[0].read_messages(topics=[topic_name]):
            save_time = rospy.Time.from_sec(time.to_sec()).to_sec() - self.start_time
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            quaternion = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
            self.add_matrix_to_matrices_base_link_static_frame(quaternion.rotation_matrix,
                                                               [position.x, position.y, position.z],
                                                               self.matrices_base_link_map)
            icp.append(np.array([[position.x], [position.y], [position.z]]))
            if first_rotation_matrix_icp is None:
                first_rotation_matrix_icp = quaternion.rotation_matrix
                first_transform_icp = np.array([[position.x], [position.y], [position.z]])
            print(f"The Coordinates from frame 'base_link' to frame 'map' are saved. Time: {save_time}")
            self.saved_times_icp.append(save_time)
        return np.array(icp), np.array(self.saved_times_icp), first_rotation_matrix_icp, first_transform_icp

    def read_images_and_save_video(self, folder):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        save_interval = 5
        topic_names = list(self.find_camera_topic())
        if topic_names[0] is None:
            self.data_availability["video"] = False
            print("The topic in which messages from the camera are posted was not found")
            return None
        for topic_name in topic_names:
            fps = self.calculate_fps(topic_name, save_interval)
            video_name = f"{folder}/{self.create_name_for_video(topic_name)}_video.avi"
            video_out = cv2.VideoWriter(video_name, fourcc, fps, (1920, 1200), True)
            for msg_number, (topic, msg, time) in enumerate(self.bags[0].read_messages(topics=[topic_name])):
                if msg_number % save_interval == 0:
                    time = rospy.Time.from_sec(time.to_sec())
                    time_from_start = int(time.to_sec() - self.start_time)
                    msg = CompressedImage(*self.slots(msg))
                    np_arr = np.fromstring(msg.data, np.uint8)
                    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    current_datetime = self.get_datetime(time_from_start)
                    cv2.putText(image, current_datetime, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
                    video_out.write(image)
                    print(f"Image from topic {topic_name} for video is saved. Time: {time.to_sec() - self.start_time}")
            print(f"Video  from topic {topic_name} is saved.")
            video_out.release()

    def read_joy_topic(self):
        joy_control_times = []
        topic_name = self.find_joy_topic()
        if topic_name is None:
            return None
        for topic, msg, time in self.bags[0].read_messages(topics=[topic_name]):
            time = rospy.Time.from_sec(time.to_sec())
            control_time = time.to_sec() - self.start_time
            joy_control_times.append(control_time)
        return np.array(joy_control_times)

    def load_buffer(self):
        self.buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600))
        for bag in self.bags:
            try:
                for topic, msg, time in tqdm(bag.read_messages(topics='/tf_static'),
                                             total=bag.get_message_count(topic_filters='/tf_static')):
                    for tf in msg.transforms:
                        self.buffer.set_transform_static(tf, 'bag')
            except ROSBagException:
                print('Could not read')

    def find_joy_topic(self):
        topics_info = self.bags[0].get_type_and_topic_info()[1]
        for topic_name, topics_info in topics_info.items():
            if "joy" in topic_name and "cmd_vel" in topic_name:
                return topic_name
        return None

    def find_points_topic(self):
        for topic_name, topic_info in self.topics_info.items():
            if "/points" in topic_name or "/destagerred_points" in topic_name:
                return topic_name
        return None

    def find_odom_topic(self):
        for topic_name, topic_info in self.topics_info.items():
            if "/imu_and_wheel_odom" in topic_name:
                return topic_name
        return None

    def find_icp_topic(self):
        for topic_name, topic_info in self.topics_info.items():
            if "/icp_odom" in topic_name:
                return topic_name
        return None

    def find_camera_topic(self):
        for topic_name, topic_info in self.topics_info.items():
            if "CompressedImage" in topic_info.msg_type:
                yield topic_name
        return None

    def calculate_fps(self, topic_name, save_interval):
        video_duration = 20
        return self.topics_info[topic_name].message_count / (video_duration * save_interval)

    def get_datetime(self, time_from_start):
        return datetime.datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S') + "+" + \
                                                                str(datetime.timedelta(seconds=time_from_start))

    def get_data_availability(self):
        return self.data_availability

    def get_matrix_from_lidar_to_static_frame(self, matrix_base_link_lidar, save_time):
        index_icp = np.unique(np.searchsorted(self.saved_times_icp, save_time))[0]
        index_odom = np.unique(np.searchsorted(self.saved_times_odom, save_time))[0]
        if 0 <= index_icp < len(self.matrices_base_link_map) and len(self.matrices_base_link_map) > 0:
            matrix_lidar_static_frame = self.matrices_base_link_map[index_icp]
        elif (0 <= index_odom < len(self.matrices_base_link_odom) and len(self.matrices_base_link_odom) > 0 and
              not self.data_availability["icp"]):
            matrix_lidar_static_frame = self.matrices_base_link_odom[index_odom]
        else:
            return None
        return matrix_lidar_static_frame @ matrix_base_link_lidar

    def get_matrix_from_lidar_to_base_link(self, topic_name):
        for msg_number, (topic, msg, time) in enumerate(self.bags[0].read_messages(topics=[topic_name])):
            try:
                transform_base_link_lidar = self.buffer.lookup_transform_full("base_link", time,
                                                                              msg.header.frame_id,
                                                                              time,
                                                                              "base_link",
                                                                              rospy.Duration.from_sec(0.3))
                return numpify(transform_base_link_lidar.transform)
            except ExtrapolationException:
                print(f"Transformation from lidar coordinate system to base_link was not found.")
            except LookupException as e:
                missing_frame = str(e).split()[0]
                print(f"Frame {missing_frame} doesn't exist")
        return None

    @staticmethod
    def add_matrix_to_matrices_base_link_static_frame(rotation_matrix, translation,
                                                      matrices_base_link_static_frame):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation
        matrices_base_link_static_frame.append(transform_matrix)

    @staticmethod
    def create_name_for_video(topic_name):
        return topic_name.replace('/', '_')[1:]

    @staticmethod
    def slots(msg):
        return [getattr(msg, var) for var in msg.__slots__]
