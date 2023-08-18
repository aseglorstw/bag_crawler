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
        self.start_time = bag.get_start_time()
        self.topics_info = self.bags[0].get_type_and_topic_info()[1]
        self.matrices_base_link_map = []
        self.saved_times_icp = []
        self.matrices_base_link_odom = []
        self.saved_times_odom = []
        self.data_availability = {"icp": True, "odom": True, "point_cloud": True, "video": True}

    def read_images_and_save_video(self, folder):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        save_interval = 5
        topic_names = list(self.find_camera_topic())
        if topic_names[0] is None:
            self.data_availability["video"] = False
            print("The topic in which messages from the camera are posted was not found")
            return False
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
        return True

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

    def find_joy_topic(self):
        topics_info = self.bags[0].get_type_and_topic_info()[1]
        for topic_name, topics_info in topics_info.items():
            if "joy" in topic_name and "cmd_vel" in topic_name:
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

    @staticmethod
    def create_name_for_video(topic_name):
        return topic_name.replace('/', '_')[1:]

    @staticmethod
    def slots(msg):
        return [getattr(msg, var) for var in msg.__slots__]
