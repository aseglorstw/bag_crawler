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


class Reader:

    def __init__(self, bag, loc_file):
        self.bags = [bag, loc_file] if loc_file is not None else [bag]
        self.buffer = None
        self.start_time = bag.get_start_time()
        self.topics_info = self.bags[0].get_type_and_topic_info()[1]
        rospy.init_node('tf_listener')
        self.load_buffer()
        self.data_availability = {"map": True, "odom": True, "base_link": True, "images": True, "points": True}

    def read_point_cloud(self):
        topic_name = self.find_points_topic()
        if topic_name is None:
            self.data_availability["points"] = False
            print("The topic lidar posted to was not found")
            return None
        save_interval = 20
        for msg_number, (topic, msg, time) in enumerate(self.bags[0].read_messages(topics=[topic_name])):
            if msg_number % save_interval == 0:
                time = rospy.Time.from_sec(time.to_sec())
                save_time = time.to_sec() - self.start_time
                try:
                    msg = PointCloud2(*self.slots(msg))
                    cloud = np.array(list(read_points(msg)))
                    transform_map_lidar = self.buffer.lookup_transform_full("map", time, msg.header.frame_id, time,
                                                                            "map", rospy.Duration.from_sec(0.3))
                    matrix = numpify(transform_map_lidar.transform)
                    vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                    transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4]
                    print(f"Point cloud is saved. Time: {save_time}")
                    yield transformed_vectors
                except ExtrapolationException:
                    print(f"Transformation from lidar coordinate system to map was not found. Time: {save_time}")
                except LookupException as e:
                    missing_frame = str(e).split()[0]
                    self.data_availability[missing_frame[1:-1]] = False
                    print(f"Frame {missing_frame} doesn't exist")
                    return None

    def read_icp_odom(self):
        icp = []
        odom = []
        saved_times_icp = []
        saved_times_odom = []
        rotation_matrix_icp = None
        rotation_matrix_odom = None
        is_icp_works = True
        is_odom_works = True
        topic_name = self.find_points_topic()
        if topic_name is None:
            print("The topic lidar posted to was not found")
            self.data_availability["points"] = False
            return [None] * 6
        for topic, msg, time in self.bags[0].read_messages(topics=[topic_name]):
            time = rospy.Time.from_sec(time.to_sec())
            save_time = time.to_sec() - self.start_time
            if is_icp_works:
                try:
                    transform_icp = self.buffer.lookup_transform_full("map", time, "base_link", time, "map",
                                                                      rospy.Duration.from_sec(0.3))
                    icp.append(np.array([[transform_icp.transform.translation.x], [transform_icp.transform.translation.y],
                                         [transform_icp.transform.translation.z]]))
                    if rotation_matrix_icp is None:
                        rotation_matrix_icp = transform_icp.transform
                    saved_times_icp.append(save_time)
                    print(f"The coordinates of the robot relative to the 'map' frame are saved.Time: {save_time}")
                except ExtrapolationException:
                    print(f"The coordinates of the robot relative to the 'map' frame aren't saved.Time: {save_time}")
                except LookupException as e:
                    missing_frame = str(e).split()[0]
                    is_icp_works = False
                    self.data_availability[missing_frame[1:-1]] = False
                    print(f"Frame {missing_frame} doesn't exist")
            if is_odom_works:
                try:
                    transform_odom = self.buffer.lookup_transform_full("odom", time, "base_link", time, "odom",
                                                                       rospy.Duration.from_sec(0.3))
                    odom.append(
                        np.array([[transform_odom.transform.translation.x], [transform_odom.transform.translation.y],
                                  [transform_odom.transform.translation.z]]))
                    if rotation_matrix_odom is None:
                        rotation_matrix_odom = transform_odom.transform
                    saved_times_odom.append(save_time)
                    print(f"The coordinates of the robot relative to the 'odom' frame are saved.Time: {save_time}")
                except ExtrapolationException:
                    print(f"The coordinates of the robot relative to the 'odom' frame aren't saved.Time: {save_time}")
                except LookupException as e:
                    missing_frame = str(e).split()[0]
                    is_odom_works = False
                    self.data_availability[missing_frame[1:-1]] = False
                    print(f"Frame {missing_frame} doesn't exist")

        return (np.array(icp), np.array(odom), np.array(saved_times_icp), np.array(saved_times_odom),
                rotation_matrix_icp, rotation_matrix_odom)

    def read_images_and_save_video(self, folder):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        save_interval = 5
        topic_names = list(self.find_camera_topic())
        if topic_names[0] is None:
            self.data_availability["images"] = False
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

    def find_camera_topic(self):
        for topic_name, topic_info in self.topics_info.items():
            if "CompressedImage" in topic_info.msg_type:
                yield topic_name
        return None

    @staticmethod
    def create_name_for_video(topic_name):
        return topic_name.replace('/', '_')[1:]

    def calculate_fps(self, topic_name, save_interval):
        video_duration = 20
        return self.topics_info[topic_name].message_count / (video_duration * save_interval)

    def get_datetime(self, time_from_start):
        return datetime.datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S') + "+" + \
                                                                str(datetime.timedelta(seconds=time_from_start))

    def get_data_availability(self):
        return self.data_availability

    @staticmethod
    def slots(msg):
        return [getattr(msg, var) for var in msg.__slots__]

