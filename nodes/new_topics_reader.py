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
import yaml


class Reader:

    def __init__(self, bag):
        self.bag = bag
        self.buffer = []
        self.start_time = bag.get_start_time()
        rospy.init_node('tf_listener')

    def read_point_cloud(self):
        first_transform = []
        point_cloud = []
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
                    point_cloud.append(transformed_vectors)
                except ExtrapolationException:
                    continue
        return point_cloud

    def read_icp_odom(self):
        icp = []
        odom = []
        saved_times = []
        for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=['/points'])):
            time = rospy.Time.from_sec(time.to_sec())
            save_time = time.to_sec() - self.start_time
            try:
                transform_icp = self.buffer.lookup_transform_full("map", time, "base_link", time, "map",
                                                                  rospy.Duration(1))
                icp.append([transform_icp.transform.translation.x, transform_icp.transform.translation.y,
                                 transform_icp.transform.translation.z])
                transform_imu = self.buffer.lookup_transform_full("odom", time, "base_link", time, "odom",
                                                                  rospy.Duration(1))
                odom.append([transform_imu.transform.translation.x, transform_imu.transform.translation.y,
                                  transform_imu.transform.translation.z])
                saved_times.append(save_time)
            except ExtrapolationException:
                continue
        return icp, odom, saved_times

    def read_images_and_save_video(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/video.avi"
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        fps = self.calculate_fps()
        video_out = cv2.VideoWriter(output_path, fourcc, fps, (1920, 1200), True)
        for msg_number, (topic, msg, time) in enumerate(
                self.bag.read_messages(topics=['/camera_front/image_color/compressed'])):
            time = rospy.Time.from_sec(time.to_sec())
            time_from_start = int(time.to_sec() - self.start_time)
            msg = CompressedImage(*self.slots(msg))
            np_arr = np.fromstring(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            current_datetime = datetime.datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S') + "+" + \
                                                                str(datetime.timedelta(seconds=time_from_start))
            cv2.putText(image, current_datetime, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
            video_out.write(image)
        video_out.release()


    def read_joy_topic(self):
        joy_control_times = []
        joy_name = self.find_joy_topic()
        if joy_name != -1:
            for topic, msg, time in self.bag.read_messages(topics=[joy_name]):
                time = rospy.Time.from_sec(time.to_sec())
                control_time = int(time.to_sec() - self.start_time)
                if control_time not in joy_control_times:
                    joy_control_times.append(control_time)
        else:
            print("Topic joy not founded")


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

    def calculate_fps(self):
        video_duration = 20
        return self.bag.get_type_and_topic_info()[1]['/camera_front/image_color/compressed'].message_count / \
            video_duration

    def find_joy_topic(self):
        topics_info = self.bag.get_type_and_topic_info()[1]
        for topic_name, topics_info in topics_info.items():
            if "joy" in topic_name:
                return topic_name
        return -1
