import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
import datetime
import os
from cv_bridge import CvBridge
import json
import tf2_ros
from rosbag import ROSBagException
from tqdm import tqdm
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
from ros_numpy import numpify


class VideoDataProcessor:
    def __init__(self, bag):
        self.bag = bag
        self.start_time = bag.get_start_time()
        self.demo_images = dict()
        self.buffer = None
        self.load_buffer()

    def create_videos(self, folder):
        topic_names = list(self.get_camera_topics())
        if topic_names[0] is None:
            print("The topic in which messages from the camera are posted was not found")
            return False
        for topic_name in topic_names:
            save_interval = self.get_save_interval(topic_name)
            mid_video = self.get_mid_video(topic_name)
            if "omnicam" in topic_name:
                self.save_video_omnicam(topic_name, save_interval, mid_video, folder)
            else:
                is_gray = self.get_is_gray(topic_name)
                is_depth = "depth" in topic_name
                rotation_angle = self.get_rotation_angle(topic_name)
                self.save_video(topic_name, save_interval, mid_video, is_gray, is_depth, rotation_angle, folder)
        self.save_demo_images(folder)
        return True

    def save_video(self, topic_name, save_interval, mid_video, is_gray, is_depth, rotation_angle, folder):
        upper_limit, lower_limit = None, None
        if is_depth:
            upper_limit, lower_limit = self.get_upper_and_lower_limits(topic_name, save_interval)
        fps = 60
        video_name = f"{folder}/{self.get_name_for_video(topic_name)}_video.avi"
        video_out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'MJPG'), fps, (1920, 1200), True)
        for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=[topic_name])):
            print(topic_name, save_interval, mid_video, is_gray, is_depth, rotation_angle, msg.header.frame_id)
            if msg_number == mid_video and not is_gray and not is_depth:
                msg = CompressedImage(*self.slots(msg))
                demo_image = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
                self.add_image_to_demo(demo_image, topic_name)
            if msg_number % save_interval == 0:
                time = rospy.Time.from_sec(time.to_sec())
                time_from_start = int(time.to_sec() - self.start_time)
                msg = CompressedImage(*self.slots(msg))
                image = self.get_processed_image(msg, is_depth, is_gray, upper_limit, lower_limit, rotation_angle,
                                                 time_from_start)
                cv2.imwrite(f"{folder}/{self.get_name_for_video(topic_name)}_img.png", image)
                video_out.write(image)
                print(f"Image from topic {topic_name} for video is saved. Time: {time.to_sec() - self.start_time}")
        print(f"Video  from topic {topic_name} is saved.")
        video_out.release()

    def save_video_omnicam(self, topic_name, save_interval, mid_video, folder):
        video_outs = self.get_video_outs_for_omnicam(topic_name, folder)
        for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=[topic_name])):
            if msg_number % save_interval == 0:
                time = rospy.Time.from_sec(time.to_sec())
                time_from_start = int(time.to_sec() - self.start_time)
                print(topic_name, save_interval, mid_video, msg.header.frame_id, time_from_start)
                msg = CompressedImage(*self.slots(msg))
                images = self.get_processed_images_omnicam(msg, time_from_start)
                if msg_number == mid_video:
                    demo = self.get_demo_image_omnicam(msg)
                    self.add_image_to_demo(demo, topic_name)
                for i in range(6):
                    video_outs[i].write(images[i])
        for i in range(6):
            video_outs[i].release()

    def save_demo_images(self, output_folder):
        if "demo_panorama" in self.demo_images:
            cv2.imwrite(os.path.join(output_folder, "demo_panorama.jpg"), self.demo_images["demo_panorama"])
        else:
            for key in self.demo_images.keys():
                cv2.imwrite(os.path.join(output_folder, f"{key}.jpg"), self.demo_images[key])

    def get_rotation_angle(self, topic_name):
        for topic, msg, time in self.bag.read_messages(topics=[topic_name]):
            try:
                transform_base_link_camera = self.buffer.lookup_transform_full("base_link", time,
                                                                               msg.header.frame_id, time, "base_link")
                rotate_matrix = numpify(transform_base_link_camera.transform)[:3, :3]
                # Robot HUSKY has non-standard frame layout, so I'm checking an angle between z-axes.
                angle_between_z_axes = self.get_angle_between_vectors(np.array([0, 0, 1]), rotate_matrix[:, 2])
                if 0 <= angle_between_z_axes <= 30:
                    break
                rotation_angle = self.get_angle_between_vectors(np.array([0, 0, 1]), rotate_matrix[:, 1])
                if 70 < rotation_angle < 110:
                    return -90 if self.get_is_turn_right(rotate_matrix) else 90
                elif 0 < rotation_angle < 50:
                    return -180
            except LookupException:
                break
        return 0

    def get_size_of_image(self, topic_name, is_grey):
        for topic, msg, time in self.bag.read_messages(topics=[topic_name]):
            cv_bridge = CvBridge()
            cv_image = cv_bridge.compressed_imgmsg_to_cv2(msg)
            if is_grey:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            height, width, layers = cv_image.shape
            return width, height

    def get_upper_and_lower_limits(self, topic_name, save_interval):
        depth_histogram = dict()
        for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=[topic_name])):
            if msg_number % save_interval == 0:
                msg = CompressedImage(*self.slots(msg))
                cv_bridge = CvBridge()
                image = cv_bridge.compressed_imgmsg_to_cv2(msg)
                depth_histogram = self.get_updated_depth_histogram(image, depth_histogram)
                time = rospy.Time.from_sec(time.to_sec())
                time_from_start = int(time.to_sec() - self.start_time)
                print(time_from_start)
        depths = list()
        for key, value in depth_histogram.items():
            depths.extend([key] * value)
        return np.percentile(depths, 99),  np.percentile(depths, 2)

    def get_camera_topics(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "CompressedImage" in topic_info.msg_type:
                yield topic_name
        return None

    def get_save_interval(self, topic_name):
        fps = 60
        duration_of_video = 20
        save_interval = int(self.bag.get_type_and_topic_info()[1][topic_name].message_count / (fps * duration_of_video))
        return save_interval if save_interval > 0 else 1

    def get_mid_video(self, topic_name):
        center_of_video = int(self.bag.get_type_and_topic_info()[1][topic_name].message_count / 2)
        return center_of_video

    def get_datetime(self, time_from_start):
        return datetime.datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S') + "+" + \
                                                                str(datetime.timedelta(seconds=time_from_start))

    def get_is_gray(self, topic_name):
        for topic, msg, time in self.bag.read_messages(topics=[topic_name]):
            cv_bridge = CvBridge()
            image = cv_bridge.compressed_imgmsg_to_cv2(msg)
            return image.ndim == 2 or (image.ndim == 3 and image.shape[2] == 1)

    def get_is_turn_right(self, rotate_matrix):
        angle_between_x_and_z_new_base = self.get_angle_between_vectors(np.array([1, 0, 0]), rotate_matrix[:, 2])
        if 0 < angle_between_x_and_z_new_base < 50 or 130 < angle_between_x_and_z_new_base < 180:
            angle_between_y_and_y_new_base = self.get_angle_between_vectors(np.array([0, 1, 0]), rotate_matrix[:, 1])
            return 0 < angle_between_y_and_y_new_base < 90
        else:
            angle_between_x_and_y_new_base = self.get_angle_between_vectors(np.array([1, 0, 0]), rotate_matrix[:, 1])
            return 0 < angle_between_x_and_y_new_base < 90

    def get_processed_image(self, msg, is_depth, is_gray, upper_limit, lower_limit, rotation_angle, time_from_start):
        cv_bridge = CvBridge()
        image = cv_bridge.compressed_imgmsg_to_cv2(msg)
        if is_depth:
            image = self.get_transformed_rgbd_image(image, upper_limit, lower_limit)
            image = cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_GRAY2RGB)
        elif is_gray:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        if abs(rotation_angle) > 0:
            image = self.get_rotated_image(image, rotation_angle)
        image = cv2.resize(np.asarray(image, dtype=np.uint8), (1920, 1200))
        image = cv2.putText(image, self.get_datetime(time_from_start), (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)
        return image

    @staticmethod
    def get_rotated_image(image, rotation_angle):
        if rotation_angle == 90:
            k = 1
        elif rotation_angle == -90:
            k = -1
        else:
            k = 2
        rotated_image = np.rot90(image, k=k)
        return rotated_image

    @staticmethod
    def get_name_for_video(topic_name):
        return topic_name.replace('/', '_')[1:]

    @staticmethod
    def get_angle_between_vectors(vector1, vector2):
        cos_angle = (np.dot(vector1, vector2) / np.linalg.norm(vector1) / np.linalg.norm(vector2))
        angle = np.degrees(np.arccos(cos_angle))
        return angle

    @staticmethod
    def get_updated_depth_histogram(image, depth_histogram):
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                key = int(50 * np.round(image[y][x] / 50))
                if key in depth_histogram:
                    depth_histogram[key] += 1
                else:
                    depth_histogram[key] = 1
        return depth_histogram

    @staticmethod
    def get_transformed_rgbd_image(image, upper_limit, lower_limit):
        max_value_below_percentile = np.max(image[image <= upper_limit])
        image[image > upper_limit] = max_value_below_percentile
        min_value_over_percentile = np.min(image[image >= lower_limit])
        image[image < lower_limit] = min_value_over_percentile
        image -= np.min(image)
        image = (image / np.max(image)) * 255
        return image

    def get_video_outs_for_omnicam(self, topic_name, folder):
        video_outs = []
        fps = 60
        for i in range(6):
            video_name = f"{folder}/{self.get_name_for_video(topic_name)}_video_{i}.avi"
            video_outs.append(cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'MJPG'), fps, (1920, 1200), True))
        return video_outs

    def get_processed_images_omnicam(self, msg, time_from_start):
        cv_bridge = CvBridge()
        image = cv_bridge.compressed_imgmsg_to_cv2(msg)
        images = []
        height_one_image = 1232
        rotated_angle = -90
        number_of_images = 6
        for i in range(number_of_images):
            one_image = image[height_one_image * i: height_one_image + height_one_image * i, :, :]
            one_image = self.get_rotated_image(one_image, rotated_angle)
            one_image = cv2.resize(np.asarray(one_image, dtype=np.uint8), (1920, 1200))
            one_image = cv2.putText(one_image, self.get_datetime(time_from_start), (30, 40),
                                              cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)
            images.append(one_image)
        return images

    def get_demo_image_omnicam(self, msg):
        cv_bridge = CvBridge()
        image = cv_bridge.compressed_imgmsg_to_cv2(msg)
        images = []
        height_one_image = 1232
        rotated_angle = -90
        number_of_images = 6
        for i in range(number_of_images):
            one_image = image[height_one_image * i: height_one_image + height_one_image * i, :, :]
            one_image = self.get_rotated_image(one_image, rotated_angle)
            one_image = cv2.resize(np.asarray(one_image, dtype=np.uint8), (1920, 1200))
            images.append(one_image)
        return np.hstack(images)

    def add_image_to_demo(self, image, topic_name):
        if "front" in topic_name:
            self.demo_images["demo_image_front"] = image
        elif "left" in topic_name:
            self.demo_images["demo_image_left"] = image
        elif "right" in topic_name:
            self.demo_images["demo_image_right"] = image
        elif "rear" in topic_name:
            self.demo_images["demo_image_rear"] = image
        elif "pano" or "omnicam" in topic_name:
            self.demo_images["demo_panorama"] = image

    def load_buffer(self):
        self.buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600), False)
        try:
            for topic, msg, time in tqdm(self.bag.read_messages(topics='/tf_static'),
                                         total=self.bag.get_message_count(topic_filters='/tf_static')):
                for tf in msg.transforms:
                    self.buffer.set_transform_static(tf, 'bag')
        except ROSBagException:
            print('Could not read')

    @staticmethod
    def write_info_to_data_availability(result, output_folder):
        with open(f"{output_folder}/.data_availability.json", "r", encoding="utf-8") as file:
            task_list = json.load(file)
        task_list["video"] = result
        with open(f"{output_folder}/.data_availability.json", "w", encoding="utf-8") as file:
            json.dump(task_list, file, indent=4)

    @staticmethod
    def slots(msg):
        return [getattr(msg, var) for var in msg.__slots__]
