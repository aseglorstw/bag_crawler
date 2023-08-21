import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
from ros_numpy import numpify
import tf2_ros
from tqdm import tqdm
from rosbag import ROSBagException
import os


class PointCloudDataProcessor:

    def __init__(self, bag, icp, odom):
        self.bag = bag
        self.icp = icp
        self.odom = odom
        self.transformed_point_cloud = []

    def read_point_cloud(self):
        topic_name = self.get_points_topic()
        if topic_name is None:
            print("The topic lidar posted to was not found")
            return None
        save_interval = 20
        matrix_lidar_base_link = self.get_matrix_from_lidar_to_base_link(topic_name)
        start_time = self.bag.get_start_time()
        for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=[topic_name])):
            if msg_number % save_interval == 0:
                time = rospy.Time.from_sec(time.to_sec())
                save_time = time.to_sec() - start_time
                msg = PointCloud2(*self.slots(msg))
                cloud = np.array(list(read_points(msg)))
                vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                matrix_lidar_static_frame = self.get_matrix_from_lidar_to_static_frame(matrix_lidar_base_link, save_time)
                if matrix_lidar_static_frame is None:
                    continue
                transformed_vectors = matrix_lidar_static_frame[:3, :3] @ vectors + matrix_lidar_static_frame[:3, 3:4]
                print(f"Point cloud is saved. Time: {save_time}")
                yield transformed_vectors

    def transform_point_cloud(self, point_cloud):
        first_rotation_matrix_icp = self.icp.get_first_rotation_matrix_icp()
        first_rotation_matrix_odom = self.odom.get_first_rotation_matrix_odom()
        if len(point_cloud) == 0 or (first_rotation_matrix_odom is None and first_rotation_matrix_icp is None):
            return None
        first_matrix = first_rotation_matrix_icp if first_rotation_matrix_icp is not None else first_rotation_matrix_odom
        first_transform_icp = self.icp.get_first_transform_icp()
        first_transform_odom = self.odom.get_first_transform_odom()
        first_transform = first_transform_icp if first_transform_icp is not None else first_transform_odom
        inv_matrix = np.linalg.inv(first_matrix[:3, :3])
        point_cloud = np.concatenate(point_cloud, axis=1)
        self.transformed_point_cloud = inv_matrix @ point_cloud - inv_matrix @ first_transform
        return self.transformed_point_cloud

    def load_buffer(self):
        rospy.init_node('tf_listener')
        buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600))
        try:
            for topic, msg, time in tqdm(self.bag.read_messages(topics='/tf_static'),
                                         total=self.bag.get_message_count(topic_filters='/tf_static')):
                for tf in msg.transforms:
                    buffer.set_transform_static(tf, 'bag')
        except ROSBagException:
            print('Could not read')
        return buffer

    def get_matrix_from_lidar_to_base_link(self, topic_name):
        for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=[topic_name])):
            try:
                buffer = self.load_buffer()
                transform_base_link_lidar = buffer.lookup_transform_full("base_link", time,
                                                                         msg.header.frame_id, time, "base_link",
                                                                         rospy.Duration.from_sec(0.3))
                return numpify(transform_base_link_lidar.transform)
            except ExtrapolationException:
                print(f"Transformation from lidar coordinate system to base_link was not found.")
            except LookupException as e:
                missing_frame = str(e).split()[0]
                print(f"Frame {missing_frame} doesn't exist")
        return None

    def get_points_topic(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "/points" in topic_name or "/destagerred_points" in topic_name:
                return topic_name
        return None

    def get_matrix_from_lidar_to_static_frame(self, matrix_base_link_lidar, save_time):
        index_icp = np.unique(np.searchsorted(self.icp.get_times_icp(), save_time))[0]
        index_odom = np.unique(np.searchsorted(self.odom.get_times_odom(), save_time))[0]
        matrices_odom = self.odom.get_matrices_odom()
        matrices_icp = self.icp.get_matrices_icp()
        if 0 <= index_icp < len(matrices_icp) and len(matrices_icp) > 0:
            matrix_lidar_static_frame = matrices_icp[index_icp]
        elif len(matrices_icp) <= index_odom < len(matrices_odom) and len(matrices_odom) > 0:
            matrix_lidar_static_frame = matrices_odom[index_odom]
        else:
            return None
        return matrix_lidar_static_frame @ matrix_base_link_lidar

    def get_transformed_point_cloud(self):
        return self.transformed_point_cloud

    def save_class_object(self, output_folder):
        state_point_cloud = "False"
        if self.transformed_point_cloud is not None:
            state_point_cloud = "True"
            np.savez(f"{output_folder}/.point_cloud.npz", point_cloud=self.transformed_point_cloud)
        is_point_cloud_in_file = False
        if os.path.exists(f"{output_folder}/.data_availability.txt"):
            with open(f"{output_folder}/.data_availability.txt", 'r', encoding="utf-8") as file:
                lines = file.readlines()
            with open(f"{output_folder}/.data_availability.txt", 'w', encoding="utf-8") as file:
                for line in lines:
                    if line.startswith('point_cloud'):
                        file.write(f"point_cloud {state_point_cloud}\n")
                        is_point_cloud_in_file = True
                    else:
                        file.write(line)
                if not is_point_cloud_in_file:
                    file.write(f"point_cloud {state_point_cloud}\n")
        else:
            with open(f"{output_folder}/.data_availability.txt", 'w', encoding="utf-8") as file:
                file.write(f"point_cloud {state_point_cloud}\n")

    def load_class_object(self, output_folder):
        self.transformed_point_cloud = np.load(f"{output_folder}/.point_cloud.npz")["point_cloud"]

    @staticmethod
    def slots(msg):
        return [getattr(msg, var) for var in msg.__slots__]

