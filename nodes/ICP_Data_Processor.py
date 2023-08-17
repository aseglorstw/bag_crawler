import numpy as np
import rospy
from pyquaternion import Quaternion


class ICPDataProcessor:
    def __init__(self, bag):
        self.bag = bag
        self.times_icp = []
        self.first_rotation_matrix_icp = None
        self.first_transform_icp = None
        self.transformed_icp = None
        self.matrices_icp = []

    def read_icp_topic(self):
        icp = []
        topic_name = self.get_icp_topic()
        start_time = self.bag.get_start_time()
        if topic_name is None:
            print("The topic icp was not found")
            return None
        for topic, msg, time in self.bag.read_messages(topics=[topic_name]):
            save_time = rospy.Time.from_sec(time.to_sec()).to_sec() - start_time
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            quaternion = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
            self.add_matrix_to_matrices_icp(quaternion.rotation_matrix, [position.x, position.y, position.z])
            icp.append(np.array([[position.x], [position.y], [position.z]]))
            if self.first_rotation_matrix_icp is None:
                self.first_rotation_matrix_icp = quaternion.rotation_matrix
                self.first_transform_icp = np.array([[position.x], [position.y], [position.z]])
            print(f"The Coordinates from frame 'base_link' to frame 'map' are saved. Time: {save_time}")
            self.times_icp.append(save_time)
        return icp

    def add_matrix_to_matrices_icp(self, rotation_matrix, translation):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation
        self.matrices_icp.append(transform_matrix)

    def transform_icp_trajectory(self, icp):
        if len(icp) == 0:
            return None
        inv_matrix = np.linalg.inv(self.first_rotation_matrix_icp[:3, :3])
        coordinates = np.concatenate(icp, axis=1)
        self.transformed_icp = inv_matrix @ coordinates - np.expand_dims(inv_matrix @ coordinates[:, 0], axis=1)
        return self.transformed_icp

    def get_icp_topic(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "/icp_odom" in topic_name:
                return topic_name
        return None

    def get_times_icp(self):
        return self.times_icp

    def get_transformed_icp(self):
        return self.transformed_icp

    def get_first_rotation_matrix_icp(self):
        return self.first_rotation_matrix_icp

    def get_first_transform_icp(self):
        return self.first_transform_icp

    def get_matrices_icp(self):
        return self.matrices_icp

    def load_class_object(self):
        pass
