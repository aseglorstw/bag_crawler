import numpy as np
import rospy
from pyquaternion import Quaternion


class ICPDataProcessor:
    def __init__(self, bag):
        self.bag = bag
        self.times_icp = []
        self.matrices_icp = []
        self.first_rotation_matrix_icp = None
        self.first_transform_icp = None
        self.transformed_icp = None
        self.speeds = None

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
        self.times_icp = np.array(self.times_icp)
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

    def get_distances_icp(self):
        if self.transformed_icp is None:
            return None
        transpose_coordinates = self.transformed_icp.T
        distances_one_period = np.abs(transpose_coordinates[1:] - transpose_coordinates[:-1])
        distances_xyz = np.concatenate((np.zeros((1, 3)), np.cumsum(distances_one_period, axis=0)), axis=0)
        distances_icp = np.linalg.norm(distances_xyz, axis=1)
        return distances_icp

    def get_speeds_icp(self):
        if self.transformed_icp is None:
            return None
        distances_one_period = np.abs(self.transformed_icp.T[1:] - self.transformed_icp.T[:-1])
        times_one_period = self.times_icp[1:] - self.times_icp[:-1]
        speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
        self.speeds = np.linalg.norm(speeds_xyz, axis=1)
        return self.speeds

    def get_average_speed(self):
        return np.sum(self.speeds) / len(self.speeds) if self.speeds is not None else None

    def get_start_and_end_of_moving(self):
        self.speeds = self.get_speeds_icp()
        if self.speeds is None:
            return None, None
        moving = np.where(self.speeds > 0.2)[0]
        if len(moving) == 0:
            return None, None
        saved_times = self.times_icp
        start_of_moving = saved_times[moving[0]]
        end_of_moving = saved_times[moving[-1] + 1] if len(moving) < len(saved_times) else saved_times[moving[-1]]
        return start_of_moving, end_of_moving

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

    def save_class_object(self):
        pass
