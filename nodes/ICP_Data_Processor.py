import numpy as np
import rospy
from pyquaternion import Quaternion
import os


class ICPDataProcessor:
    def __init__(self, bag):
        self.bag = bag
        self.times_icp = []
        self.matrices_icp = []
        self.first_rotation_matrix_icp = None
        self.first_transform_icp = None
        self.transformed_icp = None
        self.distances_icp = None
        self.start_of_moving = None
        self.end_of_moving = None

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
        if icp is None:
            return None
        inv_matrix = np.linalg.inv(self.first_rotation_matrix_icp[:3, :3])
        coordinates = np.concatenate(icp, axis=1)
        self.transformed_icp = inv_matrix @ coordinates - np.expand_dims(inv_matrix @ coordinates[:, 0], axis=1)

    def get_icp_topic(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "/icp_odom" in topic_name:
                return topic_name
        return None

    def get_distances_icp(self):
        if self.transformed_icp is None:
            return None
        if self.distances_icp is None:
            transpose_coordinates = self.transformed_icp.T
            distances_one_period = np.abs(transpose_coordinates[1:] - transpose_coordinates[:-1])
            distances_xyz = np.concatenate((np.zeros((1, 3)), np.cumsum(distances_one_period, axis=0)), axis=0)
            self.distances_icp = np.linalg.norm(distances_xyz, axis=1)
        return self.distances_icp

    def get_average_speed(self):
        if self.transformed_icp is None:
            return None
        distances_one_period = np.abs(self.transformed_icp.T[1:] - self.transformed_icp.T[:-1])
        times_one_period = self.times_icp[1:] - self.times_icp[:-1]
        speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
        speeds = np.linalg.norm(speeds_xyz, axis=1)
        return np.sum(speeds) / len(speeds) if speeds is not None else None

    def get_start_and_end_of_moving(self):
        if self.transformed_icp is None:
            return None, None
        if self.start_of_moving is None:
            distances_one_period_xyz = np.abs(self.transformed_icp.T[1:] - self.transformed_icp.T[:-1])
            distances_one_period = np.linalg.norm(distances_one_period_xyz, axis=1)
            moving_indexes = np.where(distances_one_period > 0.002)[0]
            self.start_of_moving = self.times_icp[moving_indexes[0]]
            self.end_of_moving = self.times_icp[moving_indexes[-1]]
        return self.start_of_moving, self.end_of_moving

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

    def load_class_object(self, output_folder):
        object_ = np.load(f"{output_folder}/.icp.npz")
        self.transformed_icp = object_["coordinates"]
        self.times_icp = object_["saved_times"]
        self.first_rotation_matrix_icp = object_["first_matrix"]
        self.first_transform_icp = object_["first_transform"]
        self.matrices_icp = object_["matrices"]

    def save_class_object(self, output_folder):
        state_icp = "False"
        if self.transformed_icp is not None:
            state_icp = "True"
            np.savez(f"{output_folder}/.icp.npz", coordinates=self.transformed_icp, saved_times=self.times_icp,
                     first_matrix=self.first_rotation_matrix_icp, first_transform=self.first_transform_icp,
                     matrices=self.matrices_icp)
        is_icp_in_file = False
        if os.path.exists(f"{output_folder}/.data_availability.txt"):
            with open(f"{output_folder}/.data_availability.txt", 'r', encoding="utf-8") as file:
                lines = file.readlines()
            with open(f"{output_folder}/.data_availability.txt", 'w', encoding="utf-8") as file:
                for line in lines:
                    if line.startswith('icp'):
                        file.write(f"icp {state_icp}\n")
                        is_icp_in_file = True
                    else:
                        file.write(line)
                if not is_icp_in_file:
                    file.write(f"icp {state_icp}\n")

        else:
            with open(f"{output_folder}/.data_availability.txt", 'w', encoding="utf-8") as file:
                file.write(f"icp {state_icp}\n")
