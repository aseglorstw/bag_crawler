import numpy as np
import rospy
from pyquaternion import Quaternion
import json


class ICPDataProcessor:
    def __init__(self, bag):
        self.bag = bag
        self.times = []
        self.transform_matrices = []
        self.first_rotation_matrix = None
        self.first_transform = None
        self.transformed_icp = None
        self.distances = None
        self.start_of_moving = None
        self.end_of_moving = None
        self.max_diff = None

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
            # Then I'll use these matrices to get the lidar coordinates.
            self.add_transform_matrix(quaternion.rotation_matrix, [position.x, position.y, position.z])
            icp.append(np.array([[position.x], [position.y], [position.z]]))
            # Then I'll use these data for transformation of trajectory.
            if self.first_rotation_matrix is None:
                self.first_rotation_matrix = quaternion.rotation_matrix
                self.first_transform = np.array([[position.x], [position.y], [position.z]])
            print(f"The Coordinates from topic /icp_odom. Time: {save_time}")
            self.times.append(save_time)
        self.times = np.array(self.times)
        return icp

    def add_transform_matrix(self, rotation_matrix, translation):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation
        self.transform_matrices.append(transform_matrix)

    def transform_icp_trajectory(self, icp):
        if icp is None:
            return
        elif np.linalg.det(self.first_rotation_matrix[:3, :3]):
            coordinates = np.concatenate(icp, axis=1)
            self.transformed_icp = coordinates - coordinates[:, 0].reshape((3, 1))
            return
        # Rotation matrix, shows how the coordinates of the robot are rotated with respect to base_link.
        inv_matrix = np.linalg.inv(self.first_rotation_matrix[:3, :3])
        coordinates = np.concatenate(icp, axis=1)
        # Multiply by the inverse of the first rotation matrix and subtract the first coordinate from the entire array.
        self.transformed_icp = inv_matrix @ coordinates - np.expand_dims(inv_matrix @ coordinates[:, 0], axis=1)

    def get_icp_topic(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "/icp_odom" in topic_name:
                return topic_name
        return None

    def get_distances_icp(self):
        if self.transformed_icp is None:
            return None
        if self.distances is None:
            transpose_coordinates = self.transformed_icp.T
            distances_one_period = np.abs(transpose_coordinates[1:] - transpose_coordinates[:-1])
            distances_xyz = np.concatenate((np.zeros((1, 3)), np.cumsum(distances_one_period, axis=0)), axis=0)
            self.distances = np.linalg.norm(distances_xyz, axis=1)
        return self.distances

    def get_average_speed_icp(self):
        if self.transformed_icp is None:
            return None
        elif self.start_of_moving is None:
            return 0
        average_speed = self.distances[-1] / (self.end_of_moving - self.start_of_moving)
        return average_speed

    def get_start_and_end_of_moving_icp(self):
        if self.transformed_icp is None:
            return None, None
        if self.start_of_moving is None:
            distances_one_period_xyz = np.abs(self.transformed_icp.T[1:] - self.transformed_icp.T[:-1])
            distances_one_period = np.linalg.norm(distances_one_period_xyz, axis=1)
            moving_indexes = np.where(distances_one_period > 0.002)[0]
            if len(moving_indexes) == 0:
                return None, None
            self.start_of_moving = self.times[moving_indexes[0]]
            self.end_of_moving = self.times[moving_indexes[-1]]
        return self.start_of_moving, self.end_of_moving

    def get_max_diff(self):
        if self.transformed_icp is None:
            return None
        if self.max_diff is None:
            x_diff = abs(max(self.transformed_icp[0, :]) - min(self.transformed_icp[0, :]))
            y_diff = abs(max(self.transformed_icp[1, :]) - min(self.transformed_icp[1, :]))
            z_diff = abs(max(self.transformed_icp[2, :]) - min(self.transformed_icp[2, :]))
            self.max_diff = max(x_diff, y_diff, z_diff)
        return self.max_diff

    def get_z_coord(self):
        if self.transformed_icp is None:
            return None
        return self.transformed_icp[2, :]

    def get_times_icp(self):
        return self.times

    def get_transformed_icp(self):
        return self.transformed_icp

    def get_first_matrix_icp(self):
        return self.first_rotation_matrix

    def get_first_transform_icp(self):
        return self.first_transform

    def get_transform_matrices_icp(self):
        return self.transform_matrices

    def load_class_object(self, output_folder):
        object_ = np.load(f"{output_folder}/.icp.npz")
        self.transformed_icp = object_["coordinates"]
        self.times = object_["times"]
        self.first_rotation_matrix = object_["first_matrix"]
        self.first_transform = object_["first_transform"]
        self.transform_matrices = object_["transform_matrices"]

    def save_class_object(self, output_folder):
        if self.transformed_icp is not None:
            np.savez(f"{output_folder}/.icp.npz", coordinates=self.transformed_icp, times=self.times,
                     first_matrix=self.first_rotation_matrix, first_transform=self.first_transform,
                     transform_matrices=self.transform_matrices)
        with open(f"{output_folder}/.data_availability.json", "r", encoding="utf-8") as file:
            task_list = json.load(file)
        task_list["icp"] = self.transformed_icp is not None
        with open(f"{output_folder}/.data_availability.json", "w", encoding="utf-8") as file:
            json.dump(task_list, file, indent=4)
