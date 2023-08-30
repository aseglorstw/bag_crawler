import numpy as np


class ODOMTopic:

    def __init__(self):
        self.topic_name = None
        self.first_rotation_matrix = None
        self.first_transform = None
        self.odom = []
        self.transformed_odom = []
        self.times = []
        self.transform_matrices = []

    def set_odom(self, odom):
        self.odom = np.array(odom)

    def set_transformed_odom(self, transformed_odom):
        self.transformed_odom = transformed_odom

    def set_topic_name(self, topic_name):
        self.topic_name = topic_name

    def set_times(self, times):
        self.times = times

    def set_first_rotation_matrix(self, first_rotation_matrix):
        self.first_rotation_matrix = first_rotation_matrix

    def set_first_transform(self, first_transform):
        self.first_transform = first_transform

    def set_transform_matrices(self, transform_matrices):
        self.transform_matrices = transform_matrices

    def get_first_matrix(self):
        return self.first_rotation_matrix

    def get_odom(self):
        return self.odom

    def get_topic_name(self):
        return self.topic_name

    def get_transformed_odom(self):
        return self.transformed_odom

    def get_distances(self):
        if self.transformed_odom is None:
            return None
        distances_one_period_xyz = np.abs(self.transformed_odom.T[1:] - self.transformed_odom.T[:-1])
        distances_xyz = np.concatenate((np.zeros((1, 3)), np.cumsum(distances_one_period_xyz, axis=0)), axis=0)
        distances = np.linalg.norm(distances_xyz, axis=1)
        return distances

    def get_start_and_end_of_moving(self):
        if self.transformed_odom is None:
            return None
        distances_one_period_xyz = np.abs(self.transformed_odom.T[1:] - self.transformed_odom.T[:-1])
        distances_one_period = np.linalg.norm(distances_one_period_xyz, axis=1)
        moving_indexes = np.where(distances_one_period > 0.002)[0]
        start_of_moving = self.times[moving_indexes[0]]
        end_of_moving = self.times[moving_indexes[-1]]
        return start_of_moving, end_of_moving

    def get_average_speed(self):
        if self.transformed_odom is None:
            return None
        distances_one_period = np.abs(self.transformed_odom.T[1:] - self.transformed_odom.T[:-1])
        times_one_period = self.times[1:] - self.times[:-1]
        speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
        speeds = np.linalg.norm(speeds_xyz, axis=1)
        return np.sum(speeds) / len(speeds) if speeds is not None else None

    def get_transform_matrices(self):
        return self.transform_matrices

    def get_times(self):
        return self.times

    def get_first_transform(self):
        return self.first_transform
