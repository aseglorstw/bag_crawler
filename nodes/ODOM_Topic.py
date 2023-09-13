import numpy as np


class ODOMTopic:

    def __init__(self):
        self.topic_name = None
        self.first_rotation_matrix = None
        self.first_transform = None
        self.start_of_moving = None
        self.end_of_moving = None
        self.distances = None
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
        self.distances = np.linalg.norm(distances_xyz, axis=1)
        return self.distances

    def get_start_and_end_of_moving(self):
        if self.transformed_odom is None:
            return None, None
        distances_one_period_xyz = np.abs(self.transformed_odom.T[1:] - self.transformed_odom.T[:-1])
        distances_one_period = np.linalg.norm(distances_one_period_xyz, axis=1)
        moving_indexes = np.where(distances_one_period > 0.002)[0]
        if len(moving_indexes) == 0:
            return None, None
        self.start_of_moving = self.times[moving_indexes[0]]
        self.end_of_moving = self.times[moving_indexes[-1]]
        return self.start_of_moving, self.end_of_moving

    def get_average_speed(self):
        if self.transformed_odom is None:
            return None
        elif self.start_of_moving is None:
            return 0
        average_speed = self.distances[-1] / (self.end_of_moving - self.start_of_moving)
        return average_speed

    def get_max_diff(self):
        if self.transformed_odom is None:
            return None
        x_diff = abs(max(self.transformed_odom[0, :]) - min(self.transformed_odom[0, :]))
        y_diff = abs(max(self.transformed_odom[1, :]) - min(self.transformed_odom[1, :]))
        z_diff = abs(max(self.transformed_odom[2, :]) - min(self.transformed_odom[2, :]))
        return max(x_diff, y_diff, z_diff)

    def get_z_coord(self):
        if self.transformed_odom is None:
            return None
        return self.transformed_odom[2, :]

    def get_transform_matrices(self):
        return self.transform_matrices

    def get_times(self):
        return self.times

    def get_first_transform(self):
        return self.first_transform
