import numpy as np


class ODOMTopic:
    def __init__(self):
        self.topic_name = None
        self.first_rotation_matrix = None
        self.first_transform = None
        self.distances = None
        self.start_of_moving = None
        self.end_of_moving = None
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

    def get_first_rotation_matrix(self):
        return self.first_rotation_matrix

    def get_odom(self):
        return self.odom

    def get_topic_name(self):
        return self.topic_name

    def get_transformed_odom(self):
        return self.transformed_odom

    def get_distances(self):
        return self.distances

    def get_start_and_end_of_moving(self):
        return self.start_of_moving, self.end_of_moving

    def get_transform_matrices(self):
        return self.transform_matrices

    def get_times(self):
        return self.times

    def get_first_transform(self):
        return self.first_transform
