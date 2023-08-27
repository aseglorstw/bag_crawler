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
        self.matrices = []

    def add_matrix_to_matrices(self, rotation_matrix, translation):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation
        self.matrices.append(transform_matrix)

    def set_odom(self, odom):
        self.odom = odom

    def set_topic_name(self, topic_name):
        self.topic_name = topic_name

    def set_times(self, times):
        self.times = times

    def set_first_rotation_matrix(self, first_rotation_matrix):
        self.first_rotation_matrix = first_rotation_matrix

    def set_first_transform(self, first_transform):
        self.first_transform = first_transform

    def get_first_rotation_matrix(self):
        return self.first_rotation_matrix
