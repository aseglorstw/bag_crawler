import numpy as np
import rospy
from pyquaternion import Quaternion


class ODOMDataProcessor:

    def __init__(self, bag):
        self.bag = bag
        self.times_odom = []
        self.first_rotation_matrix_odom = None
        self.first_transform_odom = None
        self.transformed_odom = None
        self.matrices_odom = []
        self.speeds = None

    def read_odom_topic(self):
        odom = []
        topic_name = self.get_odom_topic()
        start_time = self.bag.get_start_time()
        if topic_name is None:
            print("The topic icp was not found")
            return None
        for topic, msg, time in self.bag.read_messages(topics=[topic_name]):
            save_time = rospy.Time.from_sec(time.to_sec()).to_sec() - start_time
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            quaternion = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
            self.add_matrix_to_matrices_odom(quaternion.rotation_matrix, [position.x, position.y, position.z])
            odom.append(np.array([[position.x], [position.y], [position.z]]))
            if self.first_rotation_matrix_odom is None:
                self.first_rotation_matrix_odom = quaternion.rotation_matrix
                self.first_transform_odom = np.array([[position.x], [position.y], [position.z]])
            print(f"The Coordinates from frame 'base_link' to frame 'map' are saved. Time: {save_time}")
            self.times_odom.append(save_time)
        return odom

    def add_matrix_to_matrices_odom(self, rotation_matrix, translation):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation
        self.matrices_odom.append(transform_matrix)

    def transform_odom_trajectory(self, odom):
        if len(odom) == 0:
            return None
        inv_matrix = np.linalg.inv(self.first_rotation_matrix_odom[:3, :3])
        coordinates = np.concatenate(odom, axis=1)
        self.transformed_odom = inv_matrix @ coordinates - np.expand_dims(inv_matrix @ coordinates[:, 0], axis=1)
        return self.transformed_odom

    def get_odom_topic(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "/imu_and_wheel_odom" in topic_name:
                return topic_name
        return None

    def get_distances_odom(self):
        if self.transformed_odom is None:
            return None
        transpose_coordinates = self.transformed_odom.T
        distances_one_period = np.abs(transpose_coordinates[1:] - transpose_coordinates[:-1])
        distances_xyz = np.concatenate((np.zeros((1, 3)), np.cumsum(distances_one_period, axis=0)), axis=0)
        distances_odom = np.linalg.norm(distances_xyz, axis=1)
        return distances_odom

    def get_start_and_end_of_moving(self):
        self.speeds = self.get_speeds_odom()
        if self.speeds is None:
            return None, None
        moving = np.where(self.speeds > 0.2)[0]
        if len(moving) == 0:
            return None, None
        saved_times = self.times_odom
        start_of_moving = saved_times[moving[0]]
        end_of_moving = saved_times[moving[-1] + 1] if len(moving) < len(saved_times) else saved_times[moving[-1]]
        return start_of_moving, end_of_moving

    def get_speeds_odom(self):
        if self.transformed_odom is not None:
            return None
        distances_one_period = np.abs(self.transformed_odom.T[1:] - self.transformed_odom.T[:-1])
        times_one_period = self.times_odom[1:] - self.times_odom[:-1]
        speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
        self.speeds = np.linalg.norm(speeds_xyz, axis=1)
        return self.speeds

    def get_transformed_odom(self):
        return self.transformed_odom

    def get_times_odom(self):
        return self.times_odom

    def get_matrices_odom(self):
        return self.matrices_odom

    def get_first_rotation_matrix_odom(self):
        return self.first_rotation_matrix_odom

    def get_first_transform_odom(self):
        return self.first_transform_odom

    def load_class_object(self):
        pass


