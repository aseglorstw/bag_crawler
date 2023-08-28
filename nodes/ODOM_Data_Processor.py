import numpy as np
import rospy
from pyquaternion import Quaternion
import os
from ODOM_Topic import ODOMTopic
import pathlib


class ODOMDataProcessor:

    def __init__(self, bag):
        self.bag = bag
        self.odom_topics = []

    def read_odom_topics(self):
        topic_names = list(self.get_odom_topics())
        start_time = self.bag.get_start_time()
        if topic_names is None:
            print("The topic odom was not found")
            return None
        for topic_name in topic_names:
            odom_topic = ODOMTopic()
            odom_topic.set_topic_name(topic_name)
            times = []
            odom = []
            transform_matrices = []
            for topic, msg, time in self.bag.read_messages(topics=[topic_name]):
                save_time = rospy.Time.from_sec(time.to_sec()).to_sec() - start_time
                position = msg.pose.pose.position
                orientation = msg.pose.pose.orientation
                quaternion = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
                transform_matrices.append(self.create_transform_matrix(quaternion.rotation_matrix, [position.x, position.y, position.z]))
                odom.append(np.array([[position.x], [position.y], [position.z]]))
                if odom_topic.get_first_rotation_matrix() is None:
                    odom_topic.set_first_rotation_matrix(quaternion.rotation_matrix)
                    odom_topic.set_first_transform(np.array([[position.x], [position.y], [position.z]]))
                print(f"The Coordinates from topic {topic_name} are saved. Time: {save_time}")
                times.append(save_time)
            odom_topic.set_odom(odom)
            odom_topic.set_times(np.array(times))
            odom_topic.set_transform_matrices(transform_matrices)
            self.odom_topics.append(odom_topic)

    def transform_trajectory(self):
        for odom_topic in self.odom_topics:
            odom = odom_topic.get_odom()
            if odom is None:
                return None
            inv_matrix = np.linalg.inv(odom_topic.get_first_rotation_matrix()[:3, :3])
            coordinates = np.concatenate(odom, axis=1)
            odom_topic.set_transformed_odom(inv_matrix @ coordinates - np.expand_dims(inv_matrix @ coordinates[:, 0],
                                                                                      axis=1))

    def get_odom_topics(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "Odometry" in topic_info.msg_type and "/icp_odom" not in topic_name:
                yield topic_name
        return None

    def get_transformed_coordinates(self):
        all_transformed_coordinates = dict()
        for odom_topic in self.odom_topics:
            all_transformed_coordinates[odom_topic.get_topic_name()] = odom_topic.get_transformed_odom()
        return all_transformed_coordinates["/imu_and_wheel_odom"]

    def get_distances(self):
        all_distances = dict()
        for odom_topic in self.odom_topics:
            transformed_odom = odom_topic.get_transformed_odom()
            if transformed_odom is None:
                return None
            distances = odom_topic.get_distances()
            if distances is None:
                distances_one_period_xyz = np.abs(transformed_odom.T[1:] - transformed_odom.T[:-1])
                distances_xyz = np.concatenate((np.zeros((1, 3)), np.cumsum(distances_one_period_xyz, axis=0)), axis=0)
                distances = np.linalg.norm(distances_xyz, axis=1)
            all_distances[odom_topic.get_topic_name()] = distances
        return all_distances["/imu_and_wheel_odom"]

    def get_start_and_end_of_moving(self):
        all_starts_and_ends_of_moving = dict()
        for odom_topic in self.odom_topics:
            transformed_odom = odom_topic.get_transformed_odom()
            if transformed_odom is None:
                return None
            start_of_moving, end_of_moving = odom_topic.get_start_and_end_of_moving()
            if start_of_moving is None:
                distances_one_period_xyz = np.abs(transformed_odom.T[1:] - transformed_odom.T[:-1])
                distances_one_period = np.linalg.norm(distances_one_period_xyz, axis=1)
                moving_indexes = np.where(distances_one_period > 0.002)[0]
                times = odom_topic.get_times()
                start_of_moving = times[moving_indexes[0]]
                end_of_moving = times[moving_indexes[-1]]
            all_starts_and_ends_of_moving[odom_topic.get_topic_name()] = (start_of_moving, end_of_moving)
        return all_starts_and_ends_of_moving["/imu_and_wheel_odom"]

    def get_average_speeds(self):
        all_average_speeds = dict()
        for odom_topic in self.odom_topics:
            transformed_odom = odom_topic.get_transformed_odom()
            if transformed_odom is None:
                return None
            distances_one_period = np.abs(transformed_odom.T[1:] - transformed_odom.T[:-1])
            times = odom_topic.get_times()
            times_one_period = times[1:] - times[:-1]
            speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
            speeds = np.linalg.norm(speeds_xyz, axis=1)
            all_average_speeds[odom_topic.get_topic_name()] = np.sum(speeds) / len(speeds) if speeds is not None else None
        return all_average_speeds["/imu_and_wheel_odom"]

    def get_first__matrix_odom(self):
        first_rotation_matrices = dict()
        for odom_topic in self.odom_topics:
            first_rotation_matrices[odom_topic.get_topic_name()] = odom_topic.get_first_rotation_matrix()
        return first_rotation_matrices["/imu_and_wheel_odom"]

    def get_odom_topics_objects(self):
        return self.odom_topics

    def get_first_transform_odom(self):
        all_first_transforms = dict()
        for odom_topic in self.odom_topics:
            all_first_transforms[odom_topic.get_topic_name()] = odom_topic.get_first_transform()
        return all_first_transforms["/imu_and_wheel_odom"]

    def get_times_odom(self):
        all_times = dict()
        for odom_topic in self.odom_topics:
            all_times[odom_topic.get_topic_name()] = odom_topic.get_times()
        return all_times["/imu_and_wheel_odom"]

    def get_transform_matrices_odom(self):
        all_transform_matrices = dict()
        for odom_topic in self.odom_topics:
            all_transform_matrices[odom_topic.get_topic_name()] = odom_topic.get_transform_matrices()
        return all_transform_matrices["/imu_and_wheel_odom"]

    def load_class_object(self, output_folder):
        for file in pathlib.Path(output_folder).iterdir():
            if "npz" in file.name and "icp" not in file.name and "point_cloud" not in file.name:
                object_ = np.load(f"{output_folder}/{file.name}")
                odom_topic = ODOMTopic()
                odom_topic.set_transformed_odom(object_["coordinates"])
                odom_topic.set_times(object_["saved_times"])
                odom_topic.set_first_rotation_matrix(object_["first_rotation_matrix"])
                odom_topic.set_first_transform(object_["first_transform"])
                odom_topic.set_transform_matrices(object_["matrices"])
                odom_topic.set_topic_name(file.name.replace('.', '/').replace('/npz', ''))
                self.odom_topics.append(odom_topic)

    def save_class_object(self, output_folder):
        state_odom = "False"
        for odom_topic in self.odom_topics:
            transformed_odom = odom_topic.get_transformed_odom()
            if transformed_odom is not None:
                state_odom = "True"
                np.savez(f"{output_folder}/.{self.get_name_for_npz_file(odom_topic.get_topic_name())}.npz",
                         coordinates=transformed_odom, times=odom_topic.get_times(),
                         first_rotation_matrix=odom_topic.get_first_rotation_matrix(),
                         first_transform=odom_topic.get_first_transform(),
                         transform_matrices=odom_topic.get_transform_matrices())
        is_odom_in_file = False
        if os.path.exists(f"{output_folder}/.data_availability.txt"):
            with open(f"{output_folder}/.data_availability.txt", 'r', encoding="utf-8") as file:
                lines = file.readlines()
            with open(f"{output_folder}/.data_availability.txt", 'w', encoding="utf-8") as file:
                for line in lines:
                    if line.startswith('odom'):
                        file.write(f"odom {state_odom}\n")
                        is_odom_in_file = True
                    else:
                        file.write(line)
                if not is_odom_in_file:
                    file.write(f"odom {state_odom}\n")
        else:
            with open(f"{output_folder}/.data_availability.txt", 'w', encoding="utf-8") as file:
                file.write(f"odom {state_odom}\n")

    @staticmethod
    def create_transform_matrix(rotation_matrix, translation):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation
        return transform_matrix

    @staticmethod
    def get_name_for_npz_file(topic_name):
        return topic_name.replace('/', '_')[1:]
