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
        self.selected_topic = None

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
                if odom_topic.get_first_matrix() is None:
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
            inv_matrix = np.linalg.inv(odom_topic.get_first_matrix()[:3, :3])
            coordinates = np.concatenate(odom, axis=1)
            odom_topic.set_transformed_odom(inv_matrix @ coordinates - np.expand_dims(inv_matrix @ coordinates[:, 0],
                                                                                      axis=1))

    def find_selected_topic(self):
        random_topic = None
        for odom_topic in self.odom_topics:
            transformed_odom = odom_topic.get_transformed_odom()
            if "gps" in odom_topic.get_topic_name() and transformed_odom is not None:
                self.selected_topic = odom_topic
                return
            elif random_topic is None and transformed_odom is not None:
                random_topic = odom_topic
        self.selected_topic = random_topic

    def get_odom_topics(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "Odometry" in topic_info.msg_type and "/icp_odom" not in topic_name:
                yield topic_name
        return None

    def get_transformed_odom_from_selected_topic(self):
        return self.selected_topic.get_transformed_odom()

    def get_distances_odom_from_selected_topic(self):
        return self.selected_topic.get_distances()

    def get_start_and_end_of_moving_odom_from_selected_topic(self):
        return self.selected_topic.get_start_and_end_of_moving()

    def get_average_speed_odom_from_selected_topic(self):
        return self.selected_topic.get_average_speed()

    def get_first_matrix_odom_from_selected_topic(self):
        return self.selected_topic.get_first_matrix()

    def get_odom_topics_objects(self):
        return self.odom_topics

    def get_first_transform_odom_from_selected_topic(self):
        return self.selected_topic.get_first_transform()

    def get_times_odom_from_selected_topic(self):
        return self.selected_topic.get_times()

    def get_transform_matrices_odom_from_selected_topic(self):
        return self.selected_topic.get_transform_matrices()

    def get_name_of_selected_topic(self):
        return self.selected_topic.get_topic_name()

    def get_max_diff_from_selected_topic(self):
        return self.selected_topic.get_max_diff()

    def get_z_coord_from_selected_topic(self):
        return self.selected_topic.get_z_coord()

    def load_class_object(self, output_folder):
        for file in pathlib.Path(output_folder).iterdir():
            if "npz" in file.name and "icp" not in file.name and "point_cloud" not in file.name:
                object_ = np.load(f"{output_folder}/{file.name}")
                odom_topic = ODOMTopic()
                odom_topic.set_transformed_odom(object_["coordinates"])
                odom_topic.set_times(object_["times"])
                odom_topic.set_first_rotation_matrix(object_["first_rotation_matrix"])
                odom_topic.set_first_transform(object_["first_transform"])
                odom_topic.set_transform_matrices(object_["transform_matrices"])
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
                         first_rotation_matrix=odom_topic.get_first_matrix(),
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
