import rospy
import numpy as np
import os
import json


class JOYDataProcessor:
    def __init__(self, bag, icp, odom, elements_of_control):
        self.bag = bag
        self.icp = icp
        self.odom = odom
        self.start_time = bag.get_start_time()
        self.joy_control_coordinates = []
        self.elements_of_control = elements_of_control

    def read_joy_topic(self):
        joy_control_times = []
        topic_name = self.get_joy_topic()
        if topic_name is None:
            return None
        for topic, msg, time in self.bag.read_messages(topics=[topic_name]):
            time = rospy.Time.from_sec(time.to_sec())
            control_time = time.to_sec() - self.start_time
            print(control_time)
            joy_control_times.append(control_time)
        return np.array(joy_control_times)

    def create_joy_control_coordinates(self, joy_control_times):
        icp = self.icp.get_transformed_icp()
        odom = self.odom.get_transformed_odom_from_selected_topic()
        times_icp = self.icp.get_times_icp()
        times_odom = self.odom.get_times_odom_from_selected_topic()
        if (icp is None and odom is None) or joy_control_times is None:
            return None
        saved_times = times_icp if icp is not None else times_odom
        coordinates = icp if icp is not None else odom
        indices = np.unique(np.searchsorted(saved_times, joy_control_times))
        split_indices = np.concatenate(([-1], np.where(np.diff(indices) > 5)[0], [len(indices) - 1]))
        split_indices = [indices[split_indices[i] + 1:split_indices[i + 1] + 1] for i in range(len(split_indices) - 1)]
        for indices in split_indices:
            self.joy_control_coordinates.append(coordinates.T[indices[indices < len(saved_times)]])
        return self.joy_control_coordinates

    def get_joy_topic(self):
        topics_info = self.bag.get_type_and_topic_info()[1]
        config_topic_names = []
        if self.elements_of_control is not None:
            config_topic_names = [key for key, value in self.elements_of_control.items() if
                                  value == "robot_gamepad" or value == "gamepad_PC"]
        selected_topic_name, config_topic_name = None, None
        for topic_name, topics_info in topics_info.items():
            if "joy" in topic_name and "cmd_vel" in topic_name:
                selected_topic_name = topic_name
            elif topic_name in config_topic_names:
                config_topic_name = topic_name
        if selected_topic_name is None and config_topic_name is None:
            return None
        return config_topic_name if config_topic_names is not None else selected_topic_name

    def get_joy_control_coordinates(self):
        return self.joy_control_coordinates

    def load_class_object(self, output_folder):
        with open(f"{output_folder}/.joy.json", 'r') as json_file:
            self.joy_control_coordinates = [np.array(arr_item) for arr_item in json.load(json_file)]

    def save_class_object(self, output_folder):
        with open(f"{output_folder}/.joy.json", 'w') as file:
            json.dump([arr_item.tolist() for arr_item in self.joy_control_coordinates], file)
        with open(f"{output_folder}/.data_availability.json", "r", encoding="utf-8") as file:
            task_list = json.load(file)
        task_list["joy"] = True
        with open(f"{output_folder}/.data_availability.json", "w", encoding="utf-8") as file:
            json.dump(task_list, file, indent=4)
