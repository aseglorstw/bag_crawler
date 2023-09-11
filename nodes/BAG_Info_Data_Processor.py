import numpy as np
import yaml
import datetime
import rospy
import json
import os


class BAGInfoDataProcessor:

    def __init__(self, bag, icp, odom, folder):
        self.bag = bag
        self.folder = folder
        self.icp = icp
        self.odom = odom
        self.movement_joints = None

    def write_bag_info(self):
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.Loader)
        distances_icp = self.icp.get_distances_icp()
        distance_icp = distances_icp[-1] if distances_icp is not None else None
        distances_odom = self.odom.get_distances_odom_from_selected_topic()
        distance_odom = distances_odom[-1] if distances_odom is not None else None
        average_speed_icp = self.icp.get_average_speed_icp()
        average_speed_odom = self.odom.get_average_speed_odom_from_selected_topic()
        start_and_end_of_moving_icp = self.icp.get_start_and_end_of_moving_icp()
        start_and_end_of_moving_odom = self.odom.get_start_and_end_of_moving_odom_from_selected_topic()
        start_of_moving = start_and_end_of_moving_icp[0] if start_and_end_of_moving_icp[0] is not None else start_and_end_of_moving_odom[0]
        end_of_moving = start_and_end_of_moving_icp[1] if start_and_end_of_moving_icp[1] is not None else start_and_end_of_moving_odom[1]
        average_speed = average_speed_icp if average_speed_icp is not None else average_speed_odom
        distance = distance_icp if distance_icp is not None else distance_odom
        bag_info = {"distance": distance, "average_speed": average_speed, "start of moving": start_of_moving,
                    "end of moving": end_of_moving, "start": self.get_date(info_dict['start']), "end":
                    self.get_date(info_dict['end']), "duration": info_dict['duration'],
                    "size": info_dict['size'], "messages": info_dict['messages']}
        with open(f"{self.folder}/bag_info.json", "w", encoding="utf-8") as file:
            json.dump(bag_info, file, indent=4)

    def write_topics_info(self):
        type_info, topics_info = self.bag.get_type_and_topic_info()
        topics_info_dict = dict()
        with open(f"{self.folder}/topics_info.json", "w", encoding="utf-8") as file:
            for topic_name, topic_info in topics_info.items():
                max_time_delay, average_time_delay = self.calculate_max_and_average_time_delay(topic_name)
                msg_type = topic_info.msg_type
                message_count = topic_info.message_count
                topics_info_dict[topic_name] = {"msg type": msg_type, "message count": message_count,
                                                "average time delay": average_time_delay,
                                                "max time delay": max_time_delay}
            json.dump(topics_info_dict, file, indent=4)

    def write_moving_joints_info(self):
        all_joints = None
        old_coordinates = []
        moving_joints = set()
        for topic, msg, time in self.bag.read_messages(topics=["/joint_states"]):
            if all_joints is None:
                all_joints = np.array(msg.name)
                old_coordinates = msg.position
            distances = np.array(msg.position) - np.array(old_coordinates)
            moving_indexes = np.where(distances > 0.2)[0]
            old_coordinates = msg.position
            moving_joints.update(set(all_joints[moving_indexes]))
        self.movement_joints = list(moving_joints)
        with open(f"{self.folder}/moving_joints_info.json", "w", encoding="utf-8") as file:
            json.dump(self.movement_joints, file, indent=4)

    def write_controller_info(self, path_to_bag_file):
        path_to_config_file = path_to_bag_file.replace(".bag", ".config.json")
        with open(path_to_config_file, 'r') as file:
            elements_of_control = json.load(file)
        used_elements_of_control = set()
        for topic, msg, time in self.bag.read_messages():
            if topic in list(elements_of_control.keys()):
                used_elements_of_control.add(elements_of_control[topic])
        with open(f"{self.folder}/controller_info.json", "w", encoding="utf-8") as file:
            json.dump(list(used_elements_of_control), file, indent=4)

    @staticmethod
    def write_info_to_data_availability(output_folder):
        with open(f"{output_folder}/.data_availability.json", "r", encoding="utf-8") as file:
            task_list = json.load(file)
        task_list["bag_info"] = True
        with open(f"{output_folder}/.data_availability.json", "w", encoding="utf-8") as file:
            json.dump(task_list, file, indent=4)

    def write_movement_tag_info(self):
        max_diff_icp = self.icp.get_max_diff()
        max_diff_odom = self.odom.get_max_diff_from_selected_topic()
        max_diff = max_diff_icp if max_diff_icp is not None else max_diff_odom
        movement_tag = ""
        if max_diff is None or max_diff < 0.5:
            movement_tag = "stayed_in_place"
        else:
            z_icp = self.icp.get_z_coord()
            z_odom = self.odom.get_z_coord_from_selected_topic()
            z_coord = z_icp if z_icp is not None else z_odom
            z_diff_icp = abs(max(z_icp) - min(z_icp))
            z_diff_odom = abs(max(z_odom) - min(z_odom))
            z_diff = z_diff_icp if z_diff_icp is not None else z_diff_odom
            if len(self.movement_joints) == 1 and "laser_j" in self.movement_joints:
                movement_tag = "observation"
            if z_diff > 0.1:
                movement_tag += "overcame_obstacle" if movement_tag == "" else ",overcame_obstacle"
            if abs(z_coord[-1]) > 2:
                movement_tag += "changed_floor" if movement_tag == "" else ",changed_floor"
            if movement_tag == "":
                movement_tag = "general_movement"
        with open(f"{self.folder}/movement_tag.txt", "w", encoding="utf-8") as file:
            file.write(movement_tag)

    def calculate_max_and_average_time_delay(self, topic_name):
        times = []
        start_time = self.bag.get_start_time()
        for topic, msg, time in self.bag.read_messages(topics=[topic_name]):
            times.append(rospy.Time.from_sec(time.to_sec()).to_sec() - start_time)
        if len(times) > 1:
            times = np.array(times)
            time_delays = times[1:] - times[:-1]
            return round(np.max(time_delays), 3), round(np.average(time_delays), 3)
        return None, None

    @staticmethod
    def get_date(seconds):
        return datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')
