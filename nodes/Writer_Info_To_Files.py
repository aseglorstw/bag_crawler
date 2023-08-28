import yaml
import os
import datetime
import numpy as np


class WriterInfo:
    def __init__(self, bag, icp, odom, folder):
        self.bag = bag
        self.folder = folder
        self.icp = icp
        self.odom = odom

    def write_bag_info(self):
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.Loader)
        distances_icp = self.icp.get_distances_icp()
        distance_icp = distances_icp[-1] if distances_icp is not None else None
        distances_odom = self.odom.get_distances_odom()
        distance_odom = distances_odom[-1] if distances_odom is not None else None
        average_speed_icp = self.icp.get_average_speed_icp()
        average_speed_odom = self.odom.get_average_speed_odom()
        start_and_end_of_moving_icp = self.icp.get_start_and_end_of_moving_icp()
        start_and_end_of_moving_odom = self.odom.get_start_and_end_of_moving_odom()
        start_of_moving = start_and_end_of_moving_icp[0] if start_and_end_of_moving_icp[0] is not None else start_and_end_of_moving_odom[0]
        end_of_moving = start_and_end_of_moving_icp[1] if start_and_end_of_moving_icp[1] is not None else start_and_end_of_moving_odom[1]
        average_speed = average_speed_icp if average_speed_icp is not None else average_speed_odom
        distance = distance_icp if distance_icp is not None else distance_odom
        with open(f"{self.folder}/bag_info.txt", "w", encoding="utf-8") as file:
            file.write(f"{distance}\n{average_speed}\n{start_of_moving}\n{end_of_moving}\n"
                       f"{self.get_date(info_dict['start'])}\n{self.get_date(info_dict['end'])}\n"
                       f"{info_dict['duration']}\n" f"{round((info_dict['size']/pow(10, 9)), 2)}\n"
                       f"{info_dict['messages']}\n")

    def write_topics_info(self):
        type_info, topics_info = self.bag.get_type_and_topic_info()
        with open(f"{self.folder}/topics_info.txt", "w", encoding="utf-8") as file:
            for topic_name, topic_info in topics_info.items():
                msg_type = topic_info.msg_type
                message_count = topic_info.message_count
                frequency = topic_info.frequency
                file.write(f"{topic_name} {msg_type} {message_count} {frequency}\n")

    @staticmethod
    def get_date(seconds):
        return datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')