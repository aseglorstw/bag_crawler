import yaml
import os
import datetime


class Writer:
    def __init__(self, bag, folder):
        self.bag = bag
        self.folder = folder
        self.data_availability = {"topics": False, "info": False}

    def write_bag_info(self, distance_icp, distance_odom, start_of_moving, end_of_moving, average_speed):
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.Loader)
        distance = distance_icp if distance_icp is not None else distance_odom
        with open(f"{self.folder}/bag_info.txt", "w", encoding="utf-8") as file:
            file.write(f"{distance}\n{average_speed}\n{start_of_moving}\n{end_of_moving}\n"
                       f"{self.get_date(info_dict['start'])}\n{self.get_date(info_dict['end'])}\n"
                       f"{info_dict['duration']}\n" f"{round((info_dict['size']/pow(10, 9)), 2)}\n"
                       f"{info_dict['messages']}\n")
            self.data_availability["info"] = True

    def write_topics_info(self):
        type_info, topics_info = self.bag.get_type_and_topic_info()
        with open(f"{self.folder}/topics_info.txt", "w", encoding="utf-8") as file:
            for topic_name, topic_info in topics_info.items():
                msg_type = topic_info.msg_type
                message_count = topic_info.message_count
                frequency = topic_info.frequency
                file.write(f"{topic_name} {msg_type} {message_count} {frequency}\n")
                self.data_availability["topics"] = True

    def write_info_on_data_availability(self, data_availability):
        self.data_availability.update(data_availability)
        with open(f"{self.folder}/data_availability.txt", "w", encoding="utf-8") as file:
            for key, value in self.data_availability.items():
                file.write(f"{key} {value}\n")

    @staticmethod
    def get_date(seconds):
        return datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')
