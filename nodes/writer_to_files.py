import yaml
import numpy as np
import datetime


class Writer:
    def __init__(self, bag, path, bag_file_name):
        self.bag = bag
        self.folder = path + ".web_server_" + bag_file_name

    def write_bag_info(self, distance, start_of_moving, end_of_moving, average_speed):
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.Loader)
        with open( f"{self.folder}/bag_info.txt", "w", encoding="utf-8") as file:
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
