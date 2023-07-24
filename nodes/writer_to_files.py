import yaml
import numpy as np
import datetime


class Writer:
    def __init__(self, bag):
        self.bag = bag

    def write_bag_info(self, distance, start_of_moving, end_of_moving, speeds):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/bag_info.txt"
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.Loader)
        speeds = np.array(speeds)
        average_speed_icp = np.sum(speeds) / len(speeds)
        with open(output_path, "w", encoding="utf-8") as file:
            file.seek(0, 2)
            file.write(f"{distance}\n{average_speed_icp}\n{start_of_moving}\n{end_of_moving}\n"
                       f"{self.get_date(info_dict['start'])}\n{self.get_date(info_dict['end'])}\n"
                       f"{info_dict['duration']}\n" f"{round((info_dict['size']/pow(10, 9)), 2)}\n"
                       f"{info_dict['messages']}\n")

    def write_topics_info(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/topics.txt"
        type_info, topics_info = self.bag.get_type_and_topic_info()
        with open(output_path, "w", encoding="utf-8") as file:
            for topic_name, topic_info in topics_info.items():
                msg_type = topic_info.msg_type
                message_count = topic_info.message_count
                frequency = topic_info.frequency
                file.write(f"{topic_name} {msg_type} {message_count} {frequency}\n")

    def get_date(self, seconds):
        return datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')
