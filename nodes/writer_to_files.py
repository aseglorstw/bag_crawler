import yaml
import numpy as np
import datetime


class Writer:
    def __init__(self, bag, distance, start_of_moving, end_of_moving, speeds):
        self.bag = bag
        self.distance = distance
        self.start_of_moving = start_of_moving
        self.end_of_moving = end_of_moving
        self.speeds = speeds

    def write_bag_info(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/bag_info.txt"
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.Loader)
        speeds = np.array(self.speeds)
        average_speed_icp = np.sum(speeds) / len(speeds)
        with open(output_path, "w", encoding="utf-8") as file:
            file.seek(0, 2)
            file.write(f"{self.distance}\n{average_speed_icp}\n{self.start_of_moving}\n{self.end_of_moving}\n"
                    f"{self.get_date(info_dict['start'])}\n{self.get_date(info_dict['end'])}\n{info_dict['duration']}\n"
                    f"{round((info_dict['size']/pow(10, 9)), 2)}\n{info_dict['messages']}\n")

    def write_topics_info(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/topics.txt"
        topics_info = self.bag.get_type_and_topic_info()[1]
        with open(output_path, "w", encoding="utf-8") as file:
            for topic_name, topic_info in topics_info.items():
                msg_type = topic_info.msg_type
                message_count = topic_info.message_count
                frequency = topic_info.frequency
                file.write(f"{topic_name} {msg_type} {message_count} {frequency}\n")

    def get_date(self, seconds):
        return datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')