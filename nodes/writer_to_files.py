import yaml
import os
import datetime
import numpy as np

class Writer:
    def __init__(self, bag, folder):
        self.bag = bag
        self.folder = folder

    def write_bag_info(self, distance_icp, distance_odom, start_of_moving, end_of_moving, average_speed):
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.Loader)
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

    def write_video_to_file(self, is_video):
        with open(f"{self.folder}/.data_availability.txt", "w", encoding="utf-8") as file:
            if is_video:
                file.write('video True\n')
            else:
                file.write('video False\n')

    def write_odom_to_file(self, odom, saved_times_odom, matrix_odom, first_transform_odom, matrices_odom):
        with open(f"{self.folder}/.data_availability.txt", 'a', encoding="utf-8") as file:
            if odom is not None:
                file.write('odom True\n')
                np.savez(f"{self.folder}/.odom.npz", coordinates=odom, saved_times=saved_times_odom, first_matrix=matrix_odom,
                         first_transform=first_transform_odom, matrices=matrices_odom)
            else:
                file.write('odom False\n')

    def write_icp_to_file(self, icp, saved_times_icp, matrix_icp, first_transform_icp, matrices_icp):
        with open(f"{self.folder}/.data_availability.txt", 'a', encoding="utf-8") as file:
            if icp is not None:
                file.write('icp True\n')
                np.savez(f"{self.folder}/.icp.npz", coordinates=icp, saved_times=saved_times_icp, first_matrix=matrix_icp,
                         first_transform=first_transform_icp, matrices=matrices_icp)
            else:
                file.write('icp False\n')

    def write_point_cloud_to_file(self, point_cloud):
        with open(f"{self.folder}/.data_availability.txt", 'a', encoding="utf-8") as file:
            if point_cloud is not None:
                file.write('point_cloud True\n')
                np.savez(f"{self.folder}/.point_cloud.npz", point_cloud=point_cloud)
            else:
                file.write('point_cloud False\n')

    @staticmethod
    def get_date(seconds):
        return datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')
