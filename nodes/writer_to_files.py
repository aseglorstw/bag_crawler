import utils
import yaml


def write_bag_info(bag):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/bag_info.txt"
    info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.Loader)
    with open(output_path, "r+", encoding="utf-8") as file:
        file.seek(0, 2)
        file.write(f"{utils.get_date(info_dict['start'])}\n{utils.get_date(info_dict['end'])}\n{info_dict['duration']}\n"
                   f"{round((info_dict['size']/pow(10, 9)), 2)}\n{info_dict['messages']}\n")

