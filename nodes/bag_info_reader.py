from rosbag.bag import Bag
import yaml


def save_bag_info(bag):
    info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.Loader)
    print(f"{info_dict['start']} {info_dict['end']} {info_dict['duration']} {info_dict['size']}")
