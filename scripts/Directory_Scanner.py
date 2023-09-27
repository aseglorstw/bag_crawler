import pathlib
import os
import json
from datetime import datetime


class DirectoryScanner:

    def __init__(self):
        self.task_lists = {}
        self.paths_to_bag_files = []

    @staticmethod
    def input_check(root_directory):
        if not (os.path.exists(root_directory) and os.path.isdir(root_directory)):
            print("This directory doesn't exist.")
            return False
        return True

    def get_task_list(self, root_directory):
        self.get_bag_files(root_directory)
        for path_to_bag_file in self.paths_to_bag_files:
            task_list = self.check_web_folder(path_to_bag_file)
            if self.should_process_bag_file(task_list):
                self.task_lists[path_to_bag_file] = task_list
        return self.task_lists

    def get_bag_files(self, directory):
        stop_suffixes = ["_loc", "params", "no_sensors"]
        items = os.listdir(directory)
        child_directories = [os.path.join(directory, item) for item in items if
                             os.path.isdir(os.path.join(directory, item)) and ".web_server" not in item]
        for child_directory in child_directories:
            self.get_bag_files(child_directory)
        ignore_bag_files = []
        if os.path.exists(os.path.join(directory, ".ignore.json")):
            with open(os.path.join(directory, ".ignore.json"), "r", encoding="utf-8") as file:
                ignore_bag_files = json.load(file)
        for file in pathlib.Path(directory).iterdir():
            if (file.is_file() and file.suffix == ".bag" and not any(suffix in file.stem for suffix in stop_suffixes)
                    and file.name not in ignore_bag_files):
                # validate that the bag file has a corresponding format
                date_pattern = "%Y-%m-%d-%H-%M-%S"
                try:
                    datetime.strptime(file.stem.split('_')[-1], date_pattern)
                except ValueError:
                    print(f"Bag file {file.name} is not in the correct format.")
                    continue
                self.paths_to_bag_files.append(os.path.join(directory, file.name))

    @staticmethod
    def get_path_to_web_folder(path_to_bag_file, task_list):
        directory, bag_file_name = os.path.split(path_to_bag_file)
        web_folder = os.path.join(directory, f".web_server_{bag_file_name}")
        if not os.path.exists(web_folder):
            os.mkdir(web_folder)
            with open(f"{web_folder}/.data_availability.json", "w", encoding="utf-8") as file:
                json.dump(task_list, file, indent=4)
        return web_folder

    @staticmethod
    def get_loc_file(path_to_bag_file):
        directory, bag_file_name = os.path.split(path_to_bag_file)
        for file in pathlib.Path(directory).iterdir():
            if bag_file_name.replace(".bag", "_loc.bag") in file.name:
                return os.path.join(directory, file.name)
        return None

    """
    You can write a config for all bag files, which will be in the root. It is also possible to write a local 
    config file for a single folder with files, and in addition you can write a super local config, which will be 
    only for one bag.
    """
    @staticmethod
    def get_config(root_directory, path_to_bag_file):
        path_to_global_file_config = os.path.join(root_directory, ".bag_crawler_global_config.json")
        directory, _ = os.path.split(path_to_bag_file)
        path_to_local_file_config = os.path.join(directory, ".bag_crawler_local_config.json")
        path_to_super_local_file_config = path_to_bag_file.replace(".bag", ".config.json")
        if os.path.exists(path_to_super_local_file_config):
            with open(path_to_super_local_file_config, "r", encoding="utf-8") as file:
                config = json.load(file)
                return config
        elif os.path.exists(path_to_local_file_config):
            with open(path_to_local_file_config, "r", encoding="utf-8") as file:
                config = json.load(file)
                return config
        elif os.path.exists(path_to_global_file_config):
            with open(path_to_global_file_config, "r", encoding="utf-8") as file:
                config = json.load(file)
                return config
        return {}

    @staticmethod
    def check_web_folder(path_to_bag_file):
        task_list = {"icp": False, "odom": False, "point_cloud": False, "joy": False, "video": False, "graphs": False,
                     "bag_info": False}
        directory, bag_file_name = os.path.split(path_to_bag_file)
        web_folder = os.path.join(directory, f".web_server_{bag_file_name}")
        # Checking if no new data has been added to the bag file.
        new_file_size = os.path.getsize(path_to_bag_file)
        old_file_size = 0
        if os.path.exists(os.path.join(web_folder, "bag_info.json")):
            with open(os.path.join(web_folder, "bag_info.json"), 'r') as json_file:
                old_file_size = json.load(json_file)["size"]
        logger_file = os.path.join(web_folder, ".data_availability.json")
        if (not (os.path.exists(web_folder) and os.path.isdir(web_folder)) or not os.path.exists(logger_file)
                or new_file_size != old_file_size):
            return task_list
        with open(logger_file, "r", encoding="utf-8") as file:
            task_list = json.load(file)
        return task_list

    @staticmethod
    def should_process_bag_file(task_list):
        return not (task_list["icp"] and task_list["odom"] and task_list["point_cloud"] and
                    task_list["video"] and task_list["joy"] and task_list["graphs"] and task_list["bag_info"])
