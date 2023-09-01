import pathlib
import os
import json


class DirectoryScanner:

    def __init__(self):
        self.task_lists = {}
        self.paths_to_bag_files = []

    def create_task_list(self, root_directory):
        self.find_bag_files(root_directory)
        for path_to_bag_file in self.paths_to_bag_files:
            task_list = self.check_web_folder(path_to_bag_file)
            if self.should_process_bag_file(task_list):
                self.task_lists[path_to_bag_file] = task_list
        return self.task_lists

    def find_bag_files(self, directory):
        stop_suffixes = ["_loc", "params", "no_sensors"]
        items = os.listdir(directory)
        child_directories = [os.path.join(directory, item) for item in items if
                             os.path.isdir(os.path.join(directory, item)) and ".web_server" not in item]
        for child_directory in child_directories:
            self.find_bag_files(child_directory)
        if os.path.exists(os.path.join(directory, ".ignore")):
            with open(os.path.join(directory, ".ignore"), "r", encoding="utf-8") as file:
                ignore_bag_files = [line.strip() for line in file if line.strip()]
        for file in pathlib.Path(directory).iterdir():
            if (file.is_file() and file.suffix == ".bag" and not any(suffix in file.stem for suffix in stop_suffixes)
                    and file.name not in ignore_bag_files):
                self.paths_to_bag_files.append(os.path.join(directory, file.name))

    @staticmethod
    def input_check(root_directory):
        if not (os.path.exists(root_directory) and os.path.isdir(root_directory)):
            print("This directory doesn't exist.")
            return False
        return True

    @staticmethod
    def create_web_folder(path_to_bag_file):
        directory, bag_file_name = path_to_bag_file.rsplit('/', 1)
        web_folder = os.path.join(directory, f".web_server_{bag_file_name}")
        if not os.path.exists(web_folder):
            os.mkdir(web_folder)
        return web_folder

    @staticmethod
    def find_loc_file(path_to_bag_file):
        directory, bag_file_name = path_to_bag_file.rsplit('/', 1)
        for file in pathlib.Path(directory).iterdir():
            if bag_file_name.replace(".bag", "_loc.bag") in file.name:
                return os.path.join(directory, file.name)
        return None

    @staticmethod
    def check_web_folder(path_to_bag_file):
        task_list = {"icp": False, "odom": False, "point_cloud": False, "joy": False, "video": False, "graphs": False,
                     "bag_info": False}
        directory, bag_file_name = path_to_bag_file.rsplit('/', 1)
        web_folder = os.path.join(directory, f".web_server_{bag_file_name}")
        log_file = os.path.join(web_folder, ".data_availability.txt")
        new_file_size = os.path.getsize(path_to_bag_file)
        old_file_size = -1
        if os.path.exists(os.path.join(web_folder, "bag_info.json")):
            with open(os.path.join(web_folder, "bag_info.json"), 'r') as json_file:
                old_file_size = json.load(json_file)["size"]
        if (not (os.path.exists(web_folder) and os.path.isdir(web_folder)) or not os.path.exists(log_file)
                or new_file_size != old_file_size):
            return task_list
        with open(log_file, "r", encoding="utf-8") as file:
            for line in file:
                key, value = line.split()
                task_list[key] = True if "True" in value else False
        return task_list

    @staticmethod
    def should_process_bag_file(task_list):
        return (not task_list["icp"] or not task_list["odom"] or not task_list["point_cloud"] or not
                task_list["video"]) or not task_list["joy"] or not task_list["graphs"] or not task_list["bag_info"]
