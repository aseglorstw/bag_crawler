import pathlib
import os
from logger import setup_logger

logger = setup_logger()

class DirectoryScanner:
    def __init__(self):
        self.task_list = []

    def create_task_list(self, root):
        self.find_bag_files(root)
        return self.task_list

    def find_bag_files(self, directory):
        stop_suffixes = ["_loc", "params", "no_sensors"]
        items = os.listdir(directory)
        folders = [os.path.join(directory, item) for item in items if
                   os.path.isdir(os.path.join(directory, item)) and ".web_server" not in item]
        for folder in folders:
            self.find_bag_files(folder)
        for file in pathlib.Path(directory).iterdir():
            if file.is_file() and file.suffix == ".bag" and not any(suffix in file.stem for suffix in stop_suffixes):
                if not self.find_web_folder(directory, file.name):
                    self.task_list.append(os.path.join(directory, file.name))

    @staticmethod
    def input_check(path):
        if not (os.path.exists(path) and os.path.isdir(path)):
            logger.error("This directory doesn't exist.")
            return False
        return True

    @staticmethod
    def create_output_folder(bag_file):
        directory, file_name = bag_file.rsplit('/', 1)
        web_folder = os.path.join(directory, f".web_server_{file_name}")
        if not os.path.exists(web_folder):
            os.mkdir(web_folder)
        return web_folder

    @staticmethod
    def find_loc_file(bag_file_name):
        directory, file_name = bag_file_name.rsplit('/', 1)
        for file in pathlib.Path(directory).iterdir():
            if file_name.replace(".bag", "_loc.bag") in file.name:
                return file.name
        return None

    @staticmethod
    def find_web_folder(directory, bag_file):
        web_folder = os.path.join(directory, f".web_server_{bag_file}")
        return os.path.exists(web_folder) and os.path.isdir(web_folder)
