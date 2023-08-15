import pathlib
import os


class DirectoryScanner:
    def __init__(self):
        self.task_list = {}
        self.paths_to_bag_files = []

    def create_task_list(self, root_directory):
        self.find_bag_files(root_directory)
        for path_to_bag_file in self.paths_to_bag_files:
            task_list_for_one_bag_file = self.check_web_folder(path_to_bag_file)
            if self.should_process_bag_file(task_list_for_one_bag_file):
                self.task_list[path_to_bag_file] = task_list_for_one_bag_file
        return self.task_list

    def find_bag_files(self, directory):
        stop_suffixes = ["_loc", "params", "no_sensors"]
        items = os.listdir(directory)
        child_directories = [os.path.join(directory, item) for item in items if
                             os.path.isdir(os.path.join(directory, item)) and ".web_server" not in item]
        for child_directory in child_directories:
            self.find_bag_files(child_directory)
        for file in pathlib.Path(directory).iterdir():
            if file.is_file() and file.suffix == ".bag" and not any(suffix in file.stem for suffix in stop_suffixes):
                self.paths_to_bag_files.append(os.path.join(directory, file.name))

    @staticmethod
    def input_check(root_directory):
        if not (os.path.exists(root_directory) and os.path.isdir(root_directory)):
            print("This directory doesn't exist.")
            return False
        return True

    @staticmethod
    def create_output_folder(path_to_bag_file):
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
        task_template = {"icp": False, "odom": False, "point_cloud": False,  "video": False, "slam": False}
        directory, bag_file_name = path_to_bag_file.rsplit('/', 1)
        web_folder = os.path.join(directory, f".web_server_{bag_file_name}")
        if not (os.path.exists(web_folder) and os.path.isdir(web_folder)):
            return task_template
        log_file = os.path.join(web_folder, "data_availability.txt")
        if not os.path.exists(log_file):
            return task_template
        with open(log_file, "r", encoding="utf-8") as file:
            for line in file:
                key, value = line.split()
                task_template[key] = True if "True" in value else False
        return task_template

    def should_process_bag_file(self, task_list_for_one_bag_file):
        return (not task_list_for_one_bag_file["icp"] or not task_list_for_one_bag_file["odom"] or
                not task_list_for_one_bag_file["point_cloud"] or not task_list_for_one_bag_file["video"])
