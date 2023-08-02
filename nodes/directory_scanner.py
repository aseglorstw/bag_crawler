import pathlib
import os


class Scanner:
    def __init__(self, directory):
        self.directory = directory

    @staticmethod
    def path_check(path):
        return os.path.exists(path) and os.path.isdir(path)

    def create_output_folder(self, bag_file):
        web_folder = os.path.join(self.directory, f".web_server_{bag_file}")
        if not os.path.exists(web_folder):
            os.mkdir(web_folder)
        return web_folder
    
    def create_task_list(self):
        task_list = dict()
        bag_files = list(self.find_bag_files())
        for bag_file in bag_files:
            task_list[bag_file] = self.find_web_folder(bag_file)

    def find_loc_file(self, bag_file_name):
        for file in pathlib.Path(self.directory).iterdir():
            if bag_file_name.replace(".bag", "_loc.bag") in file.name:
                return file.name
        return None

    def find_bag_files(self):
        stop_suffixes = ["full_tf", "_loc", "orig", "params", "no_sensors", "web_server"]
        for file in pathlib.Path(self.directory).iterdir():
            if file.suffix == ".bag" and not any(suffix in file.stem for suffix in stop_suffixes):
                print(file.name)
                yield file.name

    def find_web_folder(self, bag_file):
        web_folder = os.path.join(self.directory, f".web_server_{bag_file}")
        if self.path_check(web_folder):
            return None
        for file in pathlib.Path(web_folder).iterdir():
            print(file.name)
        return "a"
