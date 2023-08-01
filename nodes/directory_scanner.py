import pathlib
import os

class Scanner:
    def __init__(self, directory, bag_file_name):
        self.directory = directory
        self.bag_file_name = bag_file_name

    def create_output_folder(self):
        output_folder = os.path.join(self.directory, f".web_server_{self.bag_file_name}")
        if not os.path.exists(output_folder):
            os.mkdir(output_folder)
        return output_folder

    def find_loc_file(self):
        bag_file_name = self.bag_file_name
        for file in pathlib.Path(self.directory).iterdir():
            if bag_file_name.replace(".bag", "_loc.bag") in file.name:
                return file.name
        return None
