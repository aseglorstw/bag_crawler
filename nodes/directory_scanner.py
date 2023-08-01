import pathlib


class Scanner:
    def __init__(self, directory):
        self.directory = directory

    def find_loc_file(self, bag_file_name):
        for file in pathlib.Path(self.directory).iterdir():
            if bag_file_name.replace(".bag", "_loc.bag") in file.name:
                return file.name
        return None
