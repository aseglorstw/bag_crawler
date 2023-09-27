import sys
import os
import json


def main(args):
    if len(args) < 3:
        print("Enter arguments in the form <operation> <path_to_bag_file>.")
        return
    operation = args[1]
    path_to_bag_file = args[2]
    path_to_ignore_file = get_path_to_ignore_file(path_to_bag_file)
    if operation == "-add":
        add_bag_to_ignore_file(path_to_bag_file, path_to_ignore_file)
    elif operation == "-rm":
        remove_bag_from_ignore_file(path_to_bag_file, path_to_ignore_file)
    else:
        print(f"'{operation}' is unknown operation. Available operations: -add, -rm.")


def get_path_to_ignore_file(path_to_bag_file):
    directory_path, _ = os.path.split(path_to_bag_file)
    path_to_ignore_file = os.path.join(directory_path, ".ignore.json")
    return path_to_ignore_file


def add_bag_to_ignore_file(path_to_bag_file,  path_to_ignore_file):
    _, bag_file = os.path.split(path_to_bag_file)
    if not os.path.exists(path_to_ignore_file):
        with open(path_to_ignore_file, "w", encoding="utf-8") as file:
            json.dump([bag_file], file, indent=4)
    else:
        with open(path_to_ignore_file, "r", encoding="utf-8") as file:
            ignored_bag_files = json.load(file)
            if bag_file in ignored_bag_files:
                print(f"The bag file '{path_to_bag_file}' is already in '{path_to_ignore_file}'.")
                return
            ignored_bag_files.append(bag_file)
        with open(path_to_ignore_file, "w", encoding="utf-8") as file:
            json.dump(ignored_bag_files, file, indent=4)


def remove_bag_from_ignore_file(path_to_bag_file, path_to_ignore_file):
    if not os.path.exists(path_to_ignore_file):
        print(f"The file '{path_to_ignore_file}' doesn't exist.")
        return
    _, bag_file = os.path.split(path_to_bag_file)
    with open(path_to_ignore_file, "r", encoding="utf-8") as file:
        ignored_bag_files = json.load(file)
        if bag_file not in ignored_bag_files:
            print(f"The file '{path_to_bag_file}' isn't in '{path_to_ignore_file}'.")
            return
        ignored_bag_files.remove(bag_file)
    with open(path_to_ignore_file, "w", encoding="utf-8") as file:
        json.dump(ignored_bag_files, file, indent=4)


if __name__ == '__main__':
    main(sys.argv)
