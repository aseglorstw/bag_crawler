import rosbag
import sys
import timeit
import schedule
from directory_scanner import DirectoryScanner
from topics_reader import Reader
import graphs_creator
import calculator
from writer_to_files import Writer
import numpy as np
from ICP_Data_Processor import ICPDataProcessor
from ODOM_Data_Processor import ODOMDataProcessor
from Point_Cloud_Data_Processor import PointCloudDataProcessor


def main(root_directory):
    directory_scanner = DirectoryScanner()
    if not directory_scanner.input_check(root_directory):
        return 1

    task_lists = directory_scanner.create_task_list(root_directory)
    print(task_lists)
    for path_to_bag_file in task_lists.keys():
        task_list = task_lists[path_to_bag_file]

        bag = open_bag_file(path_to_bag_file)
        if bag is None:
            continue

        path_to_web_folder = directory_scanner.create_web_folder(path_to_bag_file)

        print(f"Start processing file {path_to_bag_file}")
        icp = process_icp(bag, task_list["icp"])
        odom = process_odom(bag, task_list["odom"])
        point_cloud = process_point_cloud(bag, icp, odom)


        close_bag_file(bag, path_to_bag_file)
        print(f"Finish processing file {path_to_bag_file}")


def open_bag_file(path_to_bag_file):
    try:
        bag = rosbag.Bag(path_to_bag_file)
        return bag
    except Exception as e:
        print(f"When opening the file {path_to_bag_file}, a '{e}' error occurred")
        return None


def process_icp(bag, is_isp):
    icp = ICPDataProcessor(bag)
    if is_isp:
        icp.load_class_object()
        return icp
    coordinates_icp = icp.read_icp_topic()
    icp.transform_icp_trajectory(coordinates_icp)
    return icp


def process_odom(bag, is_odom):
    odom = ODOMDataProcessor(bag)
    if is_odom:
        odom.load_class_object()
        return odom
    coordinates_odom = odom.read_odom_topic()
    odom.transform_odom_trajectory(coordinates_odom)
    return odom


def process_point_cloud(bag, icp, odom):
    point_cloud = PointCloudDataProcessor(bag, icp, odom)
    bods_point_cloud = list(point_cloud.read_point_cloud())
    point_cloud.transform_point_cloud(bods_point_cloud)
    return point_cloud


def close_bag_file(bag, path_to_bag_file):
    try:
        bag.close()
    except ValueError:
        print(f"Failed to close the file {path_to_bag_file}")


if __name__ == '__main__':
    root = sys.argv[1]
    execution_time = timeit.timeit(lambda: main(root), number=1)
    print(f"Время выполнения программы: {execution_time:.6f} секунд")
    # schedule.every(10).seconds.do(lambda: main(root))
    # while True:
    #     schedule.run_pending()
