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

        output_folder = directory_scanner.create_output_folder(path_to_bag_file)

        loc_file = process_loc_file(directory_scanner, path_to_bag_file)

        print(f"Start processing file {path_to_bag_file}")
        reader = Reader(bag, loc_file)

        if not task_list["video"]:
            reader.read_images_and_save_video(output_folder)
        transformed_icp, saved_times_icp, first_matrix_icp, first_transform_icp = process_icp(reader, task_list, output_folder)
        transformed_odom, saved_times_odom, first_matrix_odom, first_transform_odom = process_odom(reader, task_list, output_folder)
        transformed_point_cloud = process_point_cloud(reader, task_list, first_matrix_icp, first_transform_icp, first_matrix_odom, first_transform_odom, output_folder)
        joy_control_coordinates = process_joy_control_times(reader, transformed_icp, transformed_odom, saved_times_icp, saved_times_odom)

        distances_icp = calculator.get_distances(transformed_icp)
        distances_odom = calculator.get_distances(transformed_odom)

        speeds = calculator.get_speeds_one_period(transformed_icp, transformed_odom, saved_times_icp, saved_times_odom)
        average_speed = calculator.get_average_speed(speeds)
        start_of_moving, end_of_moving = calculator.get_start_and_end_of_moving(speeds, saved_times_icp, saved_times_odom)

        create_graphs(transformed_icp, transformed_odom, saved_times_icp, saved_times_odom, output_folder,
                      distances_icp, distances_odom, start_of_moving, end_of_moving, transformed_point_cloud,
                      joy_control_coordinates)

        write_to_files(bag, output_folder, distances_icp,  distances_odom, start_of_moving, end_of_moving,
                       average_speed, reader, transformed_odom, transformed_icp, saved_times_icp, saved_times_odom,
                       first_matrix_icp, first_matrix_odom, transformed_point_cloud, first_transform_icp,
                       first_transform_odom, task_list)

        close_bag_file(bag, path_to_bag_file)
        print(f"Finish processing file {path_to_bag_file}")


def open_bag_file(path_to_bag_file):
    try:
        bag = rosbag.Bag(path_to_bag_file)
        return bag
    except Exception as e:
        print(f"When opening the file {path_to_bag_file}, a '{e}' error occurred")
        return None


def process_loc_file(directory_scanner, path_to_bag_file):
    path_to_loc_file = directory_scanner.find_loc_file(path_to_bag_file)
    loc_file = open_bag_file(path_to_loc_file) if path_to_loc_file is not None else None
    return loc_file


def process_icp(reader, task_list, output_folder):
    if task_list["icp"]:
        data_icp = np.load(f"{output_folder}/.icp.npz")
        return data_icp["coordinates"], data_icp["saved_times"], data_icp["first_matrix"], data_icp["first_transform"]
    icp, saved_times_icp, first_matrix_icp, first_transform_icp = reader.read_icp()
    transformed_icp = calculator.transform_trajectory(icp, first_matrix_icp)
    return transformed_icp, saved_times_icp, first_matrix_icp, first_transform_icp


def process_odom(reader, task_list, output_folder):
    if task_list["odom"]:
        data_odom = np.load(f"{output_folder}/.odom.npz")
        return data_odom["coordinates"], data_odom["saved_times"], data_odom["first_matrix"], data_odom["first_transform"]
    odom, saved_times_odom, first_matrix_odom, first_transform_odom = reader.read_odom()
    transformed_odom = calculator.transform_trajectory(odom, first_matrix_odom)
    return transformed_odom, saved_times_odom, first_matrix_odom, first_transform_odom


def process_point_cloud(reader, task_list, first_matrix_icp, first_transform_icp, first_matrix_odom,
                        first_transform_odom, output_folder):
    if task_list["point_cloud"]:
        return np.load(f"{output_folder}/.point_cloud.npz")["point_cloud"]
    point_cloud = list(reader.read_point_cloud())
    transformed_point_cloud = calculator.transform_point_cloud(point_cloud, first_matrix_icp, first_transform_icp,
                                                               first_matrix_odom, first_transform_odom)
    return transformed_point_cloud


def process_joy_control_times(reader, transformed_icp, transformed_odom, saved_times_icp, saved_times_odom):
    joy_control_times = reader.read_joy_topic()
    joy_control_coordinates = calculator.get_joy_control_coordinates(transformed_icp, transformed_odom,
                                                                     joy_control_times, saved_times_icp,
                                                                     saved_times_odom)
    return joy_control_coordinates


def create_graphs(transformed_icp, transformed_odom, saved_times_icp, saved_times_odom, output_folder, distances_icp,
                  distances_odom, start_of_moving, end_of_moving, transformed_point_cloud, joy_control_coordinates):
    graphs_creator.create_graph_x_over_time(transformed_odom, transformed_icp, saved_times_odom, saved_times_icp,
                                            output_folder)
    graphs_creator.create_graph_y_over_time(transformed_odom, transformed_icp, saved_times_odom, saved_times_icp,
                                            output_folder)
    graphs_creator.create_graph_z_over_time(transformed_odom, transformed_icp, saved_times_odom, saved_times_icp,
                                            output_folder)
    graphs_creator.create_graph_distance_over_time(distances_icp, distances_odom, saved_times_icp, saved_times_odom,
                                                   start_of_moving, end_of_moving, output_folder)
    graphs_creator.create_graph_xy_and_point_cloud(transformed_odom, transformed_icp, transformed_point_cloud,
                                                   output_folder)
    graphs_creator.create_graph_joy_control_times_and_icp(transformed_icp, transformed_odom,
                                                          joy_control_coordinates, output_folder)


def write_to_files(bag, output_folder, distances_icp,  distances_odom, start_of_moving, end_of_moving, average_speed,
                   reader, transformed_odom, transformed_icp, saved_times_icp, saved_times_odom, first_matrix_icp,
                   first_matrix_odom, transformed_point_cloud, first_transform_icp, first_transform_odom, task_list):
    writer = Writer(bag, output_folder)
    full_distance_icp = distances_icp[-1] if distances_icp is not None else None
    full_distance_odom = distances_odom[-1] if distances_odom is not None else None
    writer.write_bag_info(full_distance_icp, full_distance_odom, start_of_moving, end_of_moving, average_speed)
    writer.write_topics_info()
    writer.write_info_on_data_availability(reader.get_data_availability())
    if not task_list["odom"]:
        writer.write_odom_to_file(transformed_odom, saved_times_odom, first_matrix_odom, first_transform_odom)
    if not task_list["icp"]:
        writer.write_icp_to_file(transformed_icp, saved_times_icp, first_matrix_icp, first_transform_icp)
    if not task_list["point_cloud"]:
        writer.write_point_cloud_to_file(transformed_point_cloud)


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
