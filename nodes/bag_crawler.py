import rosbag
import os
import sys
from directory_scanner import Scanner
from topics_reader import Reader
from graphs_creator import Creator
import calculator
from writer_to_files import Writer
import timeit


def main(directory):
    scanner = Scanner(directory)
    scanner.input_check(directory)
    task_list = list(scanner.create_task_list())
    print(task_list)
    for bag_file in task_list:
        bag = open_bag_file(directory, bag_file)
        if bag is None:
            continue
        output_folder = scanner.create_output_folder(bag_file)
        loc_file_name = scanner.find_loc_file(bag_file)
        if loc_file_name is None:
            reader = Reader([bag])
        else:
            loc_file = rosbag.Bag(os.path.join(directory, loc_file_name))
            reader = Reader([bag, loc_file])
        reader.load_buffer()
        point_cloud = list(reader.read_point_cloud())
        icp, odom, saved_times = reader.read_icp_odom()
        first_matrix_icp, first_matrix_odom = reader.get_first_rotation_matrices()
        reader.read_images_and_save_video(output_folder)
        joy_control_times = reader.read_joy_topic()

        transformed_icp = calculator.transform_trajectory(icp, first_matrix_icp)
        transformed_odom = calculator.transform_trajectory(odom, first_matrix_odom)
        transformed_point_cloud = calculator.transform_point_cloud(point_cloud, first_matrix_icp)
        distances_icp = calculator.get_distances(transformed_icp)
        distances_odom = calculator.get_distances(transformed_odom)
        speeds = calculator.get_speeds_one_period(transformed_icp, saved_times)
        average_speed = calculator.get_average_speed(speeds)
        start_of_moving, end_of_moving = calculator.get_start_and_end_of_moving(speeds, saved_times)
        joy_control_coordinates = calculator.get_joy_control_coordinates(transformed_icp, joy_control_times, saved_times)

        creator = Creator(transformed_icp, transformed_odom, saved_times, output_folder)
        creator.create_graph_x_over_time()
        creator.create_graph_y_over_time()
        creator.create_graph_z_over_time()
        creator.create_graph_xy_and_point_cloud(transformed_point_cloud)
        creator.create_graph_distance_over_time(distances_icp, distances_odom, start_of_moving, end_of_moving)
        creator.create_graph_joy_control_times_and_icp(joy_control_coordinates)

        writer = Writer(bag, output_folder)
        writer.write_topics_info()
        writer.write_bag_info(distances_icp[-1], start_of_moving, end_of_moving, average_speed)

        bag.close()


def open_bag_file(directory, bag_file):
    try:
        bag = rosbag.Bag(os.path.join(directory, bag_file))
        return bag
    except rosbag.ROSBagException as e:
        print(f"When opening the file {bag_file}, a {e} error occurred")


def close_bag_file(bag, bag_file):
    bag.close()
    if not bag.is_closed():
        print(f"Failed to close the file {bag_file}")


if __name__ == '__main__':
    arg1 = sys.argv[1]
    execution_time = timeit.timeit(lambda: main(arg1), number=1)
    print(f"Время выполнения программы: {execution_time:.6f} секунд")
