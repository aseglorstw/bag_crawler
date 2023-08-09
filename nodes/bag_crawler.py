import rosbag
import sys
import timeit
import schedule
from directory_scanner import DirectoryScanner
from topics_reader import Reader
import graphs_creator
import calculator
from writer_to_files import Writer
from logger import setup_logger

logger = setup_logger()


def main(root_directory):
    directory_scanner = DirectoryScanner()
    if not directory_scanner.input_check(root_directory):
        return 1

    task_list = directory_scanner.create_task_list(root_directory)
    for path_to_bag_file in task_list:

        bag = open_bag_file(path_to_bag_file)
        if bag is None:
            continue

        output_folder = directory_scanner.create_output_folder(path_to_bag_file)
        path_to_loc_file = directory_scanner.find_loc_file(path_to_bag_file)
        loc_file = open_bag_file(path_to_loc_file) if path_to_loc_file is not None else None

        logger.info(f"Start processing file {path_to_bag_file}")
        reader = Reader(bag, loc_file)
        icp, odom, saved_times, first_matrix_icp, first_matrix_odom = reader.read_icp_odom()
        point_cloud = list(reader.read_point_cloud())
        joy_control_times = reader.read_joy_topic()
        reader.read_images_and_save_video(output_folder)

        transformed_icp = calculator.transform_trajectory(icp, first_matrix_icp)
        transformed_odom = calculator.transform_trajectory(odom, first_matrix_odom)
        transformed_point_cloud = calculator.transform_point_cloud(point_cloud, first_matrix_icp)
        distances_icp = calculator.get_distances(transformed_icp)
        distances_odom = calculator.get_distances(transformed_odom)
        speeds = calculator.get_speeds_one_period(transformed_icp, transformed_odom, saved_times)
        average_speed = calculator.get_average_speed(speeds)
        start_of_moving, end_of_moving = calculator.get_start_and_end_of_moving(speeds, saved_times)
        joy_control_coordinates = calculator.get_joy_control_coordinates(transformed_icp, joy_control_times,
                                                                         saved_times)
        graphs_creator.create_graph_x_over_time(transformed_odom, transformed_icp, saved_times, output_folder)
        graphs_creator.create_graph_y_over_time(transformed_odom, transformed_icp, saved_times, output_folder)
        graphs_creator.create_graph_z_over_time(transformed_odom, transformed_icp, saved_times, output_folder)
        graphs_creator.create_graph_distance_over_time(distances_icp, distances_odom, saved_times, start_of_moving,
                                                       end_of_moving, output_folder)
        graphs_creator.create_graph_xy_and_point_cloud(transformed_odom, transformed_icp, transformed_point_cloud,
                                                       output_folder)
        graphs_creator.create_graph_joy_control_times_and_icp(transformed_icp, joy_control_coordinates,
                                                              output_folder)
        writer = Writer(bag, output_folder)
        writer.write_topics_info()
        writer.write_bag_info(distances_icp[-1], start_of_moving, end_of_moving, average_speed)

        close_bag_file(bag, path_to_bag_file)
        logger.info(f"Finish processing file {path_to_bag_file}")


def open_bag_file(path_to_bag_file):
    try:
        bag = rosbag.Bag(path_to_bag_file)
        return bag
    except Exception as e:
        print(f"When opening the file {path_to_bag_file}, a '{e}' error occurred")
        return None


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
