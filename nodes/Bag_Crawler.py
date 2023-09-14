import rosbag, sys, timeit, schedule
import Graphs_Creator
from Directory_Scanner import DirectoryScanner
from ICP_Data_Processor import ICPDataProcessor
from ODOM_Data_Processor import ODOMDataProcessor
from Point_Cloud_Data_Processor import PointCloudDataProcessor
from Video_Data_Processor import VideoDataProcessor
from JOY_Data_Processor import JOYDataProcessor
from BAG_Info_Data_Processor import BAGInfoDataProcessor


def main(root_directory):

    directory_scanner = DirectoryScanner()

    if not directory_scanner.input_check(root_directory):
        return 1

    task_lists = directory_scanner.get_task_list(root_directory)
    print(task_lists)
    for path_to_bag_file in task_lists.keys():
        task_list = task_lists[path_to_bag_file]

        print(path_to_bag_file)
        bag = open_bag_file(path_to_bag_file)
        if bag is None:
            continue

        path_to_web_folder = directory_scanner.get_path_to_web_folder(path_to_bag_file, task_list)
        config = directory_scanner.get_config(root_directory, path_to_bag_file)
        icp = process_icp(bag, task_list["icp"], path_to_web_folder)
        odom = process_odom(bag, task_list["odom"], path_to_web_folder)
        point_cloud = process_point_cloud(bag, icp, odom, task_list["point_cloud"], path_to_web_folder)
        joy = process_joy(bag, icp, odom, task_list["joy"], config, path_to_web_folder)
        create_graphs(icp, odom, point_cloud, joy, should_create_graphs(task_list), path_to_web_folder)
        write_bag_info_to_files(bag, icp, odom, should_write_bag_info(task_list),
                                config, path_to_web_folder)
        process_video(bag, task_list["video"], config, path_to_web_folder)

        close_bag_file(bag, path_to_bag_file)


def open_bag_file(path_to_bag_file):
    try:
        bag = rosbag.Bag(path_to_bag_file)
        return bag
    except Exception as e:
        print(f"When opening the file {path_to_bag_file}, a '{e}' error occurred")
        return None


def process_icp(bag, is_isp, output_folder):
    icp = ICPDataProcessor(bag)
    if is_isp:
        icp.load_class_object(output_folder)
        return icp
    coord_icp = icp.read_icp_topic()
    icp.transform_icp_trajectory(coord_icp)
    icp.save_class_object(output_folder)
    return icp


def process_odom(bag, is_odom, output_folder):
    odom = ODOMDataProcessor(bag)
    if is_odom:
        odom.load_class_object(output_folder)
        odom.find_selected_topic()
        return odom
    odom.read_odom_topics()
    odom.transform_trajectory()
    odom.find_selected_topic()
    odom.save_class_object(output_folder)
    return odom


def process_point_cloud(bag, icp, odom, is_point_cloud, output_folder):
    point_cloud = PointCloudDataProcessor(bag, icp, odom)
    if is_point_cloud:
        point_cloud.load_class_object(output_folder)
        return point_cloud
    bods_point_cloud = list(point_cloud.read_point_cloud())
    point_cloud.transform_point_cloud(bods_point_cloud)
    point_cloud.save_class_object(output_folder)
    return point_cloud


def process_video(bag, is_video, config, output_folder):
    if not is_video:
        video = VideoDataProcessor(bag)
        result = video.create_videos(output_folder)
        video.write_info_to_data_availability(result, output_folder)


def process_joy(bag, icp, odom, is_joy, config, output_folder):
    if "elements_of_control" in config:
        joy = JOYDataProcessor(bag, icp, odom, config["elements_of_control"])
    else:
        joy = JOYDataProcessor(bag, icp, odom, None)
    if is_joy:
        joy.load_class_object(output_folder)
        return joy
    joy_control_times = joy.read_joy_topic()
    joy.create_joy_control_coordinates(joy_control_times)
    joy.save_class_object(output_folder)
    return joy


def create_graphs(icp, odom, point_cloud, joy, are_graphs, output_folder):
    if not are_graphs:
        odom_topics_color = Graphs_Creator.match_color_odom_topic(odom.get_odom_topics_objects())
        Graphs_Creator.create_graph_x_over_time(icp.get_transformed_icp(),  icp.get_times_icp(),
                                                odom.get_odom_topics_objects(), output_folder, odom_topics_color)
        Graphs_Creator.create_graph_y_over_time(icp.get_transformed_icp(),  icp.get_times_icp(),
                                                odom.get_odom_topics_objects(), output_folder, odom_topics_color)
        Graphs_Creator.create_graph_z_over_time(icp.get_transformed_icp(),  icp.get_times_icp(),
                                                odom.get_odom_topics_objects(), output_folder, odom_topics_color)
        Graphs_Creator.create_graph_distance_over_time(icp.get_distances_icp(), icp.get_times_icp(),
                                                       icp.get_start_and_end_of_moving_icp(),
                                                       odom.get_start_and_end_of_moving_odom_from_selected_topic(),
                                                       odom.get_odom_topics_objects(), output_folder, odom_topics_color)
        Graphs_Creator.create_graph_xy_and_point_cloud(icp.get_transformed_icp(), odom.get_odom_topics_objects(),
                                                       point_cloud.get_transformed_point_cloud(), output_folder,
                                                       odom_topics_color)
        Graphs_Creator.create_graph_joy_control_times_and_icp(icp.get_transformed_icp(),
                                                              odom.get_transformed_odom_from_selected_topic(),
                                                              joy.get_joy_control_coordinates(),
                                                              odom.get_name_of_selected_topic(), joy.get_joy_topic(),
                                                              output_folder, odom_topics_color)
        Graphs_Creator.write_info_to_data_availability(output_folder)


def write_bag_info_to_files(bag, icp, odom, is_bag_info, config,  output_folder):
    if not is_bag_info:
        if "elements_of_control" in config:
            bag_info = BAGInfoDataProcessor(bag, icp, odom, config["elements_of_control"], output_folder)
        else:
            bag_info = BAGInfoDataProcessor(bag, icp, odom, None, output_folder)
        bag_info.write_bag_info()
        bag_info.write_topics_info()
        bag_info.write_moving_joints_info()
        bag_info.write_movement_tag_info()
        bag_info.write_controller_info()
        bag_info.write_info_to_data_availability(output_folder)


def should_create_graphs(task_list):
    return (task_list["icp"] and task_list["odom"] and task_list["point_cloud"] and task_list["joy"]
            and task_list["graphs"])


def should_write_bag_info(task_list):
    return (task_list["icp"] and task_list["odom"] and task_list["point_cloud"] and task_list["joy"]
            and task_list["bag_info"])


def close_bag_file(bag, path_to_bag_file):
    try:
        bag.close()
    except ValueError:
        print(f"Failed to close the file {path_to_bag_file}")


if __name__ == '__main__':
    root = sys.argv[1]
    execution_time = timeit.timeit(lambda: main(root), number=1)
    print(f"program execution time: {execution_time:.6f} seconds")
    # schedule.every(10).seconds.do(lambda: main(root))
    # while True:
    #     schedule.run_pending()

