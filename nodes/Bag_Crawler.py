import rosbag, sys, timeit, schedule
import Graphs_Creator
from Directory_Scanner import DirectoryScanner
from ICP_Data_Processor import ICPDataProcessor
from ODOM_Data_Processor import ODOMDataProcessor
from Point_Cloud_Data_Processor import PointCloudDataProcessor
from Video_Data_Processor import VideoDataProcessor
from JOY_Data_Processor import JOYDataProcessor
from Writer_Info_To_Files import WriterInfo


def main(root_directory):
    directory_scanner = DirectoryScanner()

    if not directory_scanner.input_check(root_directory):
        return 1

    task_lists = directory_scanner.create_task_list(root_directory)
    print(task_lists)
    c = 0
    for path_to_bag_file in task_lists.keys():
        if c == 0:
            c += 1
            continue

        task_list = task_lists[path_to_bag_file]
        bag = open_bag_file(path_to_bag_file)
        if bag is None:
            continue

        path_to_web_folder = directory_scanner.create_web_folder(path_to_bag_file)

        icp = process_icp(bag, task_list["icp"], path_to_web_folder)
        odom = process_odom(bag, task_list["odom"], path_to_web_folder)
        point_cloud = process_point_cloud(bag, icp, odom, task_list["point_cloud"], path_to_web_folder)
        joy = process_joy(bag, icp, odom)
        create_graphs(icp, odom, point_cloud, joy, path_to_web_folder)
        write_info_to_files(bag, icp, odom, path_to_web_folder)
        process_video(bag, task_list["video"], path_to_web_folder)

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
    coordinates_icp = icp.read_icp_topic()
    icp.transform_icp_trajectory(coordinates_icp)
    icp.save_class_object(output_folder)
    return icp


def process_odom(bag, is_odom, output_folder):
    odom = ODOMDataProcessor(bag)
    if is_odom:
        odom.load_class_object(output_folder)
        return odom
    odom.read_odom_topics()
    odom.transform_trajectory()
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


def process_video(bag, is_video, output_folder):
    video = VideoDataProcessor(bag)
    if not is_video:
        result = video.read_images_and_save_video(output_folder)
        video.write_result_to_file(result, output_folder)


def process_joy(bag, icp, odom):
    joy = JOYDataProcessor(bag, icp, odom)
    joy_control_times = joy.read_joy_topic()
    joy.create_joy_control_coordinates(joy_control_times)
    return joy


def create_graphs(icp, odom, point_cloud, joy, output_folder):
    Graphs_Creator.create_graph_x_over_time(odom.get_transformed_odom(), icp.get_transformed_icp(),
                                            odom.get_times_odom(),
                                            icp.get_times_icp(), output_folder)
    Graphs_Creator.create_graph_y_over_time(odom.get_transformed_odom(), icp.get_transformed_icp(),
                                            odom.get_times_odom(),
                                            icp.get_times_icp(), output_folder)
    Graphs_Creator.create_graph_z_over_time(odom.get_transformed_odom(), icp.get_transformed_icp(),
                                            odom.get_times_odom(),
                                            icp.get_times_icp(), output_folder)
    Graphs_Creator.create_graph_distance_over_time(icp.get_distances_icp(), odom.get_distances_odom(),
                                                   icp.get_times_icp(),
                                                   odom.get_times_odom(), icp.get_start_and_end_of_moving_icp(),
                                                   odom.get_start_and_end_of_moving_odom(), output_folder)
    Graphs_Creator.create_graph_xy_and_point_cloud(odom.get_transformed_odom(), icp.get_transformed_icp(),
                                                   point_cloud.get_transformed_point_cloud(), output_folder)
    Graphs_Creator.create_graph_joy_control_times_and_icp(icp.get_transformed_icp(), odom.get_transformed_odom(),
                                                          joy.get_joy_control_coordinates(), output_folder)


def write_info_to_files(bag, icp, odom, output_folder):
    writer = WriterInfo(bag, icp, odom, output_folder)
    writer.write_bag_info()
    writer.write_topics_info()


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
