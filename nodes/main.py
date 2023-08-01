import rosbag
import topics_reader
import graphs_creator
import calculator
import writer_to_files
import timeit


def main():
    path = '/home/robert/catkin_ws/src/bag_crawler/bagfiles/'
    #bag_file_name = 'husky_2022-09-27-15-01-44.bag'
    #bag_file_name = 'husky_2022-09-23-12-38-31.bag'
    bag_file_name = 'husky_2022-10-27-15-33-57.bag'
    bag = rosbag.Bag(path + bag_file_name)
    tf_odom_to_map = rosbag.Bag(path + "tf_odom_to_map.bag")

    reader = topics_reader.Reader([bag, tf_odom_to_map])
    reader.load_buffer()
    point_cloud = list(reader.read_point_cloud())
    icp, odom, saved_times = reader.read_icp_odom()
    first_matrix_icp, first_matrix_odom = reader.get_first_rotation_matrices()
    # reader.read_images_and_save_video()
    # joy_control_times = reader.read_joy_topic()

    transformed_icp = calculator.transform_trajectory(icp, first_matrix_icp)
    transformed_odom = calculator.transform_trajectory(odom, first_matrix_odom)
    transformed_point_cloud = calculator.transform_point_cloud(point_cloud, first_matrix_icp)
    distances_icp = calculator.get_distances(transformed_icp)
    distances_odom = calculator.get_distances(transformed_odom)
    speeds = calculator.get_speeds_one_period(transformed_icp, saved_times)
    #average_speed = calculator.get_average_speed(speeds)
    start_of_moving, end_of_moving = calculator.get_start_and_end_of_moving(speeds, saved_times)
    #joy_control_coordinates = calculator.get_joy_control_coordinates(transformed_icp, joy_control_times, saved_times)
    # joy_control_binary = calculator.get_joy_control_binary(saved_times, joy_control_times)
    #
    creator = graphs_creator.GraphsCreator(transformed_icp, transformed_odom, saved_times, path, bag_file_name)
    creator.create_folder()
    creator.create_graph_x_over_time()
    creator.create_graph_y_over_time()
    creator.create_graph_z_over_time()
    creator.create_graph_xy_and_point_cloud(transformed_point_cloud)
    creator.create_graph_distance_over_time(distances_icp, distances_odom, start_of_moving, end_of_moving)
    # creator.create_graph_joy_control_times_and_icp(joy_control_coordinates)
    # creator.create_binary_graph_joy_control_and_time(joy_control_binary)
    #
    # writer = writer_to_files.Writer(bag, path, bag_file_name)
    # writer.write_topics_info()
    # writer.write_bag_info(distances_icp[-1], start_of_moving, end_of_moving, average_speed)

    #bag.close()


if __name__ == '__main__':
    execution_time = timeit.timeit(main, number=1)
    print(f"Время выполнения программы: {execution_time:.6f} секунд")
