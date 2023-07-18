import rosbag
import new_topics_reader
import new_graphs_creator


def main():
    path = '/home/robert/catkin_ws/src/bag_crawler/bagfiles/husky_2022-09-27-15-01-44.bag'
    bag = rosbag.Bag(path)
    reader = new_topics_reader.Reader(bag)
    bag.close()
    reader.load_buffer()
    point_cloud = reader.read_point_cloud()
    icp, odom, saved_times = reader.read_icp_odom()
    reader.read_images_and_save_video()
    joy_control_times = reader.read_joy_topic()
    graphs_creator = new_graphs_creator.GraphsCreator(point_cloud, icp, odom, saved_times, joy_control_times)
    graphs_creator.create_graph_x_over_time()
    graphs_creator.create_graph_y_over_time()
    graphs_creator.create_graph_z_over_time()
    graphs_creator.create_graph_xy_and_point_cloud()
    graphs_creator.create_graph_distance_over_time()


if __name__ == '__main__':
    main()
