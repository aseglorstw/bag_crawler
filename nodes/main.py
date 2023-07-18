import rosbag
import writer_to_files
import topics_reader
import new_topics_reader


def main():
    path = '/home/robert/catkin_ws/src/bag_crawler/bagfiles/husky_2022-09-27-15-01-44.bag'
    bag = rosbag.Bag(path)
    reader = new_topics_reader.Reader(bag)
    reader.load_buffer()
    reader.read_point_cloud()
    reader.read_icp_odom()
    bag.close()


if __name__ == '__main__':
    main()
