import rosbag
import topics_reader
import image_reader
import info_topics_reader
import bag_info_reader


def main():
    path = '/home/robert/catkin_ws/src/bag_crawler/bagfiles/husky_2022-09-27-15-01-44.bag'
    bag = rosbag.Bag(path)
    topics_reader.read_lidar_topic_and_icp_with_odom(bag)
    bag.close()


if __name__ == '__main__':
    main()
