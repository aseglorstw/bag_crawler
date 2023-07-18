import rosbag
import writer_to_files
import topics_reader


def main():
    path = '/home/robert/catkin_ws/src/bag_crawler/bagfiles/husky_2022-09-27-15-01-44.bag'
    bag = rosbag.Bag(path)
    # topics_reader.read_lidar_topic_and_icp_with_odom(bag)
    # topics_reader.read_image_topic(bag)
    # topics_reader.read_joy_topic_and_icp(bag)
    writer_to_files.write_topics_info(bag)
    bag.close()


if __name__ == '__main__':
    main()
