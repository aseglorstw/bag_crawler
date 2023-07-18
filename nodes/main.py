import rosbag
import topics_reader
import image_reader
import info_topics_reader
import bag_info_reader


def main():
    path = '/home/robert/catkin_ws/src/bag_crawler/bagfiles/husky_2022-09-27-15-01-44.bag'
    bag = rosbag.Bag(path)
    #image_reader.save_video(bag)
    tf_reader.create_graphs(bag)
    #topics_reader.save_topics_info(bag)
    #topics_reader.create_graph_control_joy_and_time(bag)
    #bag_info_reader.save_bag_info(bag)
    bag.close()


if __name__ == '__main__':
    main()
