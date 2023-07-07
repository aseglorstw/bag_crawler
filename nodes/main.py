import rosbag
import shared_cloud_reader
import tf_reader
import cloud_reader
import image_reader
import numpy as np


def main():
    path = '/home/robert/catkin_ws/src/bag_crawler/bagfiles/husky_2022-09-27-15-01-44.bag'
    bag = rosbag.Bag(path)
    #tf_reader.save_graph(bag)
    #cloud_reader.save_cloud(bag)
    #image_reader.save_image(bag)
    shared_cloud_reader.save_shared_cloud(bag)
    bag.close()


if __name__ == '__main__':
    main()
