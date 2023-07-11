import rosbag
import shared_cloud_reader
import image_reader



def main():
    path = '/home/robert/catkin_ws/src/bag_crawler/bagfiles/husky_2022-09-27-15-01-44.bag'
    bag = rosbag.Bag(path)
    #image_reader.save_image(bag)
    shared_cloud_reader.save_shared_cloud_with_graph(bag)
    bag.close()


if __name__ == '__main__':
    main()
