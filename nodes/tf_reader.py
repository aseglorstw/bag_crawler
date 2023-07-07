import tf2_ros
import rospy
from rosbag import ROSBagException
from tqdm import tqdm
import matplotlib.pyplot as plt 


"""
We read data from the tf and tf static topics into the buffer for an hour, then every 5 seconds we get the coordinates 
of the robot's center relative to the start of the robot's movement. We expect the transformation within a second and 
add the coordinates to the chart, if it is not there, then we process the next 5 seconds.
"""


def save_graph(bag):
    coordinates_x = []
    coordinates_y = []
    rospy.init_node('tf_listener')
    buffer = load_buffer(bag)
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/graphs/tf_graph.png"
    for topic, msg, time in bag.read_messages(topics=['/camera_front/image_color/compressed']):
        time = rospy.Time.from_sec(time.to_sec())
        time_sec = time.to_sec()
        start_time = bag.get_start_time()
        time_sec = time_sec - start_time
        if int(time_sec) % 5 == 0:
            try:
                transform = buffer.lookup_transform_full("map", time, "base_link", time, "map", rospy.Duration(1))
                coordinates_x.append(transform.transform.translation.x)
                coordinates_y.append(transform.transform.translation.y)
            except tf2_ros.ExtrapolationException:
                continue 
    plt.plot(coordinates_x, coordinates_y)
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title('Coordinate Graph (Base Link relative to Map)')
    plt.savefig(output_path)


def load_buffer(bag):
    tf_topics = ['/tf', '/tf_static']
    buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600))
    try:
        for topic, msg, stamp in tqdm(bag.read_messages(topics=tf_topics),
                                      total=bag.get_message_count(topic_filters=tf_topics)):
            if topic == '/tf':
                for tf in msg.transforms:
                    buffer.set_transform(tf, 'bag')
            elif topic == '/tf_static':
                for tf in msg.transforms:
                    buffer.set_transform_static(tf, 'bag')
    except ROSBagException:
        print('Could not read')
    return buffer
