import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2
import rospy


def save_image(bag):
    path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/images/image_"
    saved_times = set()
    number_of_image = 0
    for topic, msg, time in bag.read_messages(topics=['/camera_front/image_color/compressed']):
        time = rospy.Time.from_sec(time.to_sec())
        time_sec = time.to_sec()
        start_time = bag.get_start_time()
        time_sec = time_sec - start_time
        save_time = int(time_sec)
        if save_time % 10 == 0 and save_time not in saved_times:
            saved_times.add(save_time)
            msg = CompressedImage(*slots(msg))
            np_arr = np.fromstring(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imwrite(path + str(number_of_image) + ".png", image)
            number_of_image += 1
    

def slots(msg):
    return [getattr(msg, var) for var in msg.__slots__]
