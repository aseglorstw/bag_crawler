import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2


def save_video(bag):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/video.avi"
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video_out = cv2.VideoWriter(output_path, fourcc, 20.0, (1920, 1200), True)
    for topic, msg, time in bag.read_messages(topics=['/camera_front/image_color/compressed']):
        msg = CompressedImage(*slots(msg))
        np_arr = np.fromstring(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        video_out.write(image)
    video_out.release()


def slots(msg):
    return [getattr(msg, var) for var in msg.__slots__]

