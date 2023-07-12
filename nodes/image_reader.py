import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2


def save_image(bag):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/video.mp4"
    frame_size = (640, 480)
    fps = 30.0
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_out = cv2.VideoWriter(output_path, fourcc, fps, frame_size)
    for topic, msg, time in bag.read_messages(topics=['/camera_front/image_color/compressed']):
        msg = CompressedImage(*slots(msg))
        np_arr = np.fromstring(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        video_out.write(image)
    video_out.release()


def slots(msg):
    return [getattr(msg, var) for var in msg.__slots__]

