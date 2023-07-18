import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2
import datetime
import rospy


def save_video(bag):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/video.avi"
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    fps = calculate_fps(bag)
    video_out = cv2.VideoWriter(output_path, fourcc, fps, (1920, 1200), True)
    start_time = bag.get_start_time()
    for msg_number, (topic, msg, time) in enumerate(bag.read_messages(topics=['/camera_front/image_color/compressed'])):
        time = rospy.Time.from_sec(time.to_sec())
        time_from_start = int(time.to_sec() - start_time)
        msg = CompressedImage(*slots(msg))
        np_arr = np.fromstring(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        current_datetime = datetime.datetime.fromtimestamp(start_time).strftime('%Y-%m-%d %H:%M:%S') + "+" + \
            str(datetime.timedelta(seconds=time_from_start))
        cv2.putText(image, current_datetime, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
        video_out.write(image)
    video_out.release()


def slots(msg):
    return [getattr(msg, var) for var in msg.__slots__]



