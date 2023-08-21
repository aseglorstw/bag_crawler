import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
import datetime
import os


class VideoDataProcessor:
    def __init__(self, bag):
        self.bag = bag
        self.start_time = bag.get_start_time()

    def read_images_and_save_video(self, folder):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        topic_names = list(self.find_camera_topic())
        if topic_names[0] is None:
            print("The topic in which messages from the camera are posted was not found")
            return False
        for topic_name in topic_names:
            save_interval = self.calculate_save_interval(topic_name)
            video_name = f"{folder}/{self.create_name_for_video(topic_name)}_video.avi"
            video_out = cv2.VideoWriter(video_name, fourcc, 60, (1920, 1200), True)
            for msg_number, (topic, msg, time) in enumerate(self.bag.read_messages(topics=[topic_name])):
                if msg_number % save_interval == 0:
                    time = rospy.Time.from_sec(time.to_sec())
                    time_from_start = int(time.to_sec() - self.start_time)
                    msg = CompressedImage(*self.slots(msg))
                    np_arr = np.fromstring(msg.data, np.uint8)
                    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    current_datetime = self.get_datetime(time_from_start)
                    cv2.putText(image, current_datetime, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
                    video_out.write(image)
                    print(f"Image from topic {topic_name} for video is saved. Time: {time.to_sec() - self.start_time}")
            print(f"Video  from topic {topic_name} is saved.")
            video_out.release()
        return True

    def find_camera_topic(self):
        for topic_name, topic_info in self.bag.get_type_and_topic_info()[1].items():
            if "CompressedImage" in topic_info.msg_type:
                yield topic_name
        return None

    def calculate_save_interval(self, topic_name):
        save_interval = int(self.bag.get_type_and_topic_info()[1][topic_name].message_count / 1200)
        return save_interval if save_interval > 0 else 1

    def get_datetime(self, time_from_start):
        return datetime.datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S') + "+" + \
                                                                str(datetime.timedelta(seconds=time_from_start))

    @staticmethod
    def write_result_to_file(result, output_folder):
        is_video_in_file = False
        state_video = "False"
        if result:
            state_video = "True"
        if os.path.exists(f"{output_folder}/.data_availability.txt"):
            with open(f"{output_folder}/.data_availability.txt", 'r+', encoding="utf-8") as file:
                lines = file.readlines()
            with open(f"{output_folder}/.data_availability.txt", 'w', encoding="utf-8") as file:
                for line in lines:
                    if line.startswith('video'):
                        file.write(f"video {state_video}\n")
                        is_video_in_file = True
                    else:
                        file.write(line)
                if not is_video_in_file:
                    file.write(f"video {state_video}\n")
        else:
            with open(f"{output_folder}/.data_availability.txt", 'w', encoding="utf-8") as file:
                file.write(f"video {state_video}\n")

    @staticmethod
    def create_name_for_video(topic_name):
        return topic_name.replace('/', '_')[1:]

    @staticmethod
    def slots(msg):
        return [getattr(msg, var) for var in msg.__slots__]

