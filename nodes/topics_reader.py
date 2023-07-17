import rospy
import numpy as np
import yaml
import matplotlib.pyplot as plt


def save_topics(bag):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/topics.txt"
    topics_info = bag.get_type_and_topic_info()[1]
    with open(output_path, "w", encoding="utf-8") as file:
        for topic_name, topic_info in topics_info.items():
            msg_type = topic_info.msg_type
            message_count = topic_info.message_count
            frequency = topic_info.frequency
            file.write(f"{topic_name} {msg_type} {message_count} {frequency}\n")


def create_graph_control_joy_and_time(bag):
    saved_times = []
    start_time = bag.get_start_time()
    joy_name = find_joy_topic(bag)
    if joy_name != -1:
        for topic, msg, time in bag.read_messages(topics=[joy_name]):
            time = rospy.Time.from_sec(time.to_sec())
            save_time = int(time.to_sec() - start_time)
            if save_time not in saved_times:
                saved_times.append(save_time)
        time_array = create_time_array(bag)
        control_joy = create_array_of_binary_control_joy(time_array, saved_times)
        save_graph_control_joy_and_time(time_array, control_joy)
    else:
        print("Topic joy not founded")


def find_joy_topic(bag):
    topics_info = bag.get_type_and_topic_info()[1]
    for topic_name, topics_info in topics_info.items():
        if "joy" in topic_name:
            return topic_name
    return -1


def create_time_array(bag):
    info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.Loader)
    time_array = np.arange(0, info_dict['duration'], 1, dtype=int)
    return time_array


def create_array_of_binary_control_joy(time_array, saved_times):
    control_joy = []
    for time in time_array:
        if time in saved_times:
            control_joy.append(1)
        else:
            control_joy.append(0)
    return control_joy


def save_graph_control_joy_and_time(time_array, control_joy):
    output_path = '/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/control_joy_and_time.png'
    fig, ax = plt.subplots()
    ax.step(time_array, control_joy, color='blue', where='post')
    ax.set_xlabel('time')
    ax.set_ylabel('control joy')
    ax.set_title('Joystick robot control chart')
    plt.savefig(output_path)
    plt.close()
