def save_topics_info(bag):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/topics.txt"
    topics_info = bag.get_type_and_topic_info()[1]
    with open(output_path, "w", encoding="utf-8") as file:
        for topic_name, topic_info in topics_info.items():
            msg_type = topic_info.msg_type
            message_count = topic_info.message_count
            frequency = topic_info.frequency
            file.write(f"{topic_name} {msg_type} {message_count} {frequency}\n")
