def save_topics(bag):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/nodes/web_server/topics.txt"
    with open(output_path, "w", encoding="utf - 8") as file:
        topics_name = bag.get_type_and_topic_info()[1].keys()
        for topic_name in topics_name:
            topics_info = bag.get_type_and_topic_info()[1]
            if topic_name in topics_info:
                topic_info = topics_info[topic_name]
                msg_type = topic_info.msg_type
                message_count = topic_info.message_count
                file.write(str(topic_name) + " " + msg_type + " " + str(message_count) + "\n")
