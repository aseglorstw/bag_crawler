class BagScanner:

    def __int__(self, bag):
        self.bag = bag

    def find_points_topic(self):
        pass

    def find_base_link_to_map_transformation(self):
        pass

    def find_base_link_to_odom_transformation(self):
        pass

    def find_video_topic(self):
        pass

    def find_joy_topic(self):
        topics_info = self.bag.get_type_and_topic_info()[1]
        for topic_name, topics_info in topics_info.items():
            if "cmd_vel" in topic_name and "joy" in topic_name:
                return topic_name
        return -1
