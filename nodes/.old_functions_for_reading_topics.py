
def read_point_cloud(self):
        topic_name = self.find_points_topic()
        if topic_name is None:
            self.data_availability["point_cloud"] = False
            print("The topic lidar posted to was not found")
            return None
        save_interval = 20
        for msg_number, (topic, msg, time) in enumerate(self.bags[0].read_messages(topics=[topic_name])):
            if msg_number % save_interval == 0:
                time = rospy.Time.from_sec(time.to_sec())
                save_time = time.to_sec() - self.start_time
                try:
                    msg = PointCloud2(*self.slots(msg))
                    cloud = np.array(list(read_points(msg)))
                    transform_map_lidar = self.buffer.lookup_transform_full("map", time, msg.header.frame_id, time,
                                                                            "map", rospy.Duration.from_sec(0.3))
                    matrix = numpify(transform_map_lidar.transform)
                    vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                    transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4]
                    print(f"Point cloud is saved. Time: {save_time}")
                    yield transformed_vectors
                except ExtrapolationException:
                    print(f"Transformation from lidar coordinate system to map was not found. Time: {save_time}")
                except LookupException as e:
                    missing_frame = str(e).split()[0]
                    print(f"Frame {missing_frame} doesn't exist")

def read_icp_odom(self):
        icp = []
        odom = []
        saved_times_icp = []
        saved_times_odom = []
        rotation_matrix_icp = None
        rotation_matrix_odom = None
        is_icp_works = True
        is_odom_works = True
        topic_name = self.find_points_topic()
        if topic_name is None:
            print("The topic lidar posted to was not found")
            self.data_availability["points"] = False
            return [None] * 6
        for topic, msg, time in self.bags[0].read_messages(topics=[topic_name]):
            time = rospy.Time.from_sec(time.to_sec())
            save_time = time.to_sec() - self.start_time
            if is_icp_works:
                try:
                    transform_icp = self.buffer.lookup_transform_full("map", time, "base_link", time, "map",
                                                                      rospy.Duration.from_sec(0.3))
                    icp.append(np.array([[transform_icp.transform.translation.x], [transform_icp.transform.translation.y],
                                         [transform_icp.transform.translation.z]]))
                    if rotation_matrix_icp is None:
                        rotation_matrix_icp = transform_icp.transform
                    saved_times_icp.append(save_time)
                    print(f"The coordinates of the robot relative to the 'map' frame are saved.Time: {save_time}")
                except ExtrapolationException:
                    print(f"The coordinates of the robot relative to the 'map' frame aren't saved.Time: {save_time}")
                except LookupException as e:
                    missing_frame = str(e).split()[0]
                    is_icp_works = False
                    self.data_availability[missing_frame[1:-1]] = False
                    print(f"Frame {missing_frame} doesn't exist")
            if is_odom_works:
                try:
                    transform_odom = self.buffer.lookup_transform_full("odom", time, "base_link", time, "odom",
                                                                       rospy.Duration.from_sec(0.3))
                    odom.append(
                        np.array([[transform_odom.transform.translation.x], [transform_odom.transform.translation.y],
                                  [transform_odom.transform.translation.z]]))
                    if rotation_matrix_odom is None:
                        rotation_matrix_odom = transform_odom.transform
                    saved_times_odom.append(save_time)
                    print(f"The coordinates of the robot relative to the 'odom' frame are saved.Time: {save_time}")
                except ExtrapolationException:
                    print(f"The coordinates of the robot relative to the 'odom' frame aren't saved.Time: {save_time}")
                except LookupException as e:
                    missing_frame = str(e).split()[0]
                    is_odom_works = False
                    self.data_availability[missing_frame[1:-1]] = False
                    print(f"Frame {missing_frame} doesn't exist")

        return (np.array(icp), np.array(odom), np.array(saved_times_icp), np.array(saved_times_odom),
                rotation_matrix_icp, rotation_matrix_odom)


 def load_buffer(self):
        tf_topics = ['/tf', '/tf_static', 'points']
        self.buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600))
        for bag in self.bags:
            try:
                for topic, msg, time in tqdm(bag.read_messages(topics=tf_topics),
                                             total=bag.get_message_count(topic_filters=tf_topics)):
                    for tf in msg.transforms:
                        if topic == '/tf':
                            self.buffer.set_transform(tf, 'bag')
                        elif topic == '/tf_static':
                            self.buffer.set_transform_static(tf, 'bag')
            except ROSBagException:
                print('Could not read')
