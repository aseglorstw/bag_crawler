import roslaunch
import rospy
import subprocess


def create_transform_odom_to_map(path_to_bag_file, duration):
    cli_args = ["/home/robert/catkin_ws/src/bag_crawler/launch/slam_with_tf_record.launch", f"bag_file:={path_to_bag_file}"]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    rospy.sleep(duration)
    full_tf_name = f'{path_to_bag_file.split(".")[0]}.full_tf.bag'
    loc_file_name = f'{path_to_bag_file.split(".")[0]}_loc.bag'
    command = (f"rosbag filter {full_tf_name} {loc_file_name} \"topic == '/tf' and "
               "m.transforms[0].child_frame_id == 'odom' and m.transforms[0].header.frame_id == 'map'\"")
    subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


create_transform_odom_to_map("/home/robert/catkin_ws/src/bag_crawler/bagfiles/bag_files_husky/husky_2022-09-23-12-38-31.bag", 100)
