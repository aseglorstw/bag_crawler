import utils
import numpy as np
import matplotlib.pyplot as plt


def create_graph_xy_and_point_cloud(cloud_combined, icp, odom):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/shared_point_cloud.png"
    fig, ax = plt.subplots()
    marker_size = 0.5
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement")
    combined_points = np.concatenate(cloud_combined, axis=1)
    colors = utils.transform_z_coordinates_to_color(combined_points[2, :])
    ax.scatter(combined_points[0, :], combined_points[1, :], s=marker_size, c=colors, cmap='Greens')
    ax.plot(odom[:, 0], odom[:, 1], color='blue', label='imu_odom')
    ax.plot(icp[:, 0], icp[:, 1], color='red', linestyle='--', label='icp_odom')
    plt.legend()
    plt.savefig(output_path)
    plt.close()


def create_graph_x_over_time(icp_x, odom_x, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/coord_x_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in X direction")
    ax.plot(saved_times, utils.move_coordinates_to_the_origin(odom_x), color='blue')
    ax.plot(saved_times, utils.move_coordinates_to_the_origin(icp_x), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_y_over_time(icp_y, odom_y, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/coord_y_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Y direction")
    ax.plot(saved_times, utils.move_coordinates_to_the_origin(odom_y), color='blue')
    ax.plot(saved_times, utils.move_coordinates_to_the_origin(icp_y), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_z_over_time(icp_z, odom_z, saved_times):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/coord_z_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Z direction")
    ax.plot(saved_times, utils.move_coordinates_to_the_origin(odom_z), color='blue')
    ax.plot(saved_times, utils.move_coordinates_to_the_origin(icp_z), color='red', linestyle='--')
    plt.savefig(output_path)
    plt.close()


def create_graph_distance_over_time(icp, odom, saved_times, start_of_moving, end_of_moving):
    output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/distance_and_time.png"
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's travelled distance over time")
    ax.plot(saved_times, utils.get_distances(odom), color='blue')
    ax.plot(saved_times, utils.get_distances(icp), color='red', linestyle='--')
    ax.axvline(start_of_moving, color='green', linestyle=':', label='start_of_moving')
    ax.axvline(end_of_moving, color='green', linestyle='--', label='end_of_moving')
    plt.legend()
    plt.savefig(output_path)
    plt.close()
