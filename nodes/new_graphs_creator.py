import numpy as np
import matplotlib.pyplot as plt
from math import sqrt


class GraphsCreator:
    def __init__(self, point_cloud, icp, odom, saved_times, joy_control_times):
        self.point_cloud = point_cloud
        self.icp = np.array(icp)
        self.odom = np.array(odom)
        self.saved_times = saved_times
        self.joy_control_times = joy_control_times

    def create_graph_xy_and_point_cloud(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/shared_point_cloud.png"
        fig, ax = plt.subplots()
        marker_size = 0.5
        plt.xlabel('X-coordinate')
        plt.ylabel('Y-coordinate')
        plt.title("XY plot of UGV's movement")
        combined_points = np.concatenate(self.point_cloud, axis=1)
        colors = self.transform_z_coordinates_to_color(combined_points[2, :])
        ax.scatter(combined_points[0, :], combined_points[1, :], s=marker_size, c=colors, cmap='Greens')
        self.icp = self.move_coordinates_to_the_origin(self.icp)
        self.odom = self.move_coordinates_to_the_origin(self.odom)
        ax.plot(self.odom[:, 0], self.odom[:, 1], color='blue', label='imu_odom')
        ax.plot(self.icp[:, 0], self.icp[:, 1], color='red', linestyle='--', label='icp_odom')
        plt.legend()
        plt.savefig(output_path)
        plt.close()

    def create_graph_x_over_time(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/coord_x_and_time.png"
        fig, ax = plt.subplots()
        plt.xlabel('time [s]')
        plt.ylabel('distance[m]')
        plt.title("UGV's movement in X direction")
        ax.plot(self.saved_times, self.move_coordinates_to_the_origin(self.odom[:, 0]), color='blue')
        ax.plot(self.saved_times, self.move_coordinates_to_the_origin(self.icp[:, 0]), color='red', linestyle='--')
        plt.savefig(output_path)
        plt.close()

    def create_graph_y_over_time(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/coord_y_and_time.png"
        fig, ax = plt.subplots()
        plt.xlabel('time [s]')
        plt.ylabel('distance[m]')
        plt.title("UGV's movement in Y direction")
        ax.plot(self.saved_times, self.move_coordinates_to_the_origin(self.odom[:, 1]), color='blue')
        ax.plot(self.saved_times, self.move_coordinates_to_the_origin(self.icp[:, 1]), color='red', linestyle='--')
        plt.savefig(output_path)
        plt.close()

    def create_graph_z_over_time(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/coord_z_and_time.png"
        fig, ax = plt.subplots()
        plt.xlabel('time [s]')
        plt.ylabel('distance[m]')
        plt.title("UGV's movement in Z direction")
        ax.plot(self.saved_times, self.move_coordinates_to_the_origin(self.odom[:, 2]), color='blue')
        ax.plot(self.saved_times, self.move_coordinates_to_the_origin(self.icp[:, 2]), color='red', linestyle='--')
        plt.savefig(output_path)
        plt.close()

    def create_graph_distance_over_time(self):
        output_path = "/home/robert/catkin_ws/src/bag_crawler/web_server/distance_and_time.png"
        fig, ax = plt.subplots()
        plt.xlabel('time [s]')
        plt.ylabel('distance[m]')
        plt.title("UGV's travelled distance over time")
        ax.plot(self.saved_times, self.get_distances(self.odom), color='blue')
        ax.plot(self.saved_times, self.get_distances(self.icp), color='red', linestyle='--')
        start_of_moving, end_of_moving = self.find_start_and_end_of_moving(self.get_speeds_one_period())
        ax.axvline(start_of_moving, color='green', linestyle=':', label='start_of_moving')
        ax.axvline(end_of_moving, color='green', linestyle='--', label='end_of_moving')
        plt.legend()
        plt.savefig(output_path)
        plt.close()

    def transform_z_coordinates_to_color(self, coord_z):
        coord_z += abs(np.min(coord_z))
        colors = np.log1p(coord_z)
        return colors

    def move_coordinates_to_the_origin(self, coordinates):
        return np.array(coordinates) - coordinates[0]

    def get_distances(self, coord):
        distances_one_period = np.abs(coord[1:] - coord[:-1])
        distances_xyz = [[0, 0, 0]]
        distances = [0]
        for distance in distances_one_period:
            distances_xyz.append(distances_xyz[-1] + distance)
            distances.append(sqrt(pow(distances_xyz[-1][0], 2) + pow(distances_xyz[-1][1], 2) +
                                  pow(distances_xyz[-1][2], 2)))
        return distances

    def find_start_and_end_of_moving(self, speeds):
        start_of_moving = -1
        end_of_moving = -1
        for i in range(len(speeds)):
            if speeds[i] > 0.2 and start_of_moving == -1:
                start_of_moving = self.saved_times[i]
            elif speeds[i] < 0.2 or (i == len(speeds) - 1 and speeds[i] > 0.2):
                end_of_moving = self.saved_times[i]
        return start_of_moving, end_of_moving

    def get_speeds_one_period(self):
        distances_one_period = np.abs(self.icp[1:] - self.icp[:-1])
        saved_times = np.array(self.saved_times)
        times_one_period = saved_times[1:] - saved_times[:-1]
        speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
        speeds = []
        for speed in speeds_xyz:
            speeds.append(sqrt(pow(speed[0], 2) + pow(speed[1], 2) + pow(speed[2], 2)))
        return speeds
