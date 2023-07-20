import numpy as np
from math import sqrt


class Calculater:
    def __init__(self, icp, odom, saved_times):
        self.icp = np.array(icp)
        self.odom = np.array(odom)
        self.distance = 0
        self.speeds = []
        self.saved_times = saved_times

    def get_distances(self, coord):
        coord = np.array(coord)
        distances_one_period = np.abs(coord[1:] - coord[:-1])
        distances_xyz = [[0, 0, 0]]
        distances = [0]
        for distance in distances_one_period:
            distances_xyz.append(distances_xyz[-1] + distance)
            distances.append(sqrt(pow(distances_xyz[-1][0], 2) + pow(distances_xyz[-1][1], 2) +
                                  pow(distances_xyz[-1][2], 2)))
        self.distance = distances[-1]
        return distances

    def get_start_and_end_of_moving(self):
        start_of_moving = -1
        end_of_moving = -1
        for i in range(len(self.speeds)):
            if self.speeds[i] > 0.2 and start_of_moving == -1:
                start_of_moving = self.saved_times[i]
            elif self.speeds[i] < 0.2 or (i == len(self.speeds) - 1 and self.speeds[i] > 0.2):
                end_of_moving = self.saved_times[i]
        return start_of_moving, end_of_moving

    def get_speeds_one_period(self):
        distances_one_period = np.abs(self.icp[1:] - self.icp[:-1])
        saved_times = np.array(self.saved_times)
        times_one_period = saved_times[1:] - saved_times[:-1]
        speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
        for speed in speeds_xyz:
            self.speeds.append(sqrt(pow(speed[0], 2) + pow(speed[1], 2) + pow(speed[2], 2)))
        return self.speeds

    def get_joy_control_coordinates(self, joy_control_times):
        indices = np.unique(np.searchsorted(self.saved_times, joy_control_times))
        split_indices = np.concatenate(([-1], np.where(np.diff(indices) > 1)[0], [len(indices) - 1]))
        split_indices = [indices[split_indices[i] + 1:split_indices[i + 1] + 1] for i in range(len(split_indices) - 1)]
        control_coordinates = []
        for indices in split_indices:
            control_coordinates.append((self.icp - self.icp[0])[indices[indices < len(self.saved_times)]])
        return control_coordinates


    def get_joy_control_binary(self, joy_control_times):
        indices = np.unique(np.searchsorted(self.saved_times, joy_control_times))
        control_binary = np.zeros(len(self.saved_times), dtype=int)
        control_binary[indices[indices < len(self.saved_times)]] = 1
        return control_binary

    def transform_trajectory(self, coord, matrix):
        transformed_coord = []
        for vector in coord:
            transformed_coord.append(np.dot(matrix, vector))
        return np.array(transformed_coord) - transformed_coord[0]
