import numpy as np
from math import sqrt
from ros_numpy import numpify


def get_distances(coordinates):
    if coordinates is None:
        return None
    transpose_coordinates = coordinates.T
    distances_one_period = np.abs(transpose_coordinates[1:] - transpose_coordinates[:-1])
    distances_xyz = np.concatenate((np.zeros((1, 3)), np.cumsum(distances_one_period, axis=0)), axis=0)
    distances = np.linalg.norm(distances_xyz, axis=1)
    return distances


def get_start_and_end_of_moving(speeds, saved_times):
    if speeds is None:
        return None, None
    moving = np.where(speeds > 0.2)[0]
    if len(moving) == 0:
        return None, None
    start_of_moving = saved_times[moving[0]]
    end_of_moving = saved_times[moving[-1] + 1] if len(moving) < len(saved_times) else saved_times[moving[-1]]
    return start_of_moving, end_of_moving


def get_speeds_one_period(coordinates_icp, coordinates_odom, saved_times):
    if coordinates_icp is None and coordinates_odom is None:
        return None
    transpose_coordinates = coordinates_icp.T if coordinates_icp is not None else coordinates_odom.T
    distances_one_period = np.abs(transpose_coordinates[1:] - transpose_coordinates[:-1])
    times_one_period = saved_times[1:] - saved_times[:-1]
    speeds_xyz = distances_one_period / times_one_period.reshape(-1, 1)
    speeds = np.linalg.norm(speeds_xyz, axis=1)
    return speeds


def get_joy_control_coordinates(coordinates, joy_control_times, saved_times):
    if coordinates is None or joy_control_times is None:
        return None
    indices = np.unique(np.searchsorted(saved_times, joy_control_times))
    print(indices)
    split_indices = np.concatenate(([-1], np.where(np.diff(indices) > 1)[0], [len(indices) - 1]))
    split_indices = [indices[split_indices[i] + 1:split_indices[i + 1] + 1] for i in range(len(split_indices) - 1)]
    control_coordinates = []
    for indices in split_indices:
        control_coordinates.append(coordinates.T[indices[indices < len(saved_times)]])
    return control_coordinates


def transform_trajectory(coordinates, matrix):
    if len(coordinates) == 0:
        return None
    inv_matrix = np.linalg.inv(numpify(matrix)[:3, :3])
    coordinates = np.concatenate(coordinates, axis=1)
    transformed_coordinates = inv_matrix @ coordinates - np.expand_dims(inv_matrix @ coordinates[:, 0], axis=1)
    return transformed_coordinates


def transform_point_cloud(point_cloud, matrix):
    if point_cloud[0] is None or matrix is None:
        return None
    inv_matrix = np.linalg.inv(numpify(matrix)[:3, :3])
    point_cloud = np.concatenate(point_cloud, axis=1)
    first_transform = np.array([[matrix.translation.x], [matrix.translation.y], [matrix.translation.z]])
    transformed_point_cloud = inv_matrix @ point_cloud - inv_matrix @ first_transform
    return transformed_point_cloud


def get_average_speed(speeds):
    return np.sum(speeds) / len(speeds) if speeds is not None else None
