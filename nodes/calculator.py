import numpy as np
from math import sqrt
from ros_numpy import numpify


def get_joy_control_coordinates(icp, odom, joy_control_times, saved_times_icp, saved_times_odom):
    if (icp is None and odom is None) or joy_control_times is None:
        return None
    saved_times = saved_times_icp if icp is not None else saved_times_odom
    coordinates = icp if icp is not None else odom
    indices = np.unique(np.searchsorted(saved_times, joy_control_times))
    split_indices = np.concatenate(([-1], np.where(np.diff(indices) > 5)[0], [len(indices) - 1]))
    split_indices = [indices[split_indices[i] + 1:split_indices[i + 1] + 1] for i in range(len(split_indices) - 1)]
    control_coordinates = []
    for indices in split_indices:
        control_coordinates.append(coordinates.T[indices[indices < len(saved_times)]])
    return control_coordinates

