import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv
import os


def create_graph_xy_and_point_cloud(odom, icp, point_cloud, folder):
    fig, ax = plt.subplots()
    marker_size = 0.5
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement")
    colors = transform_z_coordinates_to_color(point_cloud[2, :])
    ax.scatter(point_cloud[0, :], point_cloud[1, :], s=marker_size, c=colors, cmap='Greens')
    ax.plot(odom[0, :], odom[1, :], color='blue', label='imu_odom')
    ax.plot(icp[0, :], icp[1, :], color='red', linestyle='--', label='icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/XY_plot_of_UGVs_movement.png")
    plt.close()


def create_graph_x_over_time(odom, icp, saved_times, folder):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in X direction")
    ax.plot(saved_times, odom[0, :], color='blue')
    ax.plot(saved_times, icp[0, :], color='red', linestyle='--')
    plt.savefig(f"{folder}/UGVs_movement_in_X_direction.png")
    plt.close()


def create_graph_y_over_time(odom, icp, saved_times, folder):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Y direction")
    ax.plot(saved_times, odom[1, :], color='blue')
    ax.plot(saved_times, icp[1, :], color='red', linestyle='--')
    plt.savefig(f"{folder}/UGVs_movement_in_Y_direction.png")
    plt.close()


def create_graph_z_over_time(odom, icp, saved_times, folder):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Z direction")
    ax.plot(saved_times, odom[2, :], color='blue')
    ax.plot(saved_times, icp[2, :], color='red', linestyle='--')
    plt.savefig(f"{folder}/UGVs_movement_in_Z_direction.png")
    plt.close()


def create_graph_distance_over_time(distances_icp, distances_odom, saved_times, start_of_moving, end_of_moving, folder):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's travelled distance over time")
    ax.plot(saved_times, distances_odom, color='blue')
    ax.plot(saved_times, distances_icp, color='red', linestyle='--')
    ax.axvline(start_of_moving, color='green', linestyle=':', label='start_of_moving')
    ax.axvline(end_of_moving, color='green', linestyle='--', label='end_of_moving')
    plt.legend()
    plt.savefig(f"{folder}/UGVs_travelled_distance_over_time.png")
    plt.close()


def create_graph_joy_control_times_and_icp(icp, joy_control_coordinates, folder):
    fig, ax = plt.subplots()
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement along with joystick control trajectory sections")
    ax.plot(icp[0, :], icp[1, :], color='red', linestyle='--', label='icp_odom')
    for coordinates in joy_control_coordinates:
        ax.plot(coordinates[:, 0], coordinates[:, 1], color='orange')
    ax.plot([], [], color='orange', label='joy_control')
    plt.legend()
    plt.savefig(f"{folder}/XY_plot_with_joystick_control_trajectory_sections.png")
    plt.close()


def show_point_cloud(point_cloud):
    point_cloud = point_cloud.T
    points = pv.PolyData(point_cloud)
    plotter = pv.Plotter()
    plotter.add_points(points, render_points_as_spheres=True, color='blue', point_size=5)
    plotter.show_axes()
    plotter.show()


def transform_z_coordinates_to_color(coord_z):
    coord_z += abs(np.min(coord_z))
    colors = np.log1p(coord_z)
    return colors
