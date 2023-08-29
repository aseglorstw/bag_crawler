import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv


def create_graph_xy_and_point_cloud(coord_icp, objects_odom, point_cloud, folder):
    fig, ax = plt.subplots()
    marker_size = 0.5
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement")
    if point_cloud is not None:
        colors = transform_z_coordinates_to_color(point_cloud[2, :])
        ax.scatter(point_cloud[0, :], point_cloud[1, :], s=marker_size, c=colors, cmap='Greens', label="point_cloud")
    colors_odom = ['blue', 'brown', 'yellow', 'black', 'purple', 'gray']
    for idx, odom in enumerate(objects_odom):
        coord_odom = odom.get_transformed_odom()
        if colors_odom is not None:
            color = colors_odom[idx % len(colors_odom)]
            ax.plot(coord_odom[0, :], coord_odom[1, :], color=color, label=odom.get_topic_name())
    if coord_icp is not None:
        ax.plot(coord_icp[0, :], coord_icp[1, :], color='red', linestyle='--', label='/icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/XY_plot_of_UGVs_movement.png")
    plt.close()


def create_graph_x_over_time(coord_icp, times_icp, objects_odom, folder):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in X direction")
    colors_odom = ['blue', 'brown', 'yellow', 'black', 'purple', 'gray']
    for idx, odom in enumerate(objects_odom):
        coord_odom = odom.get_transformed_odom()
        if odom is not None:
            color = colors_odom[idx % len(colors_odom)]
            ax.plot(odom.get_times(), coord_odom[0, :], color=color, label=odom.get_topic_name())
    if coord_icp is not None:
        ax.plot(times_icp, coord_icp[0, :], color='red', linestyle='--', label='/icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/UGVs_movement_in_X_direction.png")
    plt.close()


def create_graph_y_over_time(coord_icp, times_icp, objects_odom, folder):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Y direction")
    colors_odom = ['blue', 'brown', 'yellow', 'black', 'purple', 'gray']
    for idx, odom in enumerate(objects_odom):
        coord_odom = odom.get_transformed_odom()
        if odom is not None:
            color = colors_odom[idx % len(colors_odom)]
            ax.plot(odom.get_times(), coord_odom[1, :], color=color, label=odom.get_topic_name())
    if coord_icp is not None:
        ax.plot(times_icp, coord_icp[1, :], color='red', linestyle='--', label='/icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/UGVs_movement_in_Y_direction.png")
    plt.close()


def create_graph_z_over_time(coord_icp, times_icp, objects_odom, folder):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Z direction")
    colors_odom = ['blue', 'brown', 'yellow', 'black', 'purple', 'gray']
    for idx, odom in enumerate(objects_odom):
        coord_odom = odom.get_transformed_odom()
        if odom is not None:
            color = colors_odom[idx % len(colors_odom)]
            ax.plot(odom.get_times(), coord_odom[2, :], color=color, label=odom.get_topic_name())
    if coord_icp is not None:
        ax.plot(times_icp, coord_icp[2, :], color='red', linestyle='--', label='/icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/UGVs_movement_in_Z_direction.png")
    plt.close()


def create_graph_distance_over_time(distances_icp, saved_times_icp, start_and_end_of_moving_icp,
                                    start_and_end_of_moving_odom, objects_odom, folder):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's travelled distance over time")
    colors_odom = ['blue', 'brown', 'yellow', 'black', 'purple', 'gray']
    for idx, odom in enumerate(objects_odom):
        distances_odom = odom.get_distances()
        if distances_odom is not None:
            color = colors_odom[idx % len(colors_odom)]
            ax.plot(odom.get_times(), distances_odom, color=color, label=odom.get_topic_name())
    if distances_icp is not None:
        ax.plot(saved_times_icp, distances_icp, color='red', linestyle='--', label='/icp_odom')
    if start_and_end_of_moving_icp[0] is not None:
        ax.axvline(start_and_end_of_moving_icp[0], color='green', linestyle=':', label='start_of_moving')
        ax.axvline(start_and_end_of_moving_icp[1], color='green', linestyle='--', label='end_of_moving')
    elif start_and_end_of_moving_odom[0] is not None:
        ax.axvline(start_and_end_of_moving_odom[0], color='green', linestyle=':', label='start of moving')
        ax.axvline(start_and_end_of_moving_odom[1], color='green', linestyle='--', label='end of moving')
    plt.legend()
    plt.savefig(f"{folder}/UGVs_travelled_distance_over_time.png")
    plt.close()


def create_graph_joy_control_times_and_icp(coord_icp, coord_odom,  joy_control_coordinates, topic_name_odom,
                                           topic_name_joy, folder):
    fig, ax = plt.subplots()
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement along with joystick control trajectory sections")
    if coord_icp is not None:
        ax.plot(coord_icp[0, :], coord_icp[1, :], color='red', linestyle='--', label='icp_odom')
    elif coord_odom is not None:
        ax.plot(coord_odom[0, :], coord_odom[1, :], color='blue', label=topic_name_odom)
    else:
        pass
    if joy_control_coordinates is not None:
        for coordinates in joy_control_coordinates:
            ax.plot(coordinates[:, 0], coordinates[:, 1], color='orange')
    ax.plot([], [], color='orange', label=topic_name_joy)
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
