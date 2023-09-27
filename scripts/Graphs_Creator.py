import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv
import json


def create_graph_xy_and_point_cloud(coord_icp, objects_odom, point_cloud, folder, odom_topics_color):
    fig, ax = plt.subplots()
    marker_size = 0.5
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement")
    if point_cloud is not None:
        not_nan_index = ~np.isnan(point_cloud[0, :]) & ~np.isnan(point_cloud[1, :]) & ~np.isnan(point_cloud[2, :])
        cleaned_point_cloud = [point_cloud[0, :][not_nan_index], point_cloud[1, :][not_nan_index],
                               point_cloud[2, :][not_nan_index]]
        colors = transform_z_coordinates_to_color(cleaned_point_cloud[2])
        ax.scatter(cleaned_point_cloud[0], cleaned_point_cloud[1], s=marker_size, c=colors, cmap='Greens',
                   label="point_cloud")
    for idx, odom in enumerate(objects_odom):
        coord_odom = odom.get_transformed_odom()
        if coord_odom is not None:
            color = odom_topics_color[odom.get_topic_name()]
            ax.plot(coord_odom[0, :], coord_odom[1, :], color=color, label=odom.get_topic_name())
    if coord_icp is not None:
        ax.plot(coord_icp[0, :], coord_icp[1, :], color='red', linestyle='--', label='/icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/XY_plot_of_UGVs_movement.png")
    plt.close()


def create_graph_x_over_time(coord_icp, times_icp, objects_odom, folder, odom_topics_color):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in X direction")
    for idx, odom in enumerate(objects_odom):
        coord_odom = odom.get_transformed_odom()
        if coord_odom is not None:
            color = odom_topics_color[odom.get_topic_name()]
            ax.plot(odom.get_times(), coord_odom[0, :], color=color, label=odom.get_topic_name())
    if coord_icp is not None:
        ax.plot(times_icp, coord_icp[0, :], color='red', linestyle='--', label='/icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/UGVs_movement_in_X_direction.png")
    plt.close()


def create_graph_y_over_time(coord_icp, times_icp, objects_odom, folder, odom_topics_color):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Y direction")
    for idx, odom in enumerate(objects_odom):
        coord_odom = odom.get_transformed_odom()
        if coord_odom is not None:
            color = odom_topics_color[odom.get_topic_name()]
            ax.plot(odom.get_times(), coord_odom[1, :], color=color, label=odom.get_topic_name())
    if coord_icp is not None:
        ax.plot(times_icp, coord_icp[1, :], color='red', linestyle='--', label='/icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/UGVs_movement_in_Y_direction.png")
    plt.close()


def create_graph_z_over_time(coord_icp, times_icp, objects_odom, folder, odom_topics_color):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's movement in Z direction")
    for idx, odom in enumerate(objects_odom):
        coord_odom = odom.get_transformed_odom()
        if coord_odom is not None:
            color = odom_topics_color[odom.get_topic_name()]
            ax.plot(odom.get_times(), coord_odom[2, :], color=color, label=odom.get_topic_name())
    if coord_icp is not None:
        ax.plot(times_icp, coord_icp[2, :], color='red', linestyle='--', label='/icp_odom')
    plt.legend()
    plt.savefig(f"{folder}/UGVs_movement_in_Z_direction.png")
    plt.close()


def create_graph_distance_over_time(distances_icp, saved_times_icp, start_and_end_of_moving_icp,
                                    start_and_end_of_moving_odom, objects_odom, folder, odom_topics_color):
    fig, ax = plt.subplots()
    plt.xlabel('time [s]')
    plt.ylabel('distance[m]')
    plt.title("UGV's travelled distance over time")
    for idx, odom in enumerate(objects_odom):
        distances_odom = odom.get_distances()
        if distances_odom is not None:
            color = odom_topics_color[odom.get_topic_name()]
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
                                           topic_name_joy, folder, odom_topics_color):
    fig, ax = plt.subplots()
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title("XY plot of UGV's movement along with joystick control trajectory sections")
    if coord_icp is not None:
        ax.plot(coord_icp[0, :], coord_icp[1, :], color='red', linestyle='--', label='icp_odom')
    elif coord_odom is not None:
        color = odom_topics_color[topic_name_odom]
        ax.plot(coord_odom[0, :], coord_odom[1, :], color=color, label=topic_name_odom)
    else:
        pass
    # I draw the graph as segments to show when the robot was controlled with the joystick against the background
    # of the entire trajectory.
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


def match_color_odom_topic(objects_odom):
    odom_topics_colors = dict()
    colors = ['blue', 'brown', 'yellow', 'black', 'purple', 'gray', 'pink', 'darkcyan', 'navy', 'khaki', 'olive']
    for idx, odom_topic in enumerate(objects_odom):
        odom_topics_colors[odom_topic.get_topic_name()] = colors[idx % len(colors)]
    return odom_topics_colors


def write_info_to_data_availability(output_folder):
    with open(f"{output_folder}/.data_availability.json", "r", encoding="utf-8") as file:
        task_list = json.load(file)
    task_list["graphs"] = True
    with open(f"{output_folder}/.data_availability.json", "w", encoding="utf-8") as file:
        json.dump(task_list, file, indent=4)
