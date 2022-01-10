#!/usr/bin/env python

import numpy as np
from bresenham import bresenham


def occupancy_grid_mapping(occ_grid_map, x_spacing=0.04, y_spacing=0.04, x_min=-5, y_min=-5, ranges=[0], scan_angles=[0], pose=[0, 0, 0]):
    """
    Function to update the cell values of an occupancy grid using lidar range values and the robot's pose
    :param occ_grid_map: The unupdated grid
    :param x_spacing: The resolution of the grid in the x-axis
    :param y_spacing: The resolution of the grid in the y-axis
    :param x_min: The lowest x-value of the map (at the top-left corner of the map)
    :param y_min: The lowest y-value of the map (at the top-left corner of the map)
    :param ranges: The lidar range values
    :param scan_angles: The angle of the range scans
    :param pose: The robot's 2D pose [x, y, theta]
    :return: The updated grid
    """

    lo_occ = 1
    lo_free = 0.5
    lo_max = 100
    lo_min = -100

    x_y = pose[0:2]
    theta = pose[2]

    for i in range(len(ranges)):
        d = ranges[i]
        alpha = scan_angles[i]

        x_y_occ = np.array([(d * np.cos(theta + alpha)), (d * np.sin(theta + alpha))]) + np.array(x_y)

        j_index_stop = int(np.ceil((max(0, ((x_y_occ[0] - x_min) - (x_spacing / 2))) / x_spacing)))
        i_index_stop = int(np.ceil((max(0, ((x_y_occ[1] - y_min) - (y_spacing / 2))) / y_spacing)))

        if j_index_stop < 0 or j_index_stop > 249 or i_index_stop < 0 or i_index_stop > 249:
            continue
        
        x_y_iocc = [i_index_stop, j_index_stop]
        stop = np.array(x_y_iocc)

        j_index_start = int(np.ceil((max(0, ((x_y[0] - x_min) - (x_spacing / 2))) / x_spacing)))
        i_index_start = int(np.ceil((max(0, ((x_y[1] - y_min) - (y_spacing / 2))) / y_spacing)))

        start_iocc = [i_index_start, j_index_start]
        start = np.array(start_iocc)

        free_spaces = np.array(bresenham(start, stop))

        for j in range(len(free_spaces)):
            occ_grid_map[free_spaces[j][0]][free_spaces[j][1]] = max(lo_min,
                                                                     occ_grid_map[free_spaces[j][0]][free_spaces[j][1]]
                                                                     - lo_free)

        occ_grid_map[i_index_stop][j_index_stop] = min(lo_max, occ_grid_map[i_index_stop][j_index_stop] + lo_occ)

    return occ_grid_map
