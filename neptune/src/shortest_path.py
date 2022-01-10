import numpy as np
import matplotlib.pyplot as plt


def a_star(occupancy_map, start, goal, x_spacing=0.04, y_spacing=0.04, x_min=-5, y_min=-5, robot_radius = 0.2, show_map=False):
    """
    Implemention of the A-star path-planning algorithm to compute the shortest path between two points on a grid based map
    :param occupancy_map: The occupancy grid map
    :param start: The start point
    :param goal: The goal point
    :param x_spacing: The resolution of the grid in the x-axis
    :param y_spacing: The resolution of the grid in the y-axis
    :param x_min: The lowest x-value of the map (at the top-left corner of the map)
    :param y_min: The lowest y-value of the map (at the top-left corner of the map)
    :param robot_radius: The radius of the robot
    :param show_map: A boolean to determine if a visual of the grid should be displayed with matplotlib
    :return: A list containing the way points of the shortest path from the start to the goal
    """
    occupancy_map = scale_map_obstacles(occupancy_map, robot_radius, x_spacing, y_spacing)

    row_count = len(occupancy_map)
    col_count = len(occupancy_map[0])


    start_j = int(np.ceil((max(0, ((start[0] - x_min) - (x_spacing / 2))) / x_spacing)))
    start_i = int(np.ceil((max(0, ((start[1] - y_min) - (y_spacing / 2))) / y_spacing)))

    goal_j = int(np.ceil((max(0, ((goal[0] - x_min) - (x_spacing / 2))) / x_spacing)))
    goal_i = int(np.ceil((max(0, ((goal[1] - y_min) - (y_spacing / 2))) / y_spacing)))

    map_ = np.zeros([row_count, col_count])
    for i in range(row_count):
        for j in range(col_count):
            if is_occupied(occupancy_map[i][j]) == True:
                map_[i][j] = 1
            elif is_occupied(occupancy_map[i][j]) == False:
                map_[i][j] = 0
            else:
                map_[i][j] = 1

    distances_f = np.full([row_count, col_count], np.inf)
    distances_g = np.full([row_count, col_count], np.inf)
    parents = np.full([row_count, col_count, 2], -1, dtype=int)

    distances_g[start_i][start_j] = 0
    distances_f[start_i][start_j] = euclid_dist(start, goal)

    path_map = np.array(map_)
    path_map[start_i][start_j] = 4
    path_map[goal_i][goal_j] = 4


    while True:
        min_f = np.min(distances_f)
        current_f = np.where(distances_f == min_f)
        current_f = [current_f[0][0], current_f[1][0]]

        i = current_f[0]
        j = current_f[1]

        current_g_value = distances_g[i][j]
        distances_f[i][j] = np.inf

        if np.isinf(min_f) or (i == goal_i and j == goal_j):
            break

        inds = ((-1, 1), (0, 1), (1, 1), (-1, 0), (1, 0), (-1, -1), (0, -1), (1, -1))
        # inds = ((1, 0), (-1, 0), (0, 1), (0, -1))
        diag_cost = np.sqrt((x_spacing ** 2) + (y_spacing ** 2))
        x_cost = x_spacing
        y_cost = y_spacing
        costs = (diag_cost, y_cost, diag_cost, x_cost, x_cost, diag_cost, y_cost, diag_cost)
        # costs = (y_cost, x_cost, x_cost, y_cost)

        try:
            for k in range(len(inds)):
                if i + inds[k][0] < 0 or j + inds[k][1] < 0:
                    continue

                if map_[i + inds[k][0]][j + inds[k][1]] != 1:
                    if (i + inds[k][0] != start_i or j + inds[k][1] != start_j) \
                            and (i + inds[k][0] != goal_i or j + inds[k][1] != goal_j):
                        path_map[i + inds[k][0]][j + inds[k][1]] = 3
                    if distances_g[i + inds[k][0]][j + inds[k][1]] > current_g_value + costs[k]:
                        distances_g[i + inds[k][0]][j + inds[k][1]] = current_g_value + costs[k]
                        if not j > 0 and i > 0:
                            current_point = [x_min, ((i - 0.5) * y_spacing) + y_min]
                        elif j > 0 and not i > 0:
                            current_point = [((j - 0.5) * x_spacing) + x_min, y_min]
                        elif not (j > 0 or i > 0):
                            current_point = [x_min, y_min]
                        else:
                            current_point = [((j - 0.5) * x_spacing) + x_min, ((i - 0.5) * y_spacing) + y_min]
                        distances_f[i + inds[k][0]][j + inds[k][1]] = distances_g[i + inds[k][0]][j + inds[k][1]] + \
                                                                      euclid_dist(current_point, goal)
                        parents[i + inds[k][0]][j + inds[k][1]] = current_f
        except IndexError:
            pass

    if np.isinf(distances_g[goal_i][goal_j]):
        path = []
    else:
        path = [[goal_i, goal_j]]

        while parents[path[0][0]][path[0][1]][0] != -1 and parents[path[0][0]][path[0][1]][1] != -1:
            path = [[parents[path[0][0]][path[0][1]][0], parents[path[0][0]][path[0][1]][1]]] + path

        path_disp = np.array(path)

        for i in range(len(path)):
            x_ = path[i][1]
            y_ = path[i][0]

            if x_ - start_j > 0:
                path[i][0] = ((x_ - 0.5) * x_spacing) + x_min
            else:
                path[i][0] = ((x_) * x_spacing) + x_min

            if y_ - start_i > 0:
                path[i][1] = ((y_ - 0.5) * y_spacing) + y_min
            else:
                path[i][1] = ((y_) * y_spacing) + y_min

        path = path + [[goal[0], goal[1]]]

        # DISPLAY PATH
        # TODO: Keep map y-inversion?
        if (show_map):
            for i in range(len(path_disp)):
                if i != 0 and i != len(path_disp) - 1:
                    path_map[path_disp[i][0]][path_disp[i][1]] = 5

                plt.imshow(path_map[::-1])
                plt.pause(0.1)

            plt.show()
            plt.close()

    return np.array(path)


def scale_map_obstacles(real_map, robot_radius, x_spacing, y_spacing):
    """
    A function to increase the area of obstacles in the map to account for the size of the robot
    :param real_map: The actual occupancy grid map
    :param robot_radius: The radius of the robot
    :param x_spacing: The resolution of the grid in the x-axis
    :param y_spacing: The resolution of the grid in the y-axis
    :return: The modified map with the scaled obstacles
    """
    i_count = int(np.ceil(robot_radius / y_spacing))
    j_count = int(np.ceil(robot_radius / x_spacing))

    scaled_map = np.array(real_map)
    row_count = len(real_map)
    col_count = len(real_map[0])

    for i in range(row_count):
        for j in range(col_count):
            map_cell_value = real_map[i][j]
            if is_occupied(map_cell_value) == True:
                m, n = i - i_count, j - j_count
                try:
                    while m <= i + i_count and j <= j + j_count:
                        if m < 0:
                            m += 1
                            n = j - j_count
                            continue
                        if n < 0:
                            n += 1
                            continue
                        if is_occupied(real_map[m][n]) == False:
                            scaled_map[m][n] = map_cell_value
                        n += 1
                        if n > j + j_count:
                            n = j - j_count
                            m += 1
                except IndexError:
                    pass

    return scaled_map



def euclid_dist(a, b):
    """
    Calculate the Euclidean distance between two points
    :param a: The first point
    :param b: The second point
    :return: The Euclidean distance between points a and b
    """
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def is_occupied(cell_value):
    """
    Function to check if a map grid (cell) is occupied
    :param cell_value: The value of the map cell
    :return: True if occupied, False if free, None if unknown
    """
    if cell_value >= 0 and (2 * cell_value - 100) > 0:
        return True
    elif cell_value >= 0 and (2 * cell_value - 100) < 0:
        return False
    else:
        return None