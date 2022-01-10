import numpy as np
from sklearn.neighbors import NearestNeighbors


def iterative_closest_point(old_pose, old_points, new_points, max_iter=100, max_dist=0.12, max_translation_error=1e-3,
                            max_rotation_error=1e-6, min_point_pairs=10):
    """
    Function to calculate the current pose from an initial pose by using the point clouds at the different poses
    The algorithm for this function is derived from https://github.com/richardos/icp/blob/master/icp.py
    which is based on the paper "Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans"
    by F. Lu and E. Milios.
    :param old_pose: The initial pose
    :param old_points: The point cloud at the initial pose
    :param new_points: The point cloud at the current pose
    :param max_iter: The maximum number of iterations allowed
    :param max_dist: The maximum distance between two points for them to be considered a pair
    :param max_translation_error: The maximum allowable translation error
    :param max_rotation_error: The maximum allowable rotation error
    :param min_point_pairs: The minimum number of point pairs for a solution to be calculated
    :return: The current pose
    """

    neighbors = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(old_points)
    d_theta = 0
    dx_dy = np.array([0, 0])
    for i in range(max_iter):
        closest_point_pairs = []

        dists, indices = neighbors.kneighbors(new_points)
        for j in range(len(dists)):
            if dists[j][0] < max_dist:
                closest_point_pairs.append((new_points[j], old_points[indices[j][0]]))

        if len(closest_point_pairs) < min_point_pairs:
            break

        rotation_angle, translation_x, translation_y = get_point_transformation(closest_point_pairs)

        if rotation_angle is None or translation_x is None or translation_y is None:
            break

        rotation_matrix = np.array([[np.cos(rotation_angle), - np.sin(rotation_angle)],
                                    [np.sin(rotation_angle), np.cos(rotation_angle)]])
        d_theta += rotation_angle
        aligned_points = np.dot(new_points, rotation_matrix.T)
        aligned_points[:, 0] += translation_x
        aligned_points[:, 1] += translation_y

        dx_dy = np.dot(dx_dy, rotation_matrix)
        dx_dy[0] += translation_x
        dx_dy[1] += translation_y

        new_points = aligned_points

        if abs(rotation_angle) < max_rotation_error and abs(translation_x) < max_translation_error and \
                abs(translation_y) < max_translation_error:
            break

    new_pose = np.zeros(3)
    new_pose[0] = old_pose[0] + dx_dy[0]
    new_pose[1] = old_pose[1] + dx_dy[1]
    new_pose[2] = old_pose[2] + d_theta

    return new_pose


def get_point_transformation(point_pairs):
    """
    Function to calculate the transformation between two point clouds
    :param point_pairs: A list containing the pairing between points in the two point clouds
    :return: The angle, x-distance and y-distance between the two point clouds.
    """

    x_old_mean, y_old_mean, x_new_mean, y_new_mean, s_x_new_x_old, s_x_new_y_old, s_y_new_x_old,\
        s_y_new_y_old = 0, 0, 0, 0, 0, 0, 0, 0

    n = len(point_pairs)

    if n == 0:
        return None, None, None

    for pair in point_pairs:
        (x_new, y_new), (x_old, y_old) = pair

        x_old_mean += x_old
        y_old_mean += y_old
        x_new_mean += x_new
        y_new_mean += y_new

    x_old_mean /= n
    y_old_mean /= n
    x_new_mean /= n
    y_new_mean /= n

    for pair in point_pairs:
        (x_new, y_new), (x_old, y_old) = pair

        s_x_new_x_old += ((x_new - x_new_mean) * (x_old - x_old_mean))
        s_x_new_y_old += ((x_new - x_new_mean) * (y_old - y_old_mean))
        s_y_new_x_old += ((y_new - y_new_mean) * (x_old - x_old_mean))
        s_y_new_y_old += ((y_new - y_new_mean) * (y_old - y_old_mean))

    rotation_angle = np.arctan2(s_x_new_y_old - s_y_new_x_old, s_x_new_x_old + s_y_new_y_old)
    translation_x = x_old_mean - (x_new_mean * np.cos(rotation_angle) - y_new_mean * np.sin(rotation_angle))
    translation_y = y_old_mean - (x_new_mean * np.sin(rotation_angle) + y_new_mean * np.cos(rotation_angle))

    return rotation_angle, translation_x, translation_y








