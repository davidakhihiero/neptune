import numpy as np


class DiffDriveController:
    # Class to create a controller object for the differential drive robot
    def __init__ (self, max_v, max_omega, min_v, min_omega):
        """
        Initialization Method
        :param max_v: The maximum linear velocity of the robot
        :param max_omega: The maximum linear velocity of the robot
        :param min_v: The minimum linear velocity of the robot
        :param min_omega: The minimum linear velocity of the robot
        """

        self.rho_tolerance = 0.1 # Maximum allowed distance from the goal position
        self.alpha_tolerance = 0.1 # Maximum allowed distance from the goal orientation

        # Controller gains
        self.k_rho = 0.3 
        self.k_alpha = 1.5
        self.k_beta = -0.1

        self.max_v = max_v
        self.max_omega = max_omega
        self.min_v = min_v
        self.min_omega = min_omega

    def compute_velocity(self, start, goal):
        """
        Method to compute the linear and angular velocity required to move from a start pose to a goal pose
        :param start: The start pose
        :param goal: The goal pose
        :return: The linear velocity, angular velocity and True if the goal has been reached else False
        """
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        theta = start[2]

        if dx < 0:
            theta += np.pi
            
        rho = np.sqrt((dx ** 2) + (dy ** 2))
        alpha = -theta + np.arctan2(dy, dx)
        beta = -theta - alpha

        if rho <= self.rho_tolerance: # and alpha <= self.alpha_tolerance:
            done = True
        else:
            done = False

        gains_matrix = np.array([[self.k_rho, 0, 0],
                                 [0, self.k_alpha, self.k_beta]])
        diff_matrix = np.array([rho,
                                alpha,
                                beta])

        velocities = np.dot(gains_matrix, diff_matrix)

        if not (-np.pi / 2 <= alpha <= np.pi / 2):
            velocities[0] = -velocities[0]

        sign_v = np.sign(velocities[0])
        sign_omega = np.sign(velocities[1])

        v = min(self.max_v, abs(velocities[0])) * sign_v
        omega = min(self.max_v, abs(velocities[1])) * sign_omega

        v = max(self.min_v, abs(v)) * sign_v
        omega = max(self.min_v, abs(omega)) * sign_omega

        if done:
            v, omega = 0, 0

        return v, omega, done




