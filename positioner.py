import numpy as np
from circle_fit import standardLSQ


class Positioner:

    def __init__(self):
        self.bus_id = -1
        self.page_id = -1
        self.current_position = [-1, -1]
        self.current_theta = -1
        self.current_phi = -1
        self.theta_arm_length = -1
        self.phi_arm_length = -1
        self.theta_center = [-1, -1]
        self.phi_center = [-1, -1]
        self.theta_hardstop = [-1, -1]
        self.phi_hardstop = [-1, -1]
        self.theta_max = -1
        self.phi_max = -1
        self.previous_positions = []
        self.theta_calib_positions = []
        self.phi_calib_positions = []
        self.window_x = [-1, -1]
        self.window_y = [-1, -1]

    def set_window(self):
        x_lower = int(self.current_position[0] - 25)
        x_upper = int(self.current_position[0] + 25)
        y_lower = int(self.current_position[1] - 25)
        y_upper = int(self.current_position[1] + 25)
        self.window_x = [x_lower, x_upper]
        self.window_y = [y_lower, y_upper]

    def compute_phi_arm_params(self):
        x, y, r, s = standardLSQ(self.phi_calib_positions)
        self.phi_center = [x, y]
        self.phi_arm_length = r
        return s
