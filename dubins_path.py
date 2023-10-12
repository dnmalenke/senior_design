from math import sin, cos, atan2, sqrt, acos, pi, hypot
import numpy as np
from utils.angle import angle_mod, rot_mat_2d

def mod2pi(theta):
    return angle_mod(theta, zero_2_2pi=True)


def calc_trig_funcs(alpha, beta):
    sin_a = sin(alpha)
    sin_b = sin(beta)
    cos_a = cos(alpha)
    cos_b = cos(beta)
    cos_ab = cos(alpha - beta)
    return sin_a, sin_b, cos_a, cos_b, cos_ab


def LSL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = calc_trig_funcs(alpha, beta)
    mode = ["L", "S", "L"]
    p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_a - sin_b))
    if p_squared < 0:  # invalid configuration
        return None, None, None, mode
    tmp = atan2((cos_b - cos_a), d + sin_a - sin_b)
    d1 = mod2pi(-alpha + tmp)
    d2 = sqrt(p_squared)
    d3 = mod2pi(beta - tmp)
    return d1, d2, d3, mode


def RSR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = calc_trig_funcs(alpha, beta)
    mode = ["R", "S", "R"]
    p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_b - sin_a))
    if p_squared < 0:
        return None, None, None, mode
    tmp = atan2((cos_a - cos_b), d - sin_a + sin_b)
    d1 = mod2pi(alpha - tmp)
    d2 = sqrt(p_squared)
    d3 = mod2pi(-beta + tmp)
    return d1, d2, d3, mode


def LSR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = calc_trig_funcs(alpha, beta)
    p_squared = -2 + d ** 2 + (2 * cos_ab) + (2 * d * (sin_a + sin_b))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    d1 = sqrt(p_squared)
    tmp = atan2((-cos_a - cos_b), (d + sin_a + sin_b)) - atan2(-2.0, d1)
    d2 = mod2pi(-alpha + tmp)
    d3 = mod2pi(-mod2pi(beta) + tmp)
    return d2, d1, d3, mode


def RSL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = calc_trig_funcs(alpha, beta)
    p_squared = d ** 2 - 2 + (2 * cos_ab) - (2 * d * (sin_a + sin_b))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    d1 = sqrt(p_squared)
    tmp = atan2((cos_a + cos_b), (d - sin_a - sin_b)) - atan2(2.0, d1)
    d2 = mod2pi(alpha - tmp)
    d3 = mod2pi(beta - tmp)
    return d2, d1, d3, mode


def RLR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = calc_trig_funcs(alpha, beta)
    mode = ["R", "L", "R"]
    tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = mod2pi(2 * pi - acos(tmp))
    d1 = mod2pi(alpha - atan2(cos_a - cos_b, d - sin_a + sin_b) + d2 / 2.0)
    d3 = mod2pi(alpha - beta - d1 + d2)
    return d1, d2, d3, mode


def LRL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = calc_trig_funcs(alpha, beta)
    mode = ["L", "R", "L"]
    tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (- sin_a + sin_b)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = mod2pi(2 * pi - acos(tmp))
    d1 = mod2pi(-alpha - atan2(cos_a - cos_b, d + sin_a - sin_b) + d2 / 2.0)
    d3 = mod2pi(mod2pi(beta) - alpha - d1 + mod2pi(d2))
    return d1, d2, d3, mode


_PATH_TYPE_MAP = {"LSL": LSL, "RSR": RSR, "LSR": LSR, "RSL": RSL,
                  "RLR": RLR, "LRL": LRL, }
class main():
    def compute_dubins_path(x_i, y_i, yaw_i, x_f, y_f, yaw_f, turn_rad):
        # https://atsushisakai.github.io/PythonRobotics/modules/path_planning/dubins_path/dubins_path.html
        # https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DubinsPath/dubins_path_planner.py

        # steps:
        # determine the shortest path
        dx = x_f
        dy = y_f
        d = hypot(dx, dy) * turn_rad

        theta = mod2pi(atan2(dy, dx))
        alpha = mod2pi(-theta)
        beta = mod2pi(yaw_f - theta)

        planning_funcs = _PATH_TYPE_MAP.values()
        best_cost = float("inf")
        b_d1, b_d2, b_d3, b_mode = None, None, None, None

        for planner in planning_funcs:
            d1, d2, d3, mode = planner(alpha, beta, d)
            if d1 is None:
                continue

            cost = (abs(d1) + abs(d2) + abs(d3))
            if best_cost > cost:  # Select minimum length one.
                b_d1, b_d2, b_d3, b_mode, best_cost = d1, d2, d3, mode, cost

        # determine length of the each segment in the shortest path
        # if length negligible, skip
        # else send left/right wheel speeds according to segment type

        pass