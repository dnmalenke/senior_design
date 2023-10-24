import numpy as np
import cv2
import sys
import socket
import json
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from math import sin, cos, atan2, sqrt, acos, pi, hypot
from utils.angle import angle_mod, rot_mat_2d
from controlpacket import *

UDP_PORT = 1234

class Main():
    def __init__(self):
        self.camera_index = 0
        pass

    def init_network(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        ip = socket.gethostbyname(socket.gethostname())
        self.ip_start = '.'.join(ip.split('.')[:-1])

    def init_camera(self):
        if sys.platform.startswith('win32'):
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(self.camera_index)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 900)
        # self.cap.set(cv2.CAP_PROP_SETTINGS, 1)

        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -9) # 1/(2^-9) = 1/512 sec

        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.0)
        self.cap.set(cv2.CAP_PROP_SHARPNESS, 255.0)
        self.cap.set(cv2.CAP_PROP_GAIN, 1.0)

    def init_aruco(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict)

    def init_gui(self):
        cv2.setMouseCallback("frame", self.handle_mouse)

    def plan_dubins_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, curvature,
                        step_size=0.1, selected_types=None):
        """
        Plan dubins path parameters
        ----------
        s_x : float
            x position of the start point [m]
        s_y : float
            y position of the start point [m]
        s_yaw : float
            yaw angle of the start point [rad]
        g_x : float
            x position of the goal point [m]
        g_y : float
            y position of the end point [m]
        g_yaw : float
            yaw angle of the end point [rad]
        curvature : float
            curvature for curve [1/m]
        step_size : float (optional)
            step size between two path points [m]. Default is 0.1
        selected_types : a list of string or None
            selected path planning types. If None, all types are used for
            path planning, and minimum path length result is returned.
            You can select used path plannings types by a string list.
            e.g.: ["RSL", "RSR"]

        Returns
        -------
        x_list: array
            x positions of the path
        y_list: array
            y positions of the path
        yaw_list: array
            yaw angles of the path
        modes: array
            mode list of the path
        lengths: array
            arrow_length list of the path segments.

        """
        if selected_types is None:
            planning_funcs = _PATH_TYPE_MAP.values()
        else:
            planning_funcs = [_PATH_TYPE_MAP[ptype] for ptype in selected_types]

        # calculate local goal x, y, yaw
        l_rot = rot_mat_2d(s_yaw)
        le_xy = np.stack([g_x - s_x, g_y - s_y]).T @ l_rot
        local_goal_x = le_xy[0]
        local_goal_y = le_xy[1]
        local_goal_yaw = g_yaw - s_yaw

        lp_x, lp_y, lp_yaw, modes, lengths = _dubins_path_planning_from_origin(
            local_goal_x, local_goal_y, local_goal_yaw, curvature, step_size,
            planning_funcs)

        # Convert a local coordinate path to the global coordinate
        rot = rot_mat_2d(-s_yaw)
        converted_xy = np.stack([lp_x, lp_y]).T @ rot
        x_list = converted_xy[:, 0] + s_x
        y_list = converted_xy[:, 1] + s_y
        yaw_list = angle_mod(np.array(lp_yaw) + s_yaw)

        return x_list, y_list, yaw_list, modes, lengths


    def _mod2pi(theta):
        return angle_mod(theta, zero_2_2pi=True)


    def _calc_trig_funcs(alpha, beta):
        sin_a = sin(alpha)
        sin_b = sin(beta)
        cos_a = cos(alpha)
        cos_b = cos(beta)
        cos_ab = cos(alpha - beta)
        return sin_a, sin_b, cos_a, cos_b, cos_ab


    def _LSL(alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
        mode = ["L", "S", "L"]
        p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_a - sin_b))
        if p_squared < 0:  # invalid configuration
            return None, None, None, mode
        tmp = atan2((cos_b - cos_a), d + sin_a - sin_b)
        d1 = _mod2pi(-alpha + tmp)
        d2 = sqrt(p_squared)
        d3 = _mod2pi(beta - tmp)
        return d1, d2, d3, mode


    def _RSR(alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
        mode = ["R", "S", "R"]
        p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_b - sin_a))
        if p_squared < 0:
            return None, None, None, mode
        tmp = atan2((cos_a - cos_b), d - sin_a + sin_b)
        d1 = _mod2pi(alpha - tmp)
        d2 = sqrt(p_squared)
        d3 = _mod2pi(-beta + tmp)
        return d1, d2, d3, mode


    def _LSR(alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
        p_squared = -2 + d ** 2 + (2 * cos_ab) + (2 * d * (sin_a + sin_b))
        mode = ["L", "S", "R"]
        if p_squared < 0:
            return None, None, None, mode
        d1 = sqrt(p_squared)
        tmp = atan2((-cos_a - cos_b), (d + sin_a + sin_b)) - atan2(-2.0, d1)
        d2 = _mod2pi(-alpha + tmp)
        d3 = _mod2pi(-_mod2pi(beta) + tmp)
        return d2, d1, d3, mode


    def _RSL(alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
        p_squared = d ** 2 - 2 + (2 * cos_ab) - (2 * d * (sin_a + sin_b))
        mode = ["R", "S", "L"]
        if p_squared < 0:
            return None, None, None, mode
        d1 = sqrt(p_squared)
        tmp = atan2((cos_a + cos_b), (d - sin_a - sin_b)) - atan2(2.0, d1)
        d2 = _mod2pi(alpha - tmp)
        d3 = _mod2pi(beta - tmp)
        return d2, d1, d3, mode


    def _RLR(alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
        mode = ["R", "L", "R"]
        tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0
        if abs(tmp) > 1.0:
            return None, None, None, mode
        d2 = _mod2pi(2 * pi - acos(tmp))
        d1 = _mod2pi(alpha - atan2(cos_a - cos_b, d - sin_a + sin_b) + d2 / 2.0)
        d3 = _mod2pi(alpha - beta - d1 + d2)
        return d1, d2, d3, mode


    def _LRL(alpha, beta, d):
        sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
        mode = ["L", "R", "L"]
        tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (- sin_a + sin_b)) / 8.0
        if abs(tmp) > 1.0:
            return None, None, None, mode
        d2 = _mod2pi(2 * pi - acos(tmp))
        d1 = _mod2pi(-alpha - atan2(cos_a - cos_b, d + sin_a - sin_b) + d2 / 2.0)
        d3 = _mod2pi(_mod2pi(beta) - alpha - d1 + _mod2pi(d2))
        return d1, d2, d3, mode


    _PATH_TYPE_MAP = {"LSL": _LSL, "RSR": _RSR, "LSR": _LSR, "RSL": _RSL,
                    "RLR": _RLR, "LRL": _LRL, }


    def _dubins_path_planning_from_origin(end_x, end_y, end_yaw, curvature,
                                        step_size, planning_funcs):
        dx = end_x
        dy = end_y
        d = hypot(dx, dy) * curvature

        theta = _mod2pi(atan2(dy, dx))
        alpha = _mod2pi(-theta)
        beta = _mod2pi(end_yaw - theta)

        best_cost = float("inf")
        b_d1, b_d2, b_d3, b_mode = None, None, None, None

        for planner in planning_funcs:
            d1, d2, d3, mode = planner(alpha, beta, d)
            if d1 is None:
                continue

            cost = (abs(d1) + abs(d2) + abs(d3))
            if best_cost > cost:  # Select minimum length one.
                b_d1, b_d2, b_d3, b_mode, best_cost = d1, d2, d3, mode, cost

        lengths = [b_d1, b_d2, b_d3]
        x_list, y_list, yaw_list = _generate_local_course(lengths, b_mode,
                                                        curvature, step_size)

        lengths = [length / curvature for length in lengths]

        return x_list, y_list, yaw_list, b_mode, lengths


    def _interpolate(length, mode, max_curvature, origin_x, origin_y,
                    origin_yaw, path_x, path_y, path_yaw):
        if mode == "S":
            path_x.append(origin_x + length / max_curvature * cos(origin_yaw))
            path_y.append(origin_y + length / max_curvature * sin(origin_yaw))
            path_yaw.append(origin_yaw)
        else:  # curve
            ldx = sin(length) / max_curvature
            ldy = 0.0
            if mode == "L":  # left turn
                ldy = (1.0 - cos(length)) / max_curvature
            elif mode == "R":  # right turn
                ldy = (1.0 - cos(length)) / -max_curvature
            gdx = cos(-origin_yaw) * ldx + sin(-origin_yaw) * ldy
            gdy = -sin(-origin_yaw) * ldx + cos(-origin_yaw) * ldy
            path_x.append(origin_x + gdx)
            path_y.append(origin_y + gdy)

            if mode == "L":  # left turn
                path_yaw.append(origin_yaw + length)
            elif mode == "R":  # right turn
                path_yaw.append(origin_yaw - length)

        return path_x, path_y, path_yaw


    def _generate_local_course(lengths, modes, max_curvature, step_size):
        p_x, p_y, p_yaw = [0.0], [0.0], [0.0]

        for (mode, length) in zip(modes, lengths):
            if length == 0.0:
                continue

            # set origin state
            origin_x, origin_y, origin_yaw = p_x[-1], p_y[-1], p_yaw[-1]

            current_length = step_size
            while abs(current_length + step_size) <= abs(length):
                p_x, p_y, p_yaw = _interpolate(current_length, mode, max_curvature,
                                            origin_x, origin_y, origin_yaw,
                                            p_x, p_y, p_yaw)
                current_length += step_size

            p_x, p_y, p_yaw = _interpolate(length, mode, max_curvature, origin_x,
                                        origin_y, origin_yaw, p_x, p_y, p_yaw)

        return p_x, p_y, p_yaw


    # David
    def send_packet(self, id, left_speed, right_speed):
        packet = ControlPacket()
        packet.left_speed = left_speed
        packet.right_speed = right_speed

        jd = json.dumps(packet, default=vars)
        print(jd)
        data = jd.encode()
        self.socket.sendto(data,(f"{self.ip_start}.{id}", UDP_PORT))

    def handle_tag(self, corners, id, frame):
        x1 = corners.item(0)
        y1 = corners.item(1)

        x2 = corners.item(2)
        y2 = corners.item(3)
    
        x3 = corners.item(4)
        y3 = corners.item(5)
    
        x4 = corners.item(6)
        y4 = corners.item(7)

        cx = ((x1 + x2 + x4) / 3 + (x2 + x3 + x4) / 3) / 2
        cy = ((y1 + y2 + y4) / 3 + (y2 + y3 + y4) / 3) / 2

        centerpoint = (int(cx), int(cy))

        angle = np.arctan2((y2 - y3), (x2 - x3))
        arrlen = 75
        arrowpoint = (int(arrlen * np.cos(angle) + cx), int(arrlen * np.sin(angle) + cy))

        cv2.arrowedLine(frame, centerpoint, arrowpoint, (255, 0, 255), 5)
        pass

    # Jax
    def handle_mouse(event, x, y, flags, param):
        pass

    def main(self):
        self.init_network()        
        self.init_camera()
        self.init_aruco()
        # self.init_gui()


        while True:
            ret, frame = self.cap.read()

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = self.aruco_detector.detectMarkers(gray_frame)
            cv2.aruco.drawDetectedMarkers(frame,corners,ids)
            

            if np.all(ids != None):
                for i in range(ids.size):
                    self.handle_tag(corners[i], ids[i], frame)
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
        pass    


if __name__ == "__main__":
    Main().main()