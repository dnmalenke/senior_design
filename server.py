import numpy as np
import cv2
import sys
import socket
import json
from controlpacket import *
from math import sin, cos, atan2, sqrt, acos, pi, hypot
from utils import angle_mod, rot_mat_2d


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

    # Vishall
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

    def compute_dubins_path(self):
        
        # https://atsushisakai.github.io/PythonRobotics/modules/path_planning/dubins_path/dubins_path.html
        # https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DubinsPath/dubins_path_planner.py

        # steps:
        # calculate the length of the 6 possible paths
        # determine the shortest path
        # determine length of the each segment in the shortest path
        # if length negligible, skip
        # else send left/right wheel speeds according to segment type

        pass

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