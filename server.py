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

    def scaling_function(self, corners):
        x1 = corners.item(0)
        y1 = corners.item(1)

        x2 = corners.item(2)
        y2 = corners.item(3)
    
        x3 = corners.item(4)
        y3 = corners.item(5)
    
        x4 = corners.item(6)
        y4 = corners.item(7)

        len1 = x2 - x1
        len2 = y2 - y3
        len3 = x3 - x4
        len4 = y1 - y4
        avg_len = (len1+len2+len3+len4)/4

        factor = avg_len / 2.2

        return factor 

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