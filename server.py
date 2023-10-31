import numpy as np
import cv2
import sys
import socket
import json
from controlpacket import *
from dubins_path import plan_dubins_path

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
        try:
            self.socket.sendto(data,(f"{self.ip_start}.{id}", UDP_PORT))
        except Exception as e:
            print(e)
            pass

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
        angle += np.pi/2
        arrlen = 75
        arrowpoint = (int(arrlen * np.cos(angle) + cx), int(arrlen * np.sin(angle) + cy))

        cv2.arrowedLine(frame, centerpoint, arrowpoint, (255, 0, 255), 5)
        cv2.putText(frame, f"location: {centerpoint}. angle: {np.round(np.rad2deg(angle),2)} deg", (centerpoint[0], centerpoint[1] + 20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2)

        start_x = float(centerpoint[0]) # [m]
        start_y = float(centerpoint[1]) # [m]
        start_yaw = angle  # [rad]

        end_x = 300.0  # [m]
        end_y = 300.0  # [m]
        end_yaw = np.deg2rad(180)  # [rad]

        cv2.drawMarker(frame, (int(end_x),int(end_y)),(0, 255, 0),cv2.MARKER_CROSS,10,3)

        curvature = 0.008
        #x_list, y_list, yaw_list, b_d1, b_d2, b_d3, b_mode, lengths
        path_x, path_y, path_yaw, mode, vectors = plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
        print(vectors)

        for x,y in zip(path_x,path_y):
            cv2.drawMarker(frame, (int(x),int(y)),(255, 0, 0),cv2.MARKER_SQUARE,2,1)

        for dir,len in vectors:
            if len < 30:
                continue
            match dir:
                case 'L':
                    self.send_packet(id,0.05,-0.3)
                case 'S':
                    self.send_packet(id,0.2,-0.2)
                case 'R':
                    self.send_packet(id,0.3,0.05)
                case 'STOP':
                    self.send_packet(id,0,0)
            break
        pass

    # Jax
    def handle_mouse(self, event, x, y, flags, param):
        
        pass

    def main(self):
        self.init_network()        
        self.init_camera()
        self.init_aruco()
        # self.init_gui

        

        while True:
            ret, frame = self.cap.read()

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = self.aruco_detector.detectMarkers(gray_frame)
            cv2.aruco.drawDetectedMarkers(frame,corners,ids)
            

            if np.all(ids != None):
                for i in range(ids.size):
                    self.handle_tag(corners[i], int(ids[i]), frame)

            
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
        pass    


if __name__ == "__main__":
    Main().main()