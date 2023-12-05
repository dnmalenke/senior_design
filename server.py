import numpy as np
import cv2
import sys
import socket
import json
import ui_helpers
from controlpacket import *
from dubins_path import plan_dubins_path

UDP_PORT = 1234
CAMERA_INDEX = 0
TEST_MODE = True

class Main():
    def init_network(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        ip = socket.gethostbyname(socket.gethostname())
        self.ip_start = '.'.join(ip.split('.')[:-1])

    def init_camera(self):
        if sys.platform.startswith('win32'):
            self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(CAMERA_INDEX)

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
        self.markers = {}
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict)

    def init_gui(self):
        cv2.namedWindow('frame')
        cv2.setMouseCallback('frame', self.handle_mouse)
        self.selected_box = -1
        self.selected_arrow = -1
        self.selected_point = -1
        self.destinations = {}
        self.running = {}
        
    def send_packet(self, id, left_speed, right_speed):
        packet = ControlPacket()
        packet.left_speed = left_speed
        packet.right_speed = right_speed

        jd = json.dumps(packet, default=vars)
        data = jd.encode()
        try:
            self.socket.sendto(data, (f"{self.ip_start}.{id}", UDP_PORT))
        except Exception as e:
            print(e)
            pass      

    def handle_tag(self, corners, id, frame):
        if id not in self.destinations:
            return
        
        end_x,end_y, end_yaw = self.destinations[id]
        x1,y1 = corners[0]
        x2,y2 = corners[1]
        x3,y3 = corners[2]
        x4,y4 = corners[3]

        cx = ((x1 + x2 + x4) / 3 + (x2 + x3 + x4) / 3) / 2
        cy = ((y1 + y2 + y4) / 3 + (y2 + y3 + y4) / 3) / 2

        centerpoint = (int(cx), int(cy))

        angle = np.arctan2((y2 - y3), (x2 - x3)) + np.pi/2

        start_x = float(centerpoint[0]) # [m]
        start_y = float(centerpoint[1]) # [m]
        start_yaw = angle  # [rad]

        curvature = 0.015
        tolerance = 80
        
        vectors = plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, tolerance)
        # print(f"{id}: {vectors}")

        ui_helpers.draw_path(frame, start_x, start_y, start_yaw, end_x, end_y, end_yaw, vectors, curvature, id,self.selected_arrow,self.selected_point)

        if self.running[id]:
            for dir,len in vectors:
                if len < 50 and dir != 'STOP':
                    continue
                match dir:
                    case 'L':
                        self.send_packet(id,-0.17,-0.08)
                    case 'S':
                        self.send_packet(id,-0.15,-0.15)
                    case 'R':
                        self.send_packet(id,-0.08,-0.17)
                    case 'STOP':
                        self.send_packet(id,0,0)
                break
        else:
            self.send_packet(id,0,0)
        
        
    def handle_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:              
            arrlen = 75
            arrwid = 8
            for id,(end_x,end_y,end_yaw) in self.destinations.items():
                if(np.sqrt((x-end_x)**2 + (y-end_y)**2) < 30):
                    self.selected_point = id
                else:
                    perp_yaw = end_yaw + np.pi/2
                    c2 = (arrwid/2 * np.cos(perp_yaw) + end_x, arrwid/2 * np.sin(perp_yaw)+ end_y)
                    c3 = (-arrwid/2 * np.cos(perp_yaw) + end_x, -arrwid/2 * np.sin(perp_yaw) + end_y)
                    arr_x = arrlen * np.cos(end_yaw) + end_x
                    arr_y = arrlen * np.sin(end_yaw) + end_y 
                    c1 = (arrwid/2 * np.cos(perp_yaw) + arr_x, arrwid/2 * np.sin(perp_yaw)+ arr_y)
                    c4 = (-arrwid/2 * np.cos(perp_yaw) + arr_x, -arrwid/2 * np.sin(perp_yaw) + arr_y)

                    if(ui_helpers.is_point_inside_quadrilateral((x,y),[c1, c2, c3, c4])):
                        self.selected_arrow = id  
            
        if event == cv2.EVENT_MOUSEMOVE:
            if self.selected_arrow != -1 and self.selected_arrow in self.destinations: 
                end_x, end_y, _ = self.destinations[self.selected_arrow]
                end_yaw = np.arctan2((y-end_y),(x-end_x))
                self.destinations[self.selected_arrow] = (end_x,end_y,end_yaw)
            if self.selected_point != -1 and self.selected_point in self.destinations:
                corners = self.markers[self.selected_box]
                x1,y1 = corners[0]
                x2,y2 = corners[1]
                x3,y3 = corners[2]
                x4,y4 = corners[3]

                cx = ((x1 + x2 + x4) / 3 + (x2 + x3 + x4) / 3) / 2
                cy = ((y1 + y2 + y4) / 3 + (y2 + y3 + y4) / 3) / 2
                self.destinations[self.selected_point] = (x,y,np.arctan2((y-cy), (x-cx)))

        if event == cv2.EVENT_LBUTTONUP:
            self.selected_point = -1

            for id,corners in self.markers.items():
                if(ui_helpers.is_point_inside_quadrilateral((x,y),corners)):
                    self.selected_box = id
                    return
                
            if(self.selected_box != -1 and self.selected_arrow == -1 and self.selected_box in self.markers):
                corners = self.markers[self.selected_box]
                
                if ui_helpers.inside_button(x,y):
                    self.running[self.selected_box] = not self.running[self.selected_box]
                else:
                    if not ui_helpers.is_point_inside_quadrilateral((x,y),corners):
                        x1,y1 = corners[0]
                        x2,y2 = corners[1]
                        x3,y3 = corners[2]
                        x4,y4 = corners[3]

                        cx = ((x1 + x2 + x4) / 3 + (x2 + x3 + x4) / 3) / 2
                        cy = ((y1 + y2 + y4) / 3 + (y2 + y3 + y4) / 3) / 2

                        self.destinations[self.selected_box] = (x,y,np.arctan2((y-cy), (x-cx)))
                    elif self.selected_box in self.destinations:
                        del(self.destinations[self.selected_box])
            self.selected_arrow = -1
            

    def main(self):
        self.init_network()     
        if not TEST_MODE:
            self.init_camera()
        self.init_aruco()
        self.init_gui()

        while True:
            if TEST_MODE:
                frame = cv2.imread('test.png')
            else:
                _ , frame = self.cap.read()

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners,ids, _ = self.aruco_detector.detectMarkers(gray_frame)

            if np.all(ids != None):
                for id,corners in zip(ids,corners):
                    car_id = int(id[0])
                    self.markers[car_id] = corners[0]
                    self.handle_tag(corners[0], car_id, frame)
                    ui_helpers.draw_tag(car_id, corners[0], frame, self.selected_box)

                    if car_id not in self.running:
                        self.running[car_id] = False
            
            if self.selected_box != -1:
                ui_helpers.draw_buttons(frame, self.running[self.selected_box])
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
        pass    


if __name__ == "__main__":
    Main().main()