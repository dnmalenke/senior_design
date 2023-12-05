import numpy as np
import cv2
import sys
import socket
import json
import ui_helpers
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
        
    # David
    def send_packet(self, id, left_speed, right_speed):
        packet = ControlPacket()
        packet.left_speed = left_speed
        packet.right_speed = right_speed

        jd = json.dumps(packet, default=vars)
        # print(jd)
        data = jd.encode()
        try:
            self.socket.sendto(data, (f"{self.ip_start}.{id}", UDP_PORT))
        except Exception as e:
            print(e)
            pass

    def draw_path(self, frame, start_x, start_y, start_yaw, end_x, end_y, end_yaw, vectors, curvature, id):
        x1, y1, a1 = start_x, start_y, start_yaw
        radius = 1 / curvature

        for dir, length in vectors:
            if dir == 'S':
                x2 = length * np.cos(a1) + x1
                y2 = length * np.sin(a1) + y1

                cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)

                x1, y1 = x2, y2

            else:
                if dir == 'R':
                    start_angle = a1 - np.pi/2
                    base_angle = 90
                    end_angle = -length / radius
                elif dir == 'L':
                    start_angle = a1 + np.pi/2
                    base_angle = -90
                    end_angle = length / radius
                else:
                    continue

                x_center = x1 + radius * np.cos(start_angle)
                y_center = y1 + radius * np.sin(start_angle)

                cv2.ellipse(frame, (int(x_center), int(y_center)), (int(radius), int(radius)),
                            np.rad2deg(a1) + base_angle, 0, np.rad2deg(end_angle), (255, 0, 0), 2)

                x1 = x_center + radius * np.cos(end_angle + a1 + np.deg2rad(base_angle))
                y1 = y_center + radius * np.sin(end_angle + a1 + np.deg2rad(base_angle))

                a1 = end_angle + a1

        arrlen = 75
        arrowpoint = (int(arrlen * np.cos(end_yaw) + end_x), int(arrlen * np.sin(end_yaw) + end_y))

        if id == self.selected_arrow:
            arrow_color = (255,0,255)
        else:
            arrow_color = (255,255,0)

        if id == self.selected_point:
            point_color = (255,0,255)
        else:
            point_color = (0,255,0)

        cv2.arrowedLine(frame, (int(end_x), int(end_y)), arrowpoint, arrow_color, 3)
        cv2.drawMarker(frame, (int(end_x), int(end_y)), point_color, cv2.MARKER_CROSS, 10, 3)        

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

        self.draw_path(frame, start_x, start_y, start_yaw, end_x, end_y, end_yaw, vectors, curvature, id)

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
        
    def draw_tag(self, id, corners, frame):
        x1,y1 = corners[0]
        x2,y2 = corners[1]
        x3,y3 = corners[2]
        x4,y4 = corners[3]

        if id == self.selected_box:
            color = (0,255,0)
        else:
            color = (0,0,255)

        cv2.line(frame,(int(x1),int(y1)),(int(x2),int(y2)),color,1)
        cv2.line(frame,(int(x2),int(y2)),(int(x3),int(y3)),color,1)
        cv2.line(frame,(int(x3),int(y3)),(int(x4),int(y4)),color,1)
        cv2.line(frame,(int(x4),int(y4)),(int(x1),int(y1)),color,1)

        cx = ((x1 + x2 + x4) / 3 + (x2 + x3 + x4) / 3) / 2
        cy = ((y1 + y2 + y4) / 3 + (y2 + y3 + y4) / 3) / 2

        centerpoint = (int(cx), int(cy))

        angle = np.arctan2((y2 - y3), (x2 - x3)) + np.pi/2

        arrlen = 75
        arrowpoint = (int(arrlen * np.cos(angle) + cx), int(arrlen * np.sin(angle) + cy))

        cv2.arrowedLine(frame, centerpoint, arrowpoint, (255,0,255), 1)
        cv2.putText(frame, f"id: {id}. location: {centerpoint}. angle: {np.round(np.rad2deg(angle),2)} deg", (centerpoint[0], centerpoint[1] + 20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2)    

    def draw_start_stop_buttons(self, frame):
        if self.selected_box != -1:
            button_x1, button_y1 = 10, 10
            button_x2, button_y2 = 110, 110

            button_color = (0, 0, 255) if self.running[self.selected_box] else (0, 255, 0)

            cv2.rectangle(frame, (button_x1, button_y1), (button_x2, button_y2), button_color, cv2.FILLED)

            button_text = "Stop" if self.running[self.selected_box] else "Start"
            button_text_size, _ = cv2.getTextSize(button_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
            button_text_x = int(button_x1 + (button_x2 - button_x1 - button_text_size[0]) / 2)
            button_text_y = int(button_y1 + (button_y2 - button_y1 + button_text_size[1]) / 2)
            cv2.putText(frame, button_text, (button_text_x, button_text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    def inside_button(self, x, y):
        return ui_helpers.is_point_inside_quadrilateral((x,y), [(10,10),(110, 10),(110,110),(10,110)])
        
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
                
                if self.inside_button(x,y):
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
        # self.init_camera()
        self.init_aruco()
        self.init_gui()

        while True:
            # ret, frame = self.cap.read()
            frame = cv2.imread('test.png')

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners,ids, _ = self.aruco_detector.detectMarkers(gray_frame)

            if np.all(ids != None):
                for id,corners in zip(ids,corners):
                    car_id = int(id[0])
                    self.markers[car_id] = corners[0]
                    self.handle_tag(corners[0], car_id, frame)
                    self.draw_tag(car_id, corners[0], frame)

                    if car_id not in self.running:
                        self.running[car_id] = False
            
            self.draw_start_stop_buttons(frame)
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
        pass    


if __name__ == "__main__":
    Main().main()