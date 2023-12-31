import numpy as np
import cv2
import sys
import socket
import json
import ui_helpers
import math_helpers
from controlpacket import *
from dubins_path import plan_dubins_path


UDP_PORT = 1234
CAMERA_INDEX = 0
TEST_MODE = True

CURVATURE = 0.015
END_TOLERANCE = 125
STEP_TOLERANCE = 50

STRAIGHT_SPEED = -0.15
TURN_SLOW_SPEED = -0.08
TURN_FAST_SPEED = -0.2

class Main():
    def init_network(self):
        """
        Initializes the shared socket for transmitting packets to the vehicle and obtains the first three octets of the ip address for transmission to vehicles by id (fourth octet)
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        ip = socket.gethostbyname(socket.gethostname())
        self.ip_start = '.'.join(ip.split('.')[:-1])

    def init_camera(self):
        """
        Initializes the camera and imaging settings. On Windows, the DSHOW driver needs to be used to provide usable performance.
        """
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
        """
        Initializes the ArucoDetector object.
        """
        self.markers = {}
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict)

    def init_gui(self):
        """
        Initializes variables for the gui
        """
        cv2.namedWindow('frame')
        cv2.setMouseCallback('frame', self.handle_mouse)
        self.selected_box = -1
        self.selected_arrow = (-1,0)
        self.selected_point = (-1,0)
        self.destinations = {}
        self.running = {}
        
    def send_packet(self, id, left_speed, right_speed):
        """
        Sends a control packet to a vehicle based on id. Requires init_network to be run first.
        """
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
        """
        Every frame when new vehicle information is gathered, this function is called. 
        """

        # if there are no destinations for the vehicle, return
        if id not in self.destinations:
            return
        
        # get the path step number from the vehicle's destinations and resetting back to zero if needed
        step = self.destinations[id][0]

        if step >= len(self.destinations[id][1]):
            step = 0
            return 
        
        end_x, end_y, end_yaw = self.destinations[id][1][step]

        start_x, start_y = math_helpers.calculate_center(corners)

        x2, y2 = corners[1]
        x3, y3 = corners[2]
        angle = np.arctan2((y2 - y3), (x2 - x3)) + np.pi/2 # calculate angle and rotate to match tag orientation on vehicle

        start_yaw = angle  # radians

        # compute and draw the current path        
        vectors = plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, CURVATURE, END_TOLERANCE)

        ui_helpers.draw_path(frame, start_x, start_y, start_yaw, end_x, end_y, end_yaw, vectors, CURVATURE, (id, step - 1), self.selected_arrow,self.selected_point)

        # if the vehicle has been started, send commands to it based on the output of the planned path
        if self.running[id]:
            for dir,length in vectors:
                if length < STEP_TOLERANCE and dir != 'STOP':
                    continue
                match dir:
                    case 'L':
                        self.send_packet(id, TURN_FAST_SPEED, TURN_SLOW_SPEED)
                    case 'S':
                        self.send_packet(id, STRAIGHT_SPEED, STRAIGHT_SPEED)
                    case 'R':
                        self.send_packet(id, TURN_SLOW_SPEED, TURN_FAST_SPEED)
                    case 'STOP':
                        self.send_packet(id, 0, 0)
                        step = step + 1
                        if step >= len(self.destinations[id][1]):
                            step = 0
                        self.destinations[id][0] = step
                break
        else:
            self.send_packet(id,0,0)

        # Draw the rest of the multi-step path
        idx = step
        for (ex,ey,eya) in self.destinations[id][1][step:]:
            start_x = end_x
            start_y = end_y
            start_yaw = end_yaw
            end_x = ex
            end_y = ey
            end_yaw = eya
            vectors = plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, CURVATURE, END_TOLERANCE)
            ui_helpers.draw_path(frame, start_x, start_y, start_yaw, end_x, end_y, end_yaw, vectors, CURVATURE, (id,idx),self.selected_arrow,self.selected_point)
            idx = idx + 1

    
    def handle_mouse(self, event, x, y, flags, param):
        """
        Called on any mouse event in the camera window
        """

        # Determines the selected point or arrow when mouse clicked down
        if event == cv2.EVENT_LBUTTONDOWN:              
            arrlen = 75
            arrwid = 8
            for id,(_,paths) in self.destinations.items():
                step = 0
                for(end_x,end_y,end_yaw) in paths:
                    if(np.sqrt((x-end_x)**2 + (y-end_y)**2) < 30):
                        self.selected_point = (id,step)
                    else:
                        perp_yaw = end_yaw + np.pi/2
                        c2 = (arrwid/2 * np.cos(perp_yaw) + end_x, arrwid/2 * np.sin(perp_yaw)+ end_y)
                        c3 = (-arrwid/2 * np.cos(perp_yaw) + end_x, -arrwid/2 * np.sin(perp_yaw) + end_y)
                        arr_x = arrlen * np.cos(end_yaw) + end_x
                        arr_y = arrlen * np.sin(end_yaw) + end_y 
                        c1 = (arrwid/2 * np.cos(perp_yaw) + arr_x, arrwid/2 * np.sin(perp_yaw)+ arr_y)
                        c4 = (-arrwid/2 * np.cos(perp_yaw) + arr_x, -arrwid/2 * np.sin(perp_yaw) + arr_y)

                        if(ui_helpers.is_point_inside_quadrilateral((x,y),[c1, c2, c3, c4])):
                            self.selected_arrow = (id,step)  
                    step = step + 1
            
        # moves the path endpoint if a point is selected and dragged or changes the target angle if an arrow is selected and dragged
        if event == cv2.EVENT_MOUSEMOVE:
            if self.selected_arrow[0] != -1 and self.selected_arrow[0] in self.destinations: 
                end_x, end_y, _ = self.destinations[self.selected_arrow[0]][1][self.selected_arrow[1]]
                end_yaw = np.arctan2((y-end_y),(x-end_x))
                self.destinations[self.selected_arrow[0]][1][self.selected_arrow[1]] = (end_x,end_y,end_yaw)
            if self.selected_point[0] != -1 and self.selected_point[0] in self.destinations:
                corners = self.markers[self.selected_box]
                cx, cy = math_helpers.calculate_center(corners)
                if self.selected_point[1] > 0:
                    cx, cy, _ = self.destinations[self.selected_point[0]][1][self.selected_point[1]-1]
                self.destinations[self.selected_point[0]][1][self.selected_point[1]] = (x,y,np.arctan2((y-cy), (x-cx)))

        # handles appending new points to the path
        if event == cv2.EVENT_LBUTTONUP:
            if self.selected_point[0] != -1:
                self.selected_point = (-1,0)
                return

            for id,corners in self.markers.items():
                if(ui_helpers.is_point_inside_quadrilateral((x,y),corners)):
                    self.selected_box = id
                    return
                
            if(self.selected_box != -1 and self.selected_arrow[0] == -1 and self.selected_box in self.markers):
                corners = self.markers[self.selected_box]
                
                if ui_helpers.inside_button(x,y):
                    self.running[self.selected_box] = not self.running[self.selected_box]
                else:
                    if not ui_helpers.is_point_inside_quadrilateral((x,y),corners):
                        cx,cy = math_helpers.calculate_center(corners)
                        
                        if flags & cv2.EVENT_FLAG_SHIFTKEY > 0 and len(self.destinations[self.selected_box][1]) > 0:
                            cx, cy, _ = self.destinations[self.selected_box][1][-1]
                            self.destinations[self.selected_box][1].append((x,y,np.arctan2((y-cy), (x-cx))))
                        else:
                            self.destinations[self.selected_box][1] = [(x,y,np.arctan2((y-cy), (x-cx)))]
            self.selected_arrow = (-1,0)
            

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
                    if car_id not in self.destinations:
                        self.destinations[car_id] = [0,[]]
            
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