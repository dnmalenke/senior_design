import cv2
import numpy as np

def is_point_inside_quadrilateral(point, quadrilateral):
    # Ensure that the quadrilateral is defined by four points (vertices).
    if len(quadrilateral) != 4:
        raise ValueError("Quadrilateral must be defined by exactly four points.")

    x, y = point
    crossings = 0
    for i in range(4):
        x1, y1 = quadrilateral[i]
        x2, y2 = quadrilateral[(i + 1) % 4]
        
        # Check if the point is on the edge of the quadrilateral.
        if (x, y) == (x1, y1) or (x, y) == (x2, y2):
            return True
        
        # Check if the point is on a horizontal line coinciding with an edge.
        if y1 == y2 and y == y1 and min(x1, x2) <= x <= max(x1, x2):
            return True
        
        # Check if the point is above or below the edge.
        if min(y1, y2) < y <= max(y1, y2) and x < max(x1, x2):
            if x1 == x2:
                # Edge is vertical.
                if x1 > x:
                    crossings += 1
            else:
                # Calculate the x-coordinate where the edge crosses the horizontal line.
                x_cross = (y - y1) * (x2 - x1) / (y2 - y1) + x1
                if x < x_cross:
                    crossings += 1
    
    return crossings % 2 == 1

def draw_path(frame, start_x, start_y, start_yaw, end_x, end_y, end_yaw, vectors, curvature, id, selected_arrow, selected_point):
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

        if id == selected_arrow:
            arrow_color = (255,0,255)
        else:
            arrow_color = (255,255,0)

        if id == selected_point:
            point_color = (255,0,255)
        else:
            point_color = (0,255,0)

        cv2.arrowedLine(frame, (int(end_x), int(end_y)), arrowpoint, arrow_color, 3)
        cv2.drawMarker(frame, (int(end_x), int(end_y)), point_color, cv2.MARKER_CROSS, 10, 3)  

def draw_buttons(frame, running):
    button_x1, button_y1 = 10, 10
    button_x2, button_y2 = 110, 110

    button_color = (0, 0, 255) if running else (0, 255, 0)

    cv2.rectangle(frame, (button_x1, button_y1), (button_x2, button_y2), button_color, cv2.FILLED)

    button_text = "Stop" if running else "Start"
    button_text_size, _ = cv2.getTextSize(button_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
    button_text_x = int(button_x1 + (button_x2 - button_x1 - button_text_size[0]) / 2)
    button_text_y = int(button_y1 + (button_y2 - button_y1 + button_text_size[1]) / 2)
    cv2.putText(frame, button_text, (button_text_x, button_text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

def inside_button(x, y):
    return is_point_inside_quadrilateral((x,y), [(10,10),(110, 10),(110,110),(10,110)])

def draw_tag(id, corners, frame, selected_box):
    x1,y1 = corners[0]
    x2,y2 = corners[1]
    x3,y3 = corners[2]
    x4,y4 = corners[3]

    if id == selected_box:
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