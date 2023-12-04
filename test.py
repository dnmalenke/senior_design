import numpy as np
import cv2

# another function that checks if a point is inside a quadrilateral
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

# mouse callback function
def handle_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            global mouseX, mouseY
            cv2.circle(img ,(x,y),10,(255,0,0),-1)
            mouseX,mouseY = x,y
            point5 = mouseX, mouseY
            print("Coordinates: (" + str(mouseX) + ", " + str(mouseY) + ")")
            index = 1
            for box in corners:
                quad = [(box[0],box[1]), (box[2], box[3]), (box[4], box[5]), (box[6], box[7])]
                result = is_point_inside_quadrilateral(point5, quad)

                if result:
                    global current_box
                    current_box = index
                    print("Box " + str(index) + " has been selected.")
                    print("Please select an endpoint.")

                index += 1

# Create a  blank image, a window, and bind the function to window
img = cv2.imread('test.png')

cv2.namedWindow('image')

corners = [[0,0, 0,100, 100,100, 100,0], [200,0, 200,100, 300,100, 300,0], [400,0, 400,100, 500,100, 500,0], [600,50, 650,0, 700,50, 650,100]]

# write function to print if a click is contained within within a box, and print which box was clicked in

for box in corners:
    x1 = box[0]
    y1 = box[1]

    x2 = box[2]
    y2 = box[3]

    x3 = box[4]
    y3 = box[5]

    x4 = box[6]
    y4 = box[7]

    cv2.line(img, (x1, y1), (x2, y2), (0,255,0), 5)
    cv2.line(img, (x2, y2), (x3, y3), (0,255,0), 5)
    cv2.line(img, (x3, y3), (x4, y4), (0,255,0), 5)
    cv2.line(img, (x4, y4), (x1, y1), (0,255,0), 5)


cv2.setMouseCallback('image', handle_mouse)

while True:
    cv2.imshow('image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
