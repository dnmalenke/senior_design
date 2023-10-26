import cv2
import numpy as np

# Global variables to keep track of selected quadrilateral and points.
'''quadrilaterals = [np.array([[0, 0], [0, 100], [100, 100], [100, 0]]),
                  np.array([[200, 0], [200, 100], [300, 100], [300, 0]]),
                  np.array([[400, 0], [400, 100], [500, 100], [500, 0]]),
                  np.array([[600, 50], [650, 0], [700, 50], [650, 100]])]'''
quadrilaterals = [np.array([[100, 100], [100, 200], [200, 200], [200, 100]]),
                  np.array([[300, 100], [300, 200], [400, 200], [400, 100]])]
selected_quadrilaterals = []  # Store the indices of selected quadrilaterals
starting_points = []  # Store the starting points for each selected quadrilateral
end_points = []  # Store the end points for each selected quadrilateral

def click_event(event, x, y, flags, param):
    global selected_quadrilaterals, starting_points, end_points

    if event == cv2.EVENT_LBUTTONDOWN:
        # Check if a quadrilateral was clicked.
        for i, quad in enumerate(quadrilaterals):
            if cv2.pointPolygonTest(quad, (x, y), False) >= 0:
                if i in selected_quadrilaterals:
                    # Deselect the quadrilateral.
                    selected_quadrilaterals.remove(i)
                else:
                    # Select the quadrilateral.
                    selected_quadrilaterals.append(i)
                    starting_points.append(np.mean(quad, axis=0).astype(int))
                return

        if selected_quadrilaterals:
            # If one or more quadrilaterals are selected, set the end point.
            end_points.append([x, y])

# Create an empty image.
image = cv2.imread('test.png')

# Create a window and set the callback function.
cv2.namedWindow("Quadrilateral Definition")
cv2.setMouseCallback("Quadrilateral Definition", click_event)

for box in quadrilaterals:
    c1 = box[0]
    c2 = box[1]
    c3 = box[2]
    c4 = box[3]


    cv2.line(image, c1, c2, (0,255,0), 5)
    cv2.line(image, c2, c3, (0,255,0), 5)
    cv2.line(image, c3, c4, (0,255,0), 5)
    cv2.line(image, c4, c1, (0,255,0), 5)

while True:
    img_copy = image.copy()

    # Draw selected quadrilaterals and display their numbers.
    for i, quad in enumerate(quadrilaterals):
        if i in selected_quadrilaterals:
            cv2.polylines(img_copy, [quad], isClosed=True, color=(0, 0, 255), thickness=2)
            cv2.putText(img_copy, str(i), (quad[0][0], quad[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Draw starting points for each selected quadrilateral.
    for starting_point in starting_points:
        cv2.circle(img_copy, tuple(starting_point), 5, (0, 255, 0), -1)

    # Draw end points for each selected quadrilateral.
    for end_point in end_points:
        cv2.circle(img_copy, tuple(end_point), 5, (0, 255, 0), -1)

    # Display the image.
    cv2.imshow("Quadrilateral Definition", img_copy)

    # Check for user input.
    key = cv2.waitKey(1) & 0xFF

    # Exit the program on 'q' key press.
    if key == ord('q'):
        break

# Release OpenCV resources.
cv2.destroyAllWindows()

# Return the list of selected starting and end points as NumPy arrays.
selected_points = list(zip(starting_points, end_points))
print("Quadrilateral Numbers and Points:")
for i, (start, end) in zip(selected_quadrilaterals, selected_points):
    print(f"Quadrilateral {i}: Start {start}, End {end}")
