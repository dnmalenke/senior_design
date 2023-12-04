import numpy as np
import cv2

# Create a black background image
image = cv2.imread('test.png')

# Define rectangle coordinates
start_x1, start_y1 = 10, 10
start_x2, start_y2 = 110, 110

# Draw a green rectangle
cv2.rectangle(image, (start_x1, start_y1), (start_x2, start_y2), (0, 255, 0), cv2.FILLED)

# Add text "Start" to the rectangle
start_text = "Start"
start_text_size, _ = cv2.getTextSize(start_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
start_text_x = int(start_x1 + (start_x2 - start_x1 - start_text_size[0]) / 2)
start_text_y = int(start_y1 + (start_y2 - start_y1 + start_text_size[1]) / 2)
cv2.putText(image, start_text, (start_text_x, start_text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

stop_x1, stop_y1 = 120, 10
stop_x2, stop_y2 = 220, 110

cv2.rectangle(image, (stop_x1, stop_y1), (stop_x2, stop_y2), (0, 0, 255), cv2.FILLED)

stop_text = "Stop"
stop_text_size, _ = cv2.getTextSize(stop_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
stop_text_x = int(stop_x1 + (stop_x2 - stop_x1 - stop_text_size[0]) / 2)
stop_text_y = int(stop_y1 + (stop_y2 - stop_y1 + stop_text_size[1]) / 2)
cv2.putText(image, stop_text, (stop_text_x, stop_text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

# Display the image
cv2.imshow('Start Button', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
