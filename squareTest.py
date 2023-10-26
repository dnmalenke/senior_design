import numpy as np
import cv2

def is_point_inside_square(p1, p2, p3, p4, fifth_point):
    # Define vectors for the sides of the square
    side1 = (p2[0] - p1[0], p2[1] - p1[1])
    side2 = (p3[0] - p2[0], p3[1] - p2[1])
    side3 = (p4[0] - p3[0], p4[1] - p3[1])
    side4 = (p1[0] - p4[0], p1[1] - p4[1])

    # Vectors from the first point of the square to the fifth point
    vector1 = (fifth_point[0] - p1[0], fifth_point[1] - p1[1])
    vector2 = (fifth_point[0] - p2[0], fifth_point[1] - p2[1])
    vector3 = (fifth_point[0] - p3[0], fifth_point[1] - p3[1])
    vector4 = (fifth_point[0] - p4[0], fifth_point[1] - p4[1])

    # Calculate the cross product of each vector with its corresponding side vector
    cross_product1 = vector1[0] * side1[1] - vector1[1] * side1[0]
    cross_product2 = vector2[0] * side2[1] - vector2[1] * side2[0]
    cross_product3 = vector3[0] * side3[1] - vector3[1] * side3[0]
    cross_product4 = vector4[0] * side4[1] - vector4[1] * side4[0]

    # If all cross products have the same sign, the fifth point is inside the square
    if (cross_product1 > 0 and cross_product2 > 0 and cross_product3 > 0 and cross_product4 > 0) or \
       (cross_product1 < 0 and cross_product2 < 0 and cross_product3 < 0 and cross_product4 < 0):
        return True
    else:
        return False

# Test
p1 = (1, 1)
p2 = (4, 1)
p3 = (4, 4)
p4 = (1, 4)
fifth_point = (2, 2)

result = is_point_inside_square(p1, p2, p3, p4, fifth_point)
print(result)
