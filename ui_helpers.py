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