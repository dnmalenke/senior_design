def calculate_center(corners) -> (float, float):
        x1,y1 = corners[0]
        x2,y2 = corners[1]
        x3,y3 = corners[2]
        x4,y4 = corners[3]

        cx = ((x1 + x2 + x4) / 3 + (x2 + x3 + x4) / 3) / 2
        cy = ((y1 + y2 + y4) / 3 + (y2 + y3 + y4) / 3) / 2
        return (cx, cy)