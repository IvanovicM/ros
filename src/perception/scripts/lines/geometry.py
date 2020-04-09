import math 

from geometry_msgs.msg import Point

def polar2xyz(rho, alpha):
    x = rho * math.cos(alpha)
    y = rho * math.sin(alpha)
    return Point(x=x, y=y, z=0)

def distance(a, b):
    # Distance |ab|
    return math.sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y))

def cross_product(a, b, c):
    # Cross product |ab| x |bc|
    # >0 - left turn
    # <0 - right turn
    # =0 - collinear
    return (b.x - a.x) * (c.y - b.y) - (c.x - b.x) * (b.y - a.y)

def distance_from_line(a, b, c):
    # Distance from point c to line ab
    return abs(cross_product(a, b, c) / distance(a, b))

def get_furthest_point(points, a, b):
    furthest_distance = 0.0
    furthest_point_idx = 0
    for idx in range(len(points)):
        dist = distance_from_line(a, b, points[idx])
        if dist > furthest_distance:
            furthest_distance = dist
            furthest_point_idx = idx

    return furthest_distance, furthest_point_idx