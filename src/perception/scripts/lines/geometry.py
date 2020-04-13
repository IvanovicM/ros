import math 

from geometry_msgs.msg import Point

def line2polar(line):
    rho = distance_from_line(line[0], line[1], Point(0, 0, 0))
    alpha = get_alpha(line[0], line[1], rho)
    return rho, alpha

def polar2xyz(rho, alpha):
    x = rho * math.cos(alpha)
    y = rho * math.sin(alpha)
    return Point(x=x, y=y, z=0)

def get_alpha(a, b, rho, eps=0.05):
    if abs(a.y - b.y) < eps:
        return math.pi/2 if a.y > 0 else -math.pi/2
    if abs(a.x - b.x) < eps:
        return 0

    k = (a.x - b.x) / (a.y - b.y)
    n = a.y - k * a.x
    x = -n / (2*k)
    y = -k * x

    if x > 0:
        return math.atan(y / x)
    if y > 0:
        return math.atan(y / x) + math.pi
    return math.atan(y / x) - math.pi

def distance(a, b):
    # Distance |ab|
    return math.sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y))

def cross_product(a, b, c, d):
    # Cross product |ab| x |cd|
    return (b.x - a.x) * (d.y - c.y) - (d.x - c.x) * (b.y - a.y)

def distance_from_line(a, b, c):
    # Distance from point c to line ab
    return abs(cross_product(a, b, b, c) / distance(a, b))

def get_furthest_point(points, a, b):
    furthest_distance = 0.0
    furthest_point_idx = 0
    for idx in range(len(points)):
        dist = distance_from_line(a, b, points[idx])
        if dist > furthest_distance:
            furthest_distance = dist
            furthest_point_idx = idx

    return furthest_distance, furthest_point_idx