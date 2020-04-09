import math

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

class LinesDetector():

    def detect_lines(self, laser_scan):
        ranges = self._preprocess_ranges(
            laser_scan.ranges, laser_scan.range_min, laser_scan.range_max
        )
        points = self._get_points(ranges, laser_scan)
        lines = self._split_n_merge(points)
        return points, lines

    def _preprocess_ranges(self, ranges, range_min, range_max):
        processes_ranges = []
        for i in range(len(ranges)):
            if ranges[i] < range_min or ranges[i] > range_max:
                processes_ranges.append(None)
            else:
                processes_ranges.append(ranges[i])
        return processes_ranges

    def _get_points(self, ranges, laser_scan):
        points = []
        for i in range(len(ranges)):
            if ranges[i] is not None:
                new_point = self._polar2xyz(
                    ranges[i],
                    i * laser_scan.angle_increment + laser_scan.angle_min
                )
                points.append(new_point)

        if len(points) == 0:
            return None
        return points

    def _polar2xyz(self, rho, alpha):
        x = rho * math.cos(alpha)
        y = rho * math.sin(alpha)
        return Point(x=x, y=y, z=0)

    def _split_n_merge(self, points):
        # TODO
        return points