import geometry

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

class LinesDetector():

    def __init__(self):
        self.threshold = 0.1
        self.ind = False

    def detect_lines(self, laser_scan):
        self.ind = True
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
                new_point = geometry.polar2xyz(
                    ranges[i],
                    i * laser_scan.angle_increment + laser_scan.angle_min
                )
                points.append(new_point)

        if len(points) == 0:
            return None
        return points

    def _split_n_merge(self, points):
        if points is None or len(points) < 2:
            return []
        lines = self._split(points, [[points[0], points[-1]]])
        lines = self._merge(lines)

        return lines

    def _split(self, points, lines):
        if len(points) < 2:
            lines
        a = points[0]
        b = points[-1]

        furthest_distance, furthest_point_idx = (
            geometry.get_furthest_point(points, a, b)
        )

        if furthest_distance < self.threshold:
            # No need for splitting anymore
            return lines

        left_lines = self._split(
            points[0 : (furthest_point_idx+1)],
            [[a, points[furthest_point_idx]]]
        )
        right_lines = self._split(
            points[furthest_point_idx :],
            [[points[furthest_point_idx], b]]
        )
        return left_lines + right_lines

    def _merge(self, lines):
        return lines