import geometry

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

class LinesDetector():

    def __init__(self):
        self.threshold = 0.1
        self.detection_period = 10
        self.counter = -1

    def detect_lines(self, laser_scan):
        self.counter = (self.counter + 1) % self.detection_period
        if self.counter != 0:
            return None, None

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
        line_idx = self._split(points, 0, [[0, len(points) - 1]])
        lines = self._merge(points, line_idx)

        return lines

    def _split(self, points, start_idx, line_idx):
        if len(points) < 2:
            return line_idx
        a = points[0]
        b = points[-1]

        furthest_distance, furthest_point_idx = (
            geometry.get_furthest_point(points, a, b)
        )
        if furthest_distance < self.threshold:
            # No need for splitting anymore
            return line_idx

        left_idx = self._split(
            points[0 : (furthest_point_idx+1)],
            0,
            [[0, furthest_point_idx]]
        )
        left_idx = [[x + start_idx, y + start_idx] for [x, y] in left_idx]
        right_idx = self._split(
            points[furthest_point_idx :],
            furthest_point_idx,
            [[furthest_point_idx, len(points) - 1]]
        )
        right_idx = [[x + start_idx, y + start_idx] for [x, y] in right_idx]
        
        return left_idx + right_idx

    def _merge(self, points, line_idx):
        if line_idx is None or len(line_idx) < 1:
            return None

        lines = []
        prev_idx = line_idx[0]

        for i in range(1, len(line_idx)):
            next_idx = line_idx[i]
            if self._should_merge(prev_idx, next_idx, points):
                prev_idx = [prev_idx[0], next_idx[1]]
            else:
                new_line = [points[prev_idx[0]], points[prev_idx[1]]]
                lines.append(new_line)
                prev_idx = next_idx

        new_line = [points[prev_idx[0]], points[prev_idx[1]]]
        lines.append(new_line)
        return lines

    def _should_merge(self, prev_idx, next_idx, points):
        furthest_distance, furthest_point_idx = (
            geometry.get_furthest_point(
                points[prev_idx[0]:next_idx[1]+1],
                points[prev_idx[0]],
                points[next_idx[1]]
            )
        )
        if furthest_distance < self.threshold:
            return True
        return False