from math import pi
from laser_line_extraction.msg import LineSegment

class GlobalMap():

    def __init__(self):
        self._get_walls_map()

    def _get_walls_map(self):
        self.walls = []

        # Wall 1
        line_segment = LineSegment(
            radius=0.3, angle=-pi, start=[-0.3, 3.2], end=[-0.3, 2]
        )
        self.walls.append(line_segment)

        # Wall 2
        line_segment = LineSegment(
            radius=2, angle=pi/2, start=[-0.3, 2], end=[2, 2]
        )
        self.walls.append(line_segment)

        # Wall 3
        line_segment = LineSegment(
            radius=2, angle=0, start=[2, 2], end=[2, 0.8]
        )
        self.walls.append(line_segment)

        # Wall 4
        line_segment = LineSegment(
            radius=0.8, angle=pi/2, start=[2, 0.8], end=[3.2, 0.8]
        )
        self.walls.append(line_segment)

        # Wall 5
        line_segment = LineSegment(
            radius=3.2, angle=0, start=[3.2, 0.8], end=[3.2, -2]
        )
        self.walls.append(line_segment)

        # Wall 6
        line_segment = LineSegment(
            radius=2, angle=-pi/2, start=[3.3, -2], end=[-1.3, -2]
        )
        self.walls.append(line_segment)

        # Wall 7 
        line_segment = LineSegment(
            radius=1.3, angle=-pi, start=[-1.3, -2], end=[-1.3, -0.8]
        )
        self.walls.append(line_segment)

        # Wall 8
        line_segment = LineSegment(
            radius=0.8, angle=-pi/2, start=[-1.3, -0.8], end=[-2.6, -0.8]
        )
        self.walls.append(line_segment)

        # Wall 9
        line_segment = LineSegment(
            radius=2.6, angle=-pi, start=[-2.6, -0.8], end=[-2.6, 1.6]
        )
        self.walls.append(line_segment)

        # Wall 10
        line_segment = LineSegment(
            radius=1.6, angle=pi/2, start=[-2.6, 1.6], end=[-1.3, 1.6]
        )
        self.walls.append(line_segment)

        # Wall 11
        line_segment = LineSegment(
            radius=1.3, angle=-pi, start=[-1.3, 1.6], end=[-1.3, 2.6]
        )
        self.walls.append(line_segment)

        # Wall 12 xxx
        line_segment = LineSegment(
            radius=2.6, angle=pi/2, start=[-1.3, 2.6], end=[-0.1, 2.6]
        )
        self.walls.append(line_segment)

    def __getitem__(self, key):
        if isinstance(key, (int, long)):
            idx = int(key)
            if idx >= 0 and idx < len(self.walls):
                return self.walls[idx]
        return None
