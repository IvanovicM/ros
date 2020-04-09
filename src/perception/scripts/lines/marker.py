import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from lines.detector import LinesDetector
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class MarkerPublisher():

    def __init__(self, publisher):
        self.publisher = publisher
        self.max_points_num = 720
        self.max_lines_num = 50

    def publish(self, lines):
        if lines is None:
            return
            
        markers = []
        for i in range(len(lines)):
            marker = self._get_line_marker(i, lines[i])
            markers.append(marker)

        marker_array = MarkerArray(markers=markers)
        self.publisher.publish(marker_array)
    
    def _get_point_marker(self, id, position, a=1):
        marker = Marker(
            ns='scans',
            pose=Pose(position=position),
            action=0,
            type=2,
            id=id,
            color=ColorRGBA(g=1, a=a)
        )
        marker.header.frame_id = 'base_link'
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        return marker

    def _get_line_marker(self, id, points, a=1):
        marker = Marker(
            ns='walls',
            pose=Pose(position=Point(x=0, y=0, z=0)),
            action=0,
            type=4,
            id=id,
            points=points,
            color=ColorRGBA(g=1, a=a)
        )
        marker.header.frame_id = 'base_link'
        marker.scale.x = 0.03

        return marker
