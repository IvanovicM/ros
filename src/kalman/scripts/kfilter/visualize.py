import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class RvizMarkerPublisher():

    def __init__(self, publisher):
        self.publisher = publisher
        self.next_id = 0

    def show_estimated_position(self, pred_matched):
        if pred_matched is None:
            print('None')
            return
        markers = []
        for i in range(len(pred_matched)):
            alpha = pred_matched[i][0]
            rho = pred_matched[i][1]
            x = rho * math.cos(alpha)
            y = rho * math.sin(alpha)

            marker = self._get_point_marker(i, Point(x=x, y=y, z=0))
            markers.append(marker)

        self.publisher.publish(MarkerArray(markers=markers))

    def show_global_map(self, global_map):
        markers = []
        for i in range(len(global_map.walls)):
            wall = global_map.walls[i]
            marker = self._get_line_marker(
                i,
                [Point(x=wall.start[0], y=wall.start[1], z=0),
                 Point(x=wall.end[0], y=wall.end[1], z=0)]
            )
            markers.append(marker)
        self.publisher.publish(MarkerArray(markers=markers))
    
    def _get_point_marker(self, id, position, a=1):
        marker = Marker(
            ns='kalman_prediciton',
            pose=Pose(position=Point(position.x, position.y, 0)),
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
            ns='global_map',
            pose=Pose(position=Point(x=0, y=0, z=0)),
            action=0,
            type=4,
            id=id,
            points=points,
            color=ColorRGBA(b=1, a=a)
        )
        marker.header.frame_id = 'base_link'
        marker.scale.x = 0.03

        return marker
