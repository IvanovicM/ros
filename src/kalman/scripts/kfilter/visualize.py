import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class RvizMarkerPublisher():

    def __init__(self, publisher):
        self.publisher = publisher
        self.kalman_idx = 0
        self.odom_idx = 0
        self.point_idx_period = 1000

    def show_estimated_position(self, position):
        marker = self._get_point_marker(
            self.kalman_idx, position, 'kalman_estimation', ColorRGBA(g=1, a=1)
        )
        self.publisher.publish(MarkerArray(markers=[marker]))
        self.kalman_idx = (self.kalman_idx + 1) % self.point_idx_period

    def show_odometry_position(self, position):
        marker = self._get_point_marker(
            self.odom_idx, position, 'odometry_estimation', ColorRGBA(r=1, a=1)
        )
        self.publisher.publish(MarkerArray(markers=[marker]))
        self.odom_idx = (self.odom_idx + 1) % self.point_idx_period

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
    
    def _get_point_marker(self, id, position, namespace, color):
        marker = Marker(
            ns=namespace,
            pose=Pose(position=Point(position.x, position.y, 0)),
            action=0,
            type=2,
            id=id,
            color=color
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
