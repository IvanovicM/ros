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

    def publish(self, position):
        marker = self._get_point_marker(self.next_id, position)
        self.publisher.publish(MarkerArray(markers=[marker]))
        self.next_id += 1
    
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
