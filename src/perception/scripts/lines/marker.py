import rospy
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

    def publish(self, lines):
        points = [Point(x=1, y=1, z=0), Point(x=1, y=-1, z=0)]
        marker1 = Marker(
            ns='walls',
            pose=Pose(position=Point(x=0, y=0, z=0)),
            action=0,
            type=4,
            id=0,
            points=points,
            color=ColorRGBA(g=1, a=1)
        )
        marker1.header.frame_id = 'base_link'
        marker1.scale.x = 0.05

        points = [Point(x=-1, y=1, z=0), Point(x=-1, y=-1, z=0)]
        marker2 = Marker(
            ns='walls',
            pose=Pose(position=Point(x=0, y=0, z=0)),
            action=0,
            type=4,
            id=1,
            points=points,
            color=ColorRGBA(b=1, a=1)
        )
        marker2.header.frame_id = 'base_link'
        marker2.scale.x = 0.05

        ma = MarkerArray(markers=[marker2, marker1])
        self.publisher.publish(ma)
