import rospy

from sensor_msgs.msg import LaserScan

class LinesDetector():

    def detect_lines(self, laser_scan):
        rospy.loginfo("New scan")