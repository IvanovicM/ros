#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('measurement_pub', String, queue_size = 10)
    rospy.init_node('measurement_pubLISHER', anonymous=True)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        strr = 'measurement %s' % rospy.get_time()
	rospy.loginfo(strr)
        pub.publish(strr)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        Cpass
