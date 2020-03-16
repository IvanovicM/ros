#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

def get_measurement():
    # TODO(jana): add one measurement reading
    return 1.0

def publish_measurement(publisher):
    measurement = get_measurement()
    publisher.publish(measurement)
    rospy.loginfo('Measurement published: {}'.format(measurement))

def start_measurements():
    measurements_publisher = rospy.Publisher(
        'measurements_topic', Float64, queue_size = 10
    )
    rospy.init_node('measurements_publisher', anonymous=True)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        publish_measurement(measurements_publisher)
        r.sleep()

if __name__ == '__main__':
    try:
        start_measurements()
    except rospy.ROSInterruptException:
        pass
