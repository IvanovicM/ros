#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

def measurement_processing(measurement):
    # TODO(jana): one measurement processing
    rospy.loginfo('Measurement heard: {}'.format(measurement.data))

def start_measurement_processing():
    rospy.init_node('measurements_subscriber', anonymous=True)
    rospy.Subscriber('measurements_topic', Float64, measurement_processing)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_measurement_processing()
    except rospy.ROSInterruptException:
        pass
