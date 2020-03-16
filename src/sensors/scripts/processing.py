#! /usr/bin/env python

import rospy
from sensors.msg import DayTemperature

def measurement_processing(measurement):
    # TODO(jana): one measurement processing
    rospy.loginfo('Average tempereature: {}'.format(measurement.avg_temp))

def start_measurement_processing():
    rospy.init_node('measurements_subscriber', anonymous=True)
    rospy.Subscriber(
        'measurements_topic', DayTemperature, measurement_processing
    )
    rospy.spin()

if __name__ == '__main__':
    try:
        start_measurement_processing()
    except rospy.ROSInterruptException:
        pass
