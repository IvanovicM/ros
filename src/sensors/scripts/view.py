#! /usr/bin/env python

import rospy
from sensors.msg import RawMeasurement

def measurement_view(measurement):
    rospy.loginfo('Average tempereature: {}'.format(measurement.avg_temp))

def start_measurement_view():
    rospy.init_node('view', anonymous=False)
    rospy.Subscriber(
        'processed_data_topic', RawMeasurement, measurement_view,
    )
    rospy.spin()

if __name__ == '__main__':
    try:
        start_measurement_view()
    except rospy.ROSInterruptException:
        pass

