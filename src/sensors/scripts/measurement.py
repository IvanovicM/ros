#! /usr/bin/env python

import rospy
from reader import MeasurementReader
from std_msgs.msg import Float64

def publish_measurement(publisher, mReader):
    measurement = mReader.get_next_measurement()
    publisher.publish(measurement)
    rospy.loginfo('Measurement published: {}'.format(measurement))

def start_measurements(mReader):
    measurements_publisher = rospy.Publisher(
        'measurements_topic', Float64, queue_size = 10
    )
    rospy.init_node('measurements_publisher', anonymous=True)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        publish_measurement(measurements_publisher, mReader)
        r.sleep()

if __name__ == '__main__':
    try:
        mReader = MeasurementReader()
        start_measurements(mReader)
    except rospy.ROSInterruptException:
        pass
