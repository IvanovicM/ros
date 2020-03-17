#! /usr/bin/env python

import rospy
from sensors.msg import RawMeasurement
from utils.reader import MeasurementReader

def publish_measurement(publisher, mReader):
    measurement = mReader.get_next_measurement()
    publisher.publish(measurement)
    
    rospy.loginfo('New measurement read.')

def start_measurements(mReader):
    measurements_publisher = rospy.Publisher(
        'raw_data_topic', RawMeasurement, queue_size = 10
    )
    rospy.init_node('measurements', anonymous=False)
    r = rospy.Rate(100)

    while not rospy.is_shutdown() and mReader.has_next_measurement():
        publish_measurement(measurements_publisher, mReader)
        r.sleep()

if __name__ == '__main__':
    try:
        mReader = MeasurementReader()
        start_measurements(mReader)
    except rospy.ROSInterruptException:
        pass
