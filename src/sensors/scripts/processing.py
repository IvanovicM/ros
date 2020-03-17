#! /usr/bin/env python

import rospy
from sensors.msg import RawMeasurement

def measurement_processing(measurement, publisher):
    publisher.publish(measurement)

    # TODO(jana): one measurement processing
    rospy.loginfo('Average tempereature: {}'.format(measurement.avg_temp))

def start_measurement_processing():
    rospy.init_node('processing', anonymous=False)
    processed_data_publisher = rospy.Publisher(
        'processed_data_topic', RawMeasurement, queue_size = 10
    )
    rospy.Subscriber(
        'raw_data_topic', RawMeasurement, measurement_processing,
        (processed_data_publisher)
    )
    rospy.spin()

if __name__ == '__main__':
    try:
        start_measurement_processing()
    except rospy.ROSInterruptException:
        pass
