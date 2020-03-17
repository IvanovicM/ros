#! /usr/bin/env python

import rospy
from sensors.msg import ProcessedMeasurement
from sensors.msg import RawMeasurement
from sensors.srv import RecordMeasurement, RecordMeasurementRequest, RecordMeasurementResponse
from utils.analysis import process_measurement

def record_measurement(request):
    rospy.wait_for_service('record_data_service')
    try:
        record_data_service = rospy.ServiceProxy(
            'record_data_service', RecordMeasurement
        )

        response = record_data_service(request)
        if response:
            rospy.loginfo('Measurement recording succeeded.')
        else:
            rospy.logerr('Measurement recording failed.')
    except rospy.ServiceException, e:
        print('Service failed, {}'.format(e))

def measurement_processing(measurement, publisher):
    proc_measurement = process_measurement(measurement)
    publisher.publish(proc_measurement)
    record_measurement(proc_measurement)

    rospy.loginfo('New measurement processed.')

def start_measurement_processing():
    rospy.init_node('processing', anonymous=False)
    processed_data_publisher = rospy.Publisher(
        'processed_data_topic', ProcessedMeasurement, queue_size = 10
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


