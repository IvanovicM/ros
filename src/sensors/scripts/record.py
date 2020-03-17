#! /usr/bin/env python

import rospy
from sensors.msg import ProcessedMeasurement
from sensors.srv import RecordMeasurement, RecordMeasurementRequest, RecordMeasurementResponse
from utils.analysis import process_measurement

def process_service_request(request):
    # TODO(marina): record in a file
    rospy.loginfo('New measurement recorded.')

    response = RecordMeasurementResponse(True)
    return response

def start_recording_service():
    rospy.init_node('recording', anonymous=False)
    rospy.Service(
        'record_data_service', RecordMeasurement, process_service_request
    )
    rospy.loginfo('Service for recording data is available.')
    rospy.spin()

if __name__ == '__main__':
    try:
        start_recording_service()
    except rospy.ROSInterruptException:
        pass

