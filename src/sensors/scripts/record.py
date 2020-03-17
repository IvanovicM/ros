#! /usr/bin/env python

import rospy
from analysis import prepare_to_record
from sensors.msg import ProcessedMeasurement
from sensors.srv import RecordMeasurement, RecordMeasurementRequest, RecordMeasurementResponse
from utils.analysis import process_measurement

def process_service_request(request):
    to_record = prepare_to_record(request)
    response = RecordMeasurementResponse(True)
    # TODO(jana): to_record na kraj nekog fajla, sve je vec u F; responce True/False u zavisnosti od toga da li je uspelo

    rospy.loginfo('New measurement recorded.')
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

