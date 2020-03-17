#! /usr/bin/env python

import rospy
from sensors.msg import ProcessedMeasurement
from sensors.srv import RecordMeasurement, RecordMeasurementRequest, RecordMeasurementResponse
from utils.analysis import process_measurement, prepare_to_record

def process_service_request(request):
    to_record = prepare_to_record(request.measurement)
    response = RecordMeasurementResponse(True)
    # TODO(jana): to_record na kraj nekog fajla, sve je vec u F; responce True/False u zavisnosti od toga da li je uspelo

    if (response):
        new_records_file = open("new_records.txt",'a+')
        new_records_file.write(to_record + "\n")
        new_records_file.close()
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

