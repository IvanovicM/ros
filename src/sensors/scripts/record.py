#! /usr/bin/env python

import rospy
from sensors.msg import ProcessedMeasurement
from sensors.srv import RecordMeasurement, RecordMeasurementRequest, RecordMeasurementResponse
from utils.analysis import process_measurement, prepare_to_record
import csv

def process_service_request(request):
    to_record = prepare_to_record(request.measurement)

    new_records_file = open('data/new_records.csv','a+')
    file_writer = csv.writer(new_records_file, delimiter = ',', lineterminator='\n')
    file_writer.writerow([to_record.day, str(to_record.min_temp), 
        str(to_record.max_temp), str(to_record.avg_temp)])
    new_records_file.close()

    rospy.loginfo('New measurement recorded.')
    return RecordMeasurementResponse(True)

def start_recording_service():
    rospy.init_node('recording', anonymous=False)


    new_records_file = open('data/new_records.csv','a+')
    file_writer = csv.writer(new_records_file, delimiter = ',',
        lineterminator='\n')
    file_writer.writerow(['date', 'minimal temperature', 
        'maximum temperature', 'average temperature'])
    new_records_file.close()

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
    



