#! /usr/bin/env python

import actionlib
import rospy
from sensors.msg import BigTempDiffAction, BigTempDiffGoal, BigTempDiffResult, BigTempDiffFeedback
from sensors.msg import ProcessedMeasurement
from sensors.msg import RawMeasurement
from sensors.srv import RecordMeasurement, RecordMeasurementRequest, RecordMeasurementResponse
from utils.analysis import process_measurement

def maybe_big_diff_action(measurement, action_client):
    if measurement.big_difference:
        rospy.loginfo('Big temp diff action sent.')
        goal = BigTempDiffGoal(1)
        action_client.send_goal(goal)

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

def measurement_processing(measurement, args):
    proc_measurement = process_measurement(measurement)
    rospy.loginfo('New measurement processed.')

    args[0].publish(proc_measurement)
    record_measurement(proc_measurement)
    maybe_big_diff_action(proc_measurement, args[1])

def start_measurement_processing():
    rospy.init_node('processing', anonymous=False)
    processed_data_publisher = rospy.Publisher(
        'processed_data_topic', ProcessedMeasurement, queue_size = 10
    )
    action_client = actionlib.SimpleActionClient(
        'big_temp_diff', BigTempDiffAction
    )
    action_client.wait_for_server()
    rospy.Subscriber(
        'raw_data_topic', RawMeasurement, measurement_processing,
        (processed_data_publisher, action_client)
    )
    rospy.loginfo('Processing node is available.')
    rospy.spin()

if __name__ == '__main__':
    try:
        start_measurement_processing()
    except rospy.ROSInterruptException:
        pass


