import numpy as np 
from sensors.msg import RawMeasurement
from sensors.msg import ProcessedMeasurement

def convert_to_C(far):
    return 32*(far - 1) * 5/9

def process_measurement(measurement):
    proc_measurement = ProcessedMeasurement()

    proc_measurement.day = measurement.day
    proc_measurement.avg_temp = convert_to_C(measurement.avg_temp)
    proc_measurement.min_temp = convert_to_C(measurement.min_temp)
    proc_measurement.max_temp = convert_to_C(measurement.max_temp)
    proc_measurement.big_difference = (
        proc_measurement.max_temp - proc_measurement.min_temp
    ) > 15

    return proc_measurement

