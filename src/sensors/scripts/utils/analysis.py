import numpy as np 
from sensors.msg import RawMeasurement
from sensors.msg import ProcessedMeasurement

def C_to_F(cel):
    return cel * 9/5 + 32

def F_to_C(far):
    return (far - 32) * 5/9

def process_measurement(measurement):
    proc_measurement = ProcessedMeasurement()

    proc_measurement.day = measurement.day
    proc_measurement.avg_temp = F_to_C(measurement.avg_temp)
    proc_measurement.min_temp = F_to_C(measurement.min_temp)
    proc_measurement.max_temp = F_to_C(measurement.max_temp)
    proc_measurement.big_difference = (
        proc_measurement.max_temp - proc_measurement.min_temp
    ) > 15

    return proc_measurement

def prepare_to_write(measurement):
    prep_measurement = ProcessedMeasurement()

    prep_measurement.day = measurement.day
    prep_measurement.avg_temp = C_to_F(measurement.avg_temp)
    prep_measurement.min_temp = C_to_F(measurement.min_temp)
    prep_measurement.max_temp = C_to_F(measurement.max_temp)

    return prep_measurement
