import numpy as np 
import csv
import time
from sensors.msg import RawMeasurement

class MeasurementReader():

    def __init__(self, filename=None, delimiter=',', file_size=10):
        self.data = None # List of RawMeasurement to be?
        self.filename = filename if (filename is not None) else (
            'data/weather_data_nyc_centralpark_2016.csv'
        )
        self.delimiter = delimiter
        self.file_size = file_size
        self.current_day = -1
        self._read_data()

    def _read_data(self):
        # Create reader
        data_file = open(self.filename)
        data_csv = csv.reader(data_file, delimiter=self.delimiter)

        # Read data
        data_iterator = iter(data_csv)
        next(data_iterator)
        self.data = []
        for row in data_iterator:
            one_day_data = RawMeasurement(
                row[0], float(row[3]), float(row[1]), float(row[2])
            )
            self.data.append(one_day_data)

    def has_next_measurement(self):
        if (self.current_day == (len(self.data) - 1)):
            return False
        return True

    def get_next_measurement(self):
        self.current_day += 1
        return self.data[self.current_day]
