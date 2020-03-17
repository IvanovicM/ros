import numpy as np 
import csv
from sensors.msg import RawMeasurement

class MeasurementReader():

    def __init__(self, filename=None, delimiter=',', file_size=10):
        self.data = None # List of RawMeasurement to be?
        self.filename = filename if (filename is not None) else (
            'data/weather_data_nyc_centralpark_2016.csv'
        )
        self.delimiter = delimiter
        self.file_size = file_size
            
        self._read_data()

    def _read_data(self):
        # Create reader
        data_file = open(self.filename)
        data_csv = csv.reader(data_file, delimiter=self.delimiter)

        # Read data
        # TODO(jana): read all measurements in self.data

    def has_next_measurement(self):
        # TODO(jana): return False when there are no more measurements
        return True

    def get_next_measurement(self):
        # TODO(jana): get next measurement from self.data
        return RawMeasurement()
