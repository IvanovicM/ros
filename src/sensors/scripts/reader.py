import numpy as np 
import csv
from sensors.msg import DayTemperature

class MeasurementReader():

    def __init__(self, filename=None, delimiter=',', file_size=10):
        self.data = None # Some class to be?
        self.filename = filename if (filename is not None) else (
            'data/weather_data_nyc_centralpark_2016.csv'
        ) # Change while testing
        self.delimiter = '.'
        self.file_size = file_size
            
        self._read_data()

    def _read_data(self):
        # Create reader
        data_file = open(self.filename)
        data_csv = csv.reader(data_file, delimiter=self.delimiter)

        # Read data
        # TODO(jana): read all measurements, put it in self.data

    def get_next_measurement(self):
        # TODO(jana): get next measurement from self.data
        return DayTemperature()
