import numpy as np 
import csv
import time
# from sensors.msg import RawMeasurement

######### OBRISI OVO ###########
class RawMeasurement():

    def __init__(self, day, max_temp, min_temp, avg_temp):
        self.day = day
        self.max_temp = max_temp
        self.min_temp = min_temp
        self.avg_temp = avg_temp

################################

class MeasurementReader():

    def __init__(self, filename=None, delimiter=',', file_size=10):
        self.data = None # List of RawMeasurement to be?
        self.filename = filename if (filename is not None) else (
            'data/weather_data_nyc_centralpark_2016.csv'
        )
        self.delimiter = ','
        self.file_size = file_size
        self.current_day = -1
        self._read_data()

    def _read_data(self):
        # Create reader
        data_file = open(self.filename)
        data_csv = csv.reader(data_file, delimiter=self.delimiter)

        # Read data
        # TODO(jana): read all measurements in self.data

        data_iterator = iter(data_csv)
        next(data_iterator)
        self.data = []
        for row in data_iterator:
            one_day_data = RawMeasurement(time.strptime(row[0],'%d-%m-%Y'), 
            float(row[1]), float(row[2]), float(row[3]))
            self.data.append(one_day_data)

    def has_next_measurement(self):
        # TODO(jana): return False when there are no more measurements

        if (self.current_day == (len(self.data) - 1)):
            return False
        return True

    def get_next_measurement(self):
        # TODO(jana): get next measurement from self.data

        self.current_day += 1
        return self.data[self.current_day]

if __name__ == '__main__':

    reading = MeasurementReader()
    for i in range(0,366):
        print(reading.has_next_measurement())
