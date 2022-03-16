from math import trunc
from machine import Pin, SoftI2C

class current_data:
    add:int
    data = 0
    timestamp = NULL

class current_sensor:
    def __init__(self, add:int, sda: int, scl:int):
        self.sda = sda
        self.scl = scl
        self.add = add

        self.i2c = SoftI2C(
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)

    def _write_calibration(self):
        cal = self._get_cal()
        

    def _get_cal(self):
        return trunc(0.04096 / ((1/(2**15)) * 0.1))