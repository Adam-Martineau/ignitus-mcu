from machine import Pin, SoftI2C

class gps_data:
    add:int
    data = 0
    timestamp = NULL

class gps_sensor:
    def __init__(self, add:int, sda: int, scl:int):
        self.sda = sda
        self.scl = scl
        self.add = add

        self.i2c = SoftI2C(
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)