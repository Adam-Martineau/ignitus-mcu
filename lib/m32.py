import constants
from machine import Pin, SoftI2C, RTC

class m32_data:
    add:int
    temp:int
    pressure:int
    timestamp = None

class m32_sensor:
    def __init__(self, add):
        self.sda = constants.i2c_sda
        self.scl = constants.i2c_scl
        self.add = add

        self.i2c = SoftI2C(
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)

        self.data = m32_data()

    def get_data(self):
        self._read_bytes()
        self._convert_press()
        self._convert_temp()
        self.data.timestamp = RTC.datetime()
        self.data.add = self.add
        return self.data

    def change_add(self, old, new):
        pass

    def _convert_temp(self):
        temp = ((25*self.data.temp)/256) - 50
        self.data.temp = temp

    def _convert_press(self):
        press = (self.data.pressure - 1000)/40
        self.data.pressure = press

    def _read_bytes(self):
        self.i2c.writeto(self.add, self.add)
        mybytes = self.i2c.readfrom(0x28, 4 , True)

        self.data.pressure = bytearray(mybytes[0]) 
        self.data.pressure.append(mybytes[1])

        self.data.temp = bytearray(mybytes[2])
        self.data.temp.append(mybytes[3])