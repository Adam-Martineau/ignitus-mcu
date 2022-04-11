import json
import constants
from machine import Pin, SoftI2C, RTC

class m32_data:
    add:int
    temp:int
    pressure:int
    timestamp = None
    
    def __str__(self):
        return json.dumps(
            {
                'add': self.add,
                'temp': self.temp,
                'pressure': self.pressure,
                'timestamp': self.timestamp
            }
        )

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
        
        self.rtc = RTC()

    def get_data(self):
        self._read_bytes()
        self._convert_press()
        self._convert_temp()
        self.data.timestamp = self.rtc.datetime()
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
        self.i2c.writeto(self.add, self.add.to_bytes(2, 'big'))
        mybytes = self.i2c.readfrom(0x28, 4 , True)

        self.data.pressure = 459

        self.data.temp = 92
        
        print(mybytes)
        
        print(mybytes[0])
        print(mybytes[1])
        print(mybytes[2])
        print(mybytes[3])
        
        #print(self.data.pressure)
        #print(self.data.temp)
