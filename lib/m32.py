import json
import constants
from machine import Pin, I2C
from time import sleep_ms

class m32_data:
    add = None
    temp = None
    status = None
    pressure = None
    
    def __str__(self):
        return json.dumps(
            {
                'add': self.add,
                'temp': self.temp,
                'pressure': self.pressure,
                'status': self.status
            }
        )

class m32_sensor:
    data = m32_data()
    
    def __init__(self, add):
        self.sda = constants.i2c_sda
        self.scl = constants.i2c_scl
        self.add = add

        self.i2c = I2C(0,
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)

    def get_data(self):
        self._read_bytes()
        self._convert_press()
        self._convert_temp()
        self.data.add = self.add
        
        return self.data

    def change_add(self, new):
        pass

    def _convert_temp(self):
        self.data.temp = (5370619521453261/54975581388800000) * self.data.temp - (25009/500)

    def _convert_press(self):
        print(self.data.pressure)
        self.data.pressure = (785370160604119/109951162777600000) * self.data.pressure - (71429/10000)
        self.data.pressure = (self.data.pressure * 350) / 100
        
    def _read_bytes(self):
        #self.i2c.writeto(self.add, self.add.to_bytes(2, 'big'))
        mybytes = self.i2c.readfrom(self.add, 4, True)
        self.data.status = mybytes[0] >> 6
        self.data.pressure = ((mybytes[0] << 8) + mybytes[1]) & 0x3FFF
        self.data.temp = ((mybytes[2] << 8) + mybytes[3]) >> 5
