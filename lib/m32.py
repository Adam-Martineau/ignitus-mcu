import json
import constants
from machine import Pin, I2C

class m32_data:
    add = None
    temp = None
    pressure = None
    
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
        self.data.temp = ((25*self.data.temp)/256) - 50

    def _convert_press(self):
        self.data.pressure = (self.data.pressure - 1000)/40

    def _read_bytes(self):
        #self.i2c.writeto(self.add, self.add.to_bytes(2, 'big'))
        mybytes = self.i2c.readfrom(self.add, 4, False)
        self.data.pressure = (mybytes[0] << 8) + mybytes[1]
        self.data.temp = ((mybytes[2] << 8) + mybytes[3]) >> 5
        
        for b in mybytes:
            print(b)
        
        print(self.data.pressure)
        print(self.data.temp)

m = m32_sensor(constants.m32_add_tank_1)
print(m.get_data())