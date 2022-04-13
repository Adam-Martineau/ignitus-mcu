import json
import constants
from machine import Pin, SoftI2C, RTC

class gps_data:
    add:int = 0
    n_sat = 0 
    long = 0
    lat = 0
    timestamp = None
    
    def __str__(self):
        return json.dumps(
            {
                'add': self.add,
                'n_sat': self.n_sat,
                'long': self.long,
                'lat': self.lat,
                'timestamp': self.timestamp
            }
        )

class gps_sensor:
    def __init__(self, add:int):
        self.sda = constants.i2c_sda
        self.scl = constants.i2c_scl
        self.add = add

        self.i2c = SoftI2C(
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)

        self.data = gps_data()

    def get_data(self):
        n_bytes = self._read_nbr_availlable_bytes()
        raw_data = self._read_bytes(n_bytes)
        self._convert_data(raw_data)
        
        rtc = RTC()
        self.data.add = self.add
        self.data.timestamp = rtc.datetime()
        
        return self.data

    def _read_nbr_availlable_bytes(self) -> int:
        n_bytes_msb = int.from_bytes(self.i2c.readfrom_mem(self.add, 0xFD, 1), "big")
        n_bytes_lsb = int.from_bytes(self.i2c.readfrom_mem(self.add, 0xFE, 1), "big")
        return (n_bytes_msb * 16) + n_bytes_lsb

    def _read_bytes(self, n_bytes):
        print(n_bytes)
        if not n_bytes == 0:
            #return self.i2c.readfrom_mem(self.add, 0xFF, n_bytes)
            data = ""
            while not data == 0xff:
                data = self.i2c.readfrom(self.add, 1).decode("utf-8")
                print(data)

    def _convert_data(self, raw_data):
        pass
