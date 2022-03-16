from asyncio.windows_events import NULL
from machine import Pin, SoftI2C
from time import sleep_us, ticks_us, ticks_diff

class m32_data:
    add:int
    temp:int
    pressure:int
    timestamp = NULL

class m32_sensor:
    def __init__(self, add:int, sda: int, scl:int):
        self.sda = sda
        self.scl = scl
        self.add = add

        self.i2c = SoftI2C(
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)

    def read_data(self):
        pass

    def _read_bytes(self):
        self.i2c.writeto(0x28, b'101000')
        mybytes = self.i2c.readfrom(0x28, 4 , True)
        for b in mybytes:
            print(bin(b))

    def _bytes_to_int(self, bytes):
        pass
