import constants
from machine import Pin, SoftI2C

class gps_data:
    add:int
    n_bytes = 0 
    data = 0
    timestamp = None

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
        self.data.n_bytes = self._read_nbr_availlable_bytes()
        self.data.data = self._read_bytes()
        self._convert_data()

    def _read_nbr_availlable_bytes(self) -> int:
        n_bytes_msb = self.i2c.readfrom_mem(self.add, 0xFD, 1)
        n_bytes_lsb = self.i2c.readfrom_mem(self.add, 0xFE, 1)

    def _read_bytes(self):
        self.data.data = self.i2c.readfrom_mem(self.add, 0xFF, self.data.n_bytes)

    def _convert_data(self):
        pass