from math import trunc
from machine import Pin, SoftI2C, RTC
import constants

class power_supplie_data:
    add:int
    power:int
    current:int
    voltage:int
    timestamp = NULL

class power_supplie_sensor:
    def __init__(self, add:int, sda: int, scl:int):
        self.sda = constants.i2c_sda
        self.scl = constants.i2c_scl
        self.add = constants.current_sensor_add

        self.i2c = SoftI2C(
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)

        self.data = power_supplie_data()
    
    def get_data(self) -> power_supplie_data:
        self._write_calibration()
        self._read_current_bytes()
        self._read_power_bytes()
        self._read_voltage_bytes()
        self._convert_current()
        self._convert_power()
        self._convert_voltage()
        self.data.timestamp = RTC.datetime()
        return self.data

    def _convert_current(self):
        pass

    def _convert_power(self):
        pass

    def _convert_voltage(self):
        pass

    def _write_calibration(self):
        reg = constants.current_sensor_reg_cal
        cal = self._get_cal()
        self.i2c.writeto_mem(self.add, reg, cal)

    def _read_current_bytes(self):
        reg = constants.current_sensor_reg_current
        self.data.current = self.i2c.readfrom_mem(self.add, reg, 2)

    def _read_power_bytes(self):
        reg = constants.current_sensor_reg_power
        self.data.power = self.i2c.readfrom_mem(self.add, reg, 2)

    def _read_voltage_bytes(self):
        reg = constants.current_sensor_reg_shunt_voltage
        self.data.voltage = self.i2c.readfrom_mem(self.add, reg, 2)

    def _get_cal(self):
        value = trunc(0.04096 / ((constants.max_current/(2**15)) * constants.r_shunt))
        return bytearray(value.to_bytes(2, 'big'))