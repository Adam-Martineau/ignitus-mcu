from math import trunc
import json
from machine import Pin, SoftI2C, RTC
import constants

class power_supplie_data:
    add = None
    power = None
    current = None
    voltage = None
    timestamp = None
    
    def __str__(self):
        return json.dumps(
            {
                'add': self.add,
                'power': self.power,
                'current': self.current,
                'voltage': self.voltage,
                'timestamp': self.timestamp
            }
        )

class power_supplie_sensor:
    data = power_supplie_data()
    rtc = RTC()
        
    def __init__(self, add:int):
        self.sda = constants.i2c_sda
        self.scl = constants.i2c_scl
        self.add = constants.current_sensor_add
        self.current_lsb = constants.max_current/(2**15)

        self.i2c = SoftI2C(
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)
    
    def get_data(self) -> power_supplie_data:
        self._write_calibration()
        
        self._read_voltage_bytes()
        self._read_current_bytes()
        self._read_power_bytes()
        
        self._convert_voltage()
        self._convert_current()
        self._convert_power()
        
        self.data.timestamp = self.rtc.datetime()
        self.data.add = self.add
        return self.data

    def _convert_current(self):
        self.data.current = self.data.current * self.current_lsb
        
    def _convert_power(self):
        self.data.power = self.data.power * (self.current_lsb * 20)

    def _convert_voltage(self):
        self.data.voltage = self.data.voltage >> 3
        self.data.voltage = self.data.voltage * 0.004

    def _write_calibration(self):
        reg = constants.current_sensor_reg_cal
        cal = self._get_cal()
        self.i2c.writeto_mem(self.add, reg, cal)

    def _read_current_bytes(self):
        reg = constants.current_sensor_reg_current
        self.data.current = int.from_bytes(self.i2c.readfrom_mem(self.add, reg, 2), "big")

    def _read_power_bytes(self):
        reg = constants.current_sensor_reg_power
        self.data.power = int.from_bytes(self.i2c.readfrom_mem(self.add, reg, 2), "big")

    def _read_voltage_bytes(self):
        reg = constants.current_sensor_reg_bus_voltage
        self.data.voltage = int.from_bytes(self.i2c.readfrom_mem(self.add, reg, 2), "big")

    def _get_cal(self):
        value = trunc(0.04096 / (self.current_lsb * constants.r_shunt))
        return bytearray(value.to_bytes(2, 'big'))
    
power = power_supplie_sensor(64)
print(power.get_data())