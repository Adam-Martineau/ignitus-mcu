import uos
import sdcard
import constants
import logging
from time import sleep_us
from m32 import m32_sensor, m32_data
from gps import gps_sensor, gps_data
from power import power_supplie_sensor, power_supplie_data
from machine import RTC

class system_data:
    tank1_data:m32_data
    tank2_data:m32_data
    engine_data:m32_data
    power_data:power_supplie_data

data = system_data()

LOGGER = logging.logger(mosi=23, miso=20, cs=9, skl=22)
        
def main():
    pass
    
if __name__ == "__main__":
    main()
