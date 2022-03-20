import uos
import sdcard
import constants
import logging
from time import sleep_us, sleep_ms, time
from m32 import m32_sensor, m32_data
from gps import gps_sensor, gps_data
from power import power_supplie_sensor, power_supplie_data
from servomotor import servo
from machine import RTC, Pin

class system_data:
    tank1:m32_data
    tank2:m32_data
    engine:m32_data
    power:power_supplie_data

data = system_data()

tank1_sensor = m32_sensor(constants.m32_add_tank_1)
tank2_sensor = m32_sensor(constants.m32_add_tank_2)
engine_sensor = m32_sensor(constants.m32_add_engine)
power_sensor = power_supplie_sensor(constants.gps_add)

def refresh_data():
    data.tank1 = tank1_sensor.get_data()
    data.tank2 = tank2_sensor.get_data()
    data.engine = engine_sensor.get_data()
    data.power = power_sensor.get_data()

#LOGGER = logging.logger("test")

def main():
    moteur = servo(constants.servo_pin)
    moteur.open()
    sleep_ms(2000)
    moteur.close()
    sleep_ms(2000)
    moteur.middle()
    sleep_ms(2000)
    
if __name__ == "__main__":
    main()
