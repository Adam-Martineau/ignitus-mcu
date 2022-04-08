# run this file on the rp2040 to use the test function in the
# python terminal

import constants

from machine import RTC, Pin
from servomotor import servo
from m32 import m32_sensor, m32_data

def open_servo():
    moteur = servo(constants.servo_pin)
    moteur.open()

def close_servo():
    moteur = servo(constants.servo_pin)
    moteur.close()

def servo_percent(p):
    moteur = servo(constants.servo_pin)
    moteur.set_percent(p)

def get_gpio(gpio):
    return Pin(gpio).value()

def get_tank1():
    tank1_sensor = m32_sensor(constants.m32_add_tank_1)
    return tank1_sensor.get_data()

def get_tank2():
    tank2_sensor = m32_sensor(constants.m32_add_tank_2)
    return tank2_sensor.get_data()

def get_engine():
    engine_sensor = m32_sensor(constants.m32_add_engine)
    return engine_sensor.get_data()

def get_currant():
    power_sensor = power_supplie_sensor(constants.gps_add)
    power_sensor.get_data()

def get_gps():
    gps_sensor_1 = gps_sensor(constants.gps_add)
    return gps_sensor_1.get_data()
    
