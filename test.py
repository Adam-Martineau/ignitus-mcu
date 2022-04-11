# run this file on the rp2040 to use the test function in the
# python terminal

import constants

from time import sleep_ms,
from machine import RTC, Pin, UART
from servomotor import servo
from m32 import m32_sensor, m32_data
from machine import Pin, SoftI2C

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
    
    
def i2c_scan():
    sda = constants.i2c_sda
    scl = constants.i2c_scl
    i2c = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=100_000)
    print(i2c.scan())


def serial_tx(msg):
    uart = UART(0,
        baudrate=9600,
        tx=Pin(constants.uart_tx),
        rx=Pin(constants.uart_rx),
        bits=8,
        parity=None,
        stop=1)
    
    uart.write(msg)
    

def serial_rx():
    uart = UART(0,
        baudrate=9600,
        tx=Pin(constants.uart_tx),
        rx=Pin(constants.uart_rx),
        rxbuf = 100,
        bits=8,
        parity=None,
        stop=1)
    
    input("Press enter want ready to uart.read()")
    print(uart.read())
    
    
def logging_test():
    uart = UART(0, baudrate=9600, tx=Pin(constants.uart_tx), rx=Pin(constants.uart_rx), bits=8, parity=None, stop=1)
    log = logger("test", uart)
    log.write("test")
    