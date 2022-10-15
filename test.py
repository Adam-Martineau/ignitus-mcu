# run this file on the rp2040 to use the test function in the
# python terminal

from time import sleep_ms,
from machine import RTC, Pin, UART, Pin, SoftI2C


###########################################
############### CONSTANTS #################
###########################################

# UART pins
uart_tx = 0
uart_rx = 1

# Servo motor
servo_pin = 13

# I2C pinout
i2c_sda = 4
i2c_scl = 5

uart = UART(0, baudrate=9600, tx=Pin(uart_tx), rx=Pin(uart_rx), bits=8, parity=None, stop=1)

###########################################
############## SERVO MOTOR ################
###########################################

pwm_min = 21845
pwm_max = 43690

class servo:
    def __init__(self, pin):
        self.pin = Pin(pin)
        self.pwm = PWM(self.pin)
        self.pwm.freq(333)

    def open(self):
        self.pwm.duty_u16(43690)
    
    def middle(self):
        self.pwm.duty_u16(32767)

    def close(self):
        self.pwm.duty_u16(21845)
        
    def set_percent(self, percent):
        range = pwm_max - pwm_min
        value = (percent * range) // 100
        value = value + pwm_min
        self.pwm.duty_u16(value)


###########################################
################# TESTS ###################
###########################################
        
def open_servo():
    moteur = servo(servo_pin)
    moteur.open()

def close_servo():
    moteur = servo(servo_pin)
    moteur.close()

def servo_percent(p):
    moteur = servo(servo_pin)
    moteur.set_percent(p)

def get_gpio(gpio):
    return Pin(gpio).value()
    
def i2c_scan():
    i2c = SoftI2C(scl=Pin(i2c_scl), sda=Pin(i2c_sda), freq=100_000)
    print(i2c.scan())
    i2c = SoftI2C(scl=Pin(3), sda=Pin(2), freq=100_000)
    print(i2c.scan())


def serial_tx(msg):   
    uart.write(msg)
    

def serial_rx():
    print(uart.read())
    
    
def logging_test():
    uart = UART(0, baudrate=9600, tx=Pin(uart_tx), rx=Pin(uart_rx), bits=8, parity=None, stop=1)
    log = logger("thisisatest", uart)
    log.write("test\n\r")
    