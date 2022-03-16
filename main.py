import uos
import sdcard
import machine
import constants
from machine import SoftI2C, Pin, UART
from m32 import stupid_i2c
from time import sleep_ms
import logging
from time import sleep_us

#LOGGER = logging.logger(mosi=23, miso=20, cs=9, skl=22)
#uart = UART(1, 9600)

class sensor_value:
    tank1_press = 0
    tank1_temp = 0
    tank2_press = 0
    tank2_temp = 0
    engine_press = 0
    engine_temp = 0
    gps = 0
    current = 0

sensor = sensor_value()

def read_serial():
    input = uart.read()
    print(input)

def test_m32():
    print("start")

    i2c = SoftI2C(scl=Pin(5), sda=Pin(4), freq=100_000)

    # print(i2c.scan())

    m32 = stupid_i2c(sda=4, scl=5)

    m32.start()
    m32.send_byte(0x28)
    m32.wait_4_ack()
    m32.stop()
    # sleep_ms(2)

    # print("m32 as woke up")

    # m32.start()
    # m32.send_byte(0x28)
    # m32.wait_4_ack()
    # print(m32.read_byte(True))
    # print(m32.read_byte(True))
    # print(m32.read_byte(True))
    # print(m32.read_byte(False))
    # m32.stop()

    print("exit")

def test_uSD():
    # Assign chip select (CS) pin (and start it high)
    cs = machine.Pin(21, machine.Pin.OUT)

    # Intialize SPI peripheral (start with 1 MHz)
    spi = machine.SPI(0,
                    baudrate=1000000,
                    polarity=0,
                    phase=0,
                    bits=8,
                    firstbit=machine.SPI.MSB,
                    sck=machine.Pin(22),
                    mosi=machine.Pin(23),
                    miso=machine.Pin(20))

    # Initialize SD card
    sd = sdcard.SDCard(spi, cs)

    # Mount filesystem
    vfs = uos.VfsFat(sd)
    uos.mount(vfs, "/sd")

    # Create a file and write something to it
    with open("/sd/test_pog.txt", "w") as file:
        file.write("INDEX: ici haha.\r\n")
        
def main():
    test_m32()
    
if __name__ == "__main__":
    main()
