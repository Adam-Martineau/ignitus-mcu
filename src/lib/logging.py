import os
import sdcard
from machine import SPI, UART, Pin
from datetime import datetime

FILE_NAME = datetime.now()


# default 23, 20, 9, 22
class logger:
    def __init__(self, mosi, miso, cs, sck):
        self.mosi = mosi
        self.miso = miso
        self.cs = cs
        self.sck = sck
        
    def write(self, msg: str):
        _write_to_serial(msg)
        _write_to_uSD(msg)
    
    def _write_to_serial(self, msg: str):
        uart = machine.UART(1, baudrate=9600, tx=Pin(0), rx=Pin(1))
        uart.write(msg)
    
    def _write_to_uSD(self, msg: str):
        # Assign chip select (CS) pin (and start it high)
        cs = machine.Pin(9, machine.Pin.OUT)

        # Intialize SPI peripheral (start with 1 MHz)
        spi = machine.SPI(1,
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
        os.mount(fsobj=sd, mount_point='/sd', readonly=False)

        # Create a file and write something to it
        with open(f"/sd/{FILE_NAME}.txt", "a") as file:
            file.write(f"{INDEX}: {msg}\r\n")
