import os
import sdcard
import constants
from machine import SPI, UART, Pin, RTC

class logger:
    def __init__(self, filename):
        self.spi = SPI(1,
            baudrate=1000000,
            polarity=0,
            phase=0,
            bits=8,
            firstbit=SPI.MSB,
            sck=Pin(constants.uSD_sck),
            mosi=Pin(constants.uSD_mosi),
            miso=Pin(constants.uSD_miso))

        self.cs = Pin(constants.uSD_cs, Pin.OUT)
        self.filename = filename
        
    def write(self, msg: str):
        self._write_to_serial(msg)
        self._write_to_uSD(msg)
    
    def _write_to_serial(self, msg: str):
        uart = UART(1, baudrate=9600, tx=Pin(0), rx=Pin(1))
        uart.write(msg)
    
    def _write_to_uSD(self, msg: str):
        # Initialize SD card
        sd = sdcard.SDCard(self.spi, self.cs)

        # Mount filesystem
        os.mount(fsobj=sd, mount_point='/sd', readonly=False)

        # Create a file and write something to it
        with open(f"/sd/{self.filename}.txt", "a") as file:
            file.write(f"<{RTC.datetime()}> {msg}\r\n")
