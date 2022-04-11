import os
import sdcard
import constants
from machine import SoftSPI, UART, Pin, RTC

class logger:
    def __init__(self, filename, uart):
        self.spi = SoftSPI(
            baudrate=1000000,
            polarity=0,
            phase=0,
            bits=8,
            firstbit=SoftSPI.MSB,
            sck=Pin(constants.uSD_sck),
            mosi=Pin(constants.uSD_mosi),
            miso=Pin(constants.uSD_miso)
        )

        self.cs = Pin(constants.uSD_cs, Pin.OUT)
        self.filename = filename
        
        self.uart = uart
        
    def write(self, msg: str):
        self._write_to_serial(msg)
        self._write_to_uSD(msg)
    
    def _write_to_serial(self, msg: str):
        self.uart.write(msg)
    
    def _write_to_uSD(self, msg: str):
        # Initialize SD card
        sd = sdcard.SDCard(self.spi, self.cs)

        # Mount filesystem
        os.mount(sd, '/sd')
        
        rtc = RTC()
        
        path = "/sd/" + self.filename + ".txt"
        msg = "<"+ str(rtc.datetime()) + "> " + msg + "\r\n"
        
        # Create a file and write something to it
        with open(path, "a") as file:
            file.write(msg)
