from machine import Pin, I2C
from time import sleep_us, ticks_us, ticks_diff


class stupid_i2c:
    _SDA_PIN = 0
    
    def __init__(self, sda: int, scl:int):
        #sda
        self._SDA_PIN = sda
        self.sda_out()
        
        #scl
        self.scl = Pin(scl, Pin.OUT)
        
        #i2c init
        self.i2c = I2C(0, sda=self.sda, scl=self.scl)
        
    #setting the sda pin as output
    def sda_out(self):
        self.sda = Pin(self._SDA_PIN, Pin.OUT)
    
    #setting the sda pin as input
    def sda_in(self):
        self.sda = Pin(self._SDA_PIN, Pin.IN)
    
    def read_byte(self, ack):
        self.sda_in()
        rx = 0
        
        for x in range(8):
            self.scl.off()
            sleep_us(2)
            self.scl.on()
            
            rx << 1
            if self.sda.value() == 1:
                rx += 1
            sleep_us(1)
        
        if ack:
            self.ack()
        else:
            self.not_ack()
            
        return rx
            
    def send_byte(self, byte):
        self.sda_out()
        self.scl.off()
        
        for x in range(8):
            if byte & 0x80:
                self.sda.on()
            else:
                self.sda.off()
            byte << 1
            
            sleep_us(2)
            self.scl.on()
            sleep_us(2)
            self.scl.off()
            sleep_us(2)
                
    def wait_4_ack(self):
        err_time = 0
        
        self.sda_in()
        self.sda.on()
        sleep_us(1)
        self.scl.on()
        sleep_us(1)
        
        start = ticks_us()
        while(self.sda.value()):
            err_time += 1
            if(ticks_diff(ticks_us, start) > 1000):
                self.stop()
                print("nack")
                return False
        
        self.scl.on()
        print("ack")
        return True
    
    #set of primitive function to send signals on the i2c lines        
    def start(self):
        self.sda_out()
        
        self.sda.on()
        self.scl.on()
        sleep_us(4)
        
        self.sda.off()
        sleep_us(4)
        self.scl.off()
        
    def stop(self):
        self.sda_out()
        
        self.scl.off()
        self.sda.off()
        sleep_us(4)
        
        self.scl.on()
        self.sda.on()
        sleep_us(4)
        
    def ack(self):
        self.sda_out()
        
        self.scl.off()
        self.sda.off()
        sleep_us(2)
        
        self.scl.on()
        sleep_us(2)
        self.scl.off()
        
    def not_ack(self):
        self.sda_out()
        
        self.scl.off()
        self.sda.on()
        sleep_us(2)
        
        self.scl.on()
        sleep_us(2)
        self.scl.off()