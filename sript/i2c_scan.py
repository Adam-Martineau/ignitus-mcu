from machine import SoftI2C, Pin
from time import sleep_us

scl=Pin(5)
sda=Pin(4)

i2c = SoftI2C(scl=scl, sda=sda, freq=100_000)

print(i2c.scan())

print(i2c.readfrom(0x28, 2, True))
