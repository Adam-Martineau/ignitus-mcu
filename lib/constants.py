# constants.py 
# By Adam Martineau
# For Ignitus II project
# 3/6/22

# File containing all the constant for the project
# DON'T TOUCH THIS FILE, unless you know what your doing

# SPI pinout for the uSD card reader
uSD_miso = 20
uSD_mosi = 23
uSD_cs = 21
uSD_sck = 22

# I2C pinout
i2c_sda = 4
i2c_scl = 5

# i2c addresses for the m3200 sensors
m32_add_tank_1 = 0x28
m32_add_tank_2 = 0x28
m32_add_engine = 0x28