# constants.py 
# By Adam Martineau
# For Ignitus II project
# 3/6/22

# File containing all the constant for the project
# DON'T TOUCH THIS FILE, unless you really know what your doing

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

# i2c param for the gps
gps_add = 0x42

# i2c param for the current sensor
current_sensor_add = 0x40
current_sensor_reg_config = 0x00
current_sensor_reg_shunt_voltage = 0x01
current_sensor_reg_bus_voltage = 0x02
current_sensor_reg_power = 0x03
current_sensor_reg_current = 0x04
current_sensor_reg_cal = 0x05