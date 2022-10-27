import os
import json
import time
from math import trunc
from machine import Pin, UART, RTC, I2C, SoftI2C, PWM, SoftSPI, Timer

###########################################
############### CONSTANTS #################
###########################################

_serial_start = "_JSONHEADER_"
_serial_stop = "_JSONFOOTER_"

# Pin
#purge_valve_pin = 3
#servo_pin = 16
#ignition_pin = 13
#continuite_pin = 9
#arming_pin = 17

purge_valve_pin = 18
servo_pin = 13
ignition_pin = 9
continuite_pin = 1
arming_pin = 17

# UART pins
uart_tx = 0
uart_rx = 1

# SPI pinout for the uSD card reader
uSD_miso = 20
uSD_mosi = 23
uSD_cs = 21
uSD_sck = 22

# I2C pinout
i2c_sda0 = 4
i2c_scl0 = 5
i2c_sda1 = 2
i2c_scl1 = 3

# i2c addresses for the m3200 sensors
m32_add = 40

# i2c param for the gps
gps_add = 66

# i2c param for the current sensor
current_sensor_add = 64
current_sensor_reg_config = 0x00
current_sensor_reg_shunt_voltage = 0x01
current_sensor_reg_bus_voltage = 0x02
current_sensor_reg_power = 0x03
current_sensor_reg_current = 0x04
current_sensor_reg_cal = 0x05
max_current = 2
r_shunt = 0.1

uart = UART(0, baudrate=9600, tx=Pin(uart_tx), rx=Pin(uart_rx), bits=8, parity=None, stop=1)


###########################################
################ SD CARD ##################
###########################################

_CMD_TIMEOUT = const(100)
_R1_IDLE_STATE = const(1 << 0)
# R1_ERASE_RESET = const(1 << 1)
_R1_ILLEGAL_COMMAND = const(1 << 2)
# R1_COM_CRC_ERROR = const(1 << 3)
# R1_ERASE_SEQUENCE_ERROR = const(1 << 4)
# R1_ADDRESS_ERROR = const(1 << 5)
# R1_PARAMETER_ERROR = const(1 << 6)
_TOKEN_CMD25 = const(0xFC)
_TOKEN_STOP_TRAN = const(0xFD)
_TOKEN_DATA = const(0xFE)


class SDCard:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs

        self.cmdbuf = bytearray(6)
        self.dummybuf = bytearray(512)
        self.tokenbuf = bytearray(1)
        for i in range(512):
            self.dummybuf[i] = 0xFF
        self.dummybuf_memoryview = memoryview(self.dummybuf)

        # initialise the card
        self.init_card()

    def init_spi(self, baudrate):
        try:
            master = self.spi.MASTER
        except AttributeError:
            # on ESP8266
            self.spi.init(baudrate=baudrate, phase=0, polarity=0)
        else:
            # on pyboard
            self.spi.init(master, baudrate=baudrate, phase=0, polarity=0)

    def init_card(self):
        # init CS pin
        self.cs.init(self.cs.OUT, value=1)

        # init SPI bus; use low data rate for initialisation
        self.init_spi(100000)

        # clock card at least 100 cycles with cs high
        for i in range(16):
            self.spi.write(b"\xff")

        # CMD0: init card; should return _R1_IDLE_STATE (allow 5 attempts)
        for _ in range(5):
            if self.cmd(0, 0, 0x95) == _R1_IDLE_STATE:
                break
        else:
            raise OSError("no SD card")

        # CMD8: determine card version
        r = self.cmd(8, 0x01AA, 0x87, 4)
        if r == _R1_IDLE_STATE:
            self.init_card_v2()
        elif r == (_R1_IDLE_STATE | _R1_ILLEGAL_COMMAND):
            self.init_card_v1()
        else:
            raise OSError("couldn't determine SD card version")

        # get the number of sectors
        # CMD9: response R2 (R1 byte + 16-byte block read)
        if self.cmd(9, 0, 0, 0, False) != 0:
            raise OSError("no response from SD card")
        csd = bytearray(16)
        self.readinto(csd)
        if csd[0] & 0xC0 == 0x40:  # CSD version 2.0
            self.sectors = ((csd[8] << 8 | csd[9]) + 1) * 1024
        elif csd[0] & 0xC0 == 0x00:  # CSD version 1.0 (old, <=2GB)
            c_size = csd[6] & 0b11 | csd[7] << 2 | (csd[8] & 0b11000000) << 4
            c_size_mult = ((csd[9] & 0b11) << 1) | csd[10] >> 7
            self.sectors = (c_size + 1) * (2 ** (c_size_mult + 2))
        else:
            raise OSError("SD card CSD format not supported")
        # print("sectors", self.sectors)

        # CMD16: set block length to 512 bytes
        if self.cmd(16, 512, 0) != 0:
            raise OSError("can't set 512 block size")

        # set to high data rate now that it"s initialised
        self.init_spi(1320000)

    def init_card_v1(self):
        for i in range(_CMD_TIMEOUT):
            self.cmd(55, 0, 0)
            if self.cmd(41, 0, 0) == 0:
                self.cdv = 512
                # print("[SDCard] v1 card")
                return
        raise OSError("timeout waiting for v1 card")

    def init_card_v2(self):
        for i in range(_CMD_TIMEOUT):
            time.sleep_ms(50)
            self.cmd(58, 0, 0, 4)
            self.cmd(55, 0, 0)
            if self.cmd(41, 0x40000000, 0) == 0:
                self.cmd(58, 0, 0, 4)
                self.cdv = 1
                # print("[SDCard] v2 card")
                return
        raise OSError("timeout waiting for v2 card")

    def cmd(self, cmd, arg, crc, final=0, release=True, skip1=False):
        self.cs(0)

        # create and send the command
        buf = self.cmdbuf
        buf[0] = 0x40 | cmd
        buf[1] = arg >> 24
        buf[2] = arg >> 16
        buf[3] = arg >> 8
        buf[4] = arg
        buf[5] = crc
        self.spi.write(buf)

        if skip1:
            self.spi.readinto(self.tokenbuf, 0xFF)

        # wait for the response (response[7] == 0)
        for i in range(_CMD_TIMEOUT):
            self.spi.readinto(self.tokenbuf, 0xFF)
            response = self.tokenbuf[0]
            if not (response & 0x80):
                # this could be a big-endian integer that we are getting here
                for j in range(final):
                    self.spi.write(b"\xff")
                if release:
                    self.cs(1)
                    self.spi.write(b"\xff")
                return response

        # timeout
        self.cs(1)
        self.spi.write(b"\xff")
        return -1

    def readinto(self, buf):
        self.cs(0)

        # read until start byte (0xff)
        for i in range(_CMD_TIMEOUT):
            self.spi.readinto(self.tokenbuf, 0xFF)
            if self.tokenbuf[0] == _TOKEN_DATA:
                break
            time.sleep_ms(1)
        else:
            self.cs(1)
            raise OSError("timeout waiting for response")

        # read data
        mv = self.dummybuf_memoryview
        if len(buf) != len(mv):
            mv = mv[: len(buf)]
        self.spi.write_readinto(mv, buf)

        # read checksum
        self.spi.write(b"\xff")
        self.spi.write(b"\xff")

        self.cs(1)
        self.spi.write(b"\xff")

    def write(self, token, buf):
        self.cs(0)

        # send: start of block, data, checksum
        self.spi.read(1, token)
        self.spi.write(buf)
        self.spi.write(b"\xff")
        self.spi.write(b"\xff")

        # check the response
        if (self.spi.read(1, 0xFF)[0] & 0x1F) != 0x05:
            self.cs(1)
            self.spi.write(b"\xff")
            return

        # wait for write to finish
        while self.spi.read(1, 0xFF)[0] == 0:
            pass

        self.cs(1)
        self.spi.write(b"\xff")

    def write_token(self, token):
        self.cs(0)
        self.spi.read(1, token)
        self.spi.write(b"\xff")
        # wait for write to finish
        while self.spi.read(1, 0xFF)[0] == 0x00:
            pass

        self.cs(1)
        self.spi.write(b"\xff")

    def readblocks(self, block_num, buf):
        nblocks = len(buf) // 512
        assert nblocks and not len(buf) % 512, "Buffer length is invalid"
        if nblocks == 1:
            # CMD17: set read address for single block
            if self.cmd(17, block_num * self.cdv, 0, release=False) != 0:
                # release the card
                self.cs(1)
                raise OSError(5)  # EIO
            # receive the data and release card
            self.readinto(buf)
        else:
            # CMD18: set read address for multiple blocks
            if self.cmd(18, block_num * self.cdv, 0, release=False) != 0:
                # release the card
                self.cs(1)
                raise OSError(5)  # EIO
            offset = 0
            mv = memoryview(buf)
            while nblocks:
                # receive the data and release card
                self.readinto(mv[offset : offset + 512])
                offset += 512
                nblocks -= 1
            if self.cmd(12, 0, 0xFF, skip1=True):
                raise OSError(5)  # EIO

    def writeblocks(self, block_num, buf):
        nblocks, err = divmod(len(buf), 512)
        assert nblocks and not err, "Buffer length is invalid"
        if nblocks == 1:
            # CMD24: set write address for single block
            if self.cmd(24, block_num * self.cdv, 0) != 0:
                raise OSError(5)  # EIO

            # send the data
            self.write(_TOKEN_DATA, buf)
        else:
            # CMD25: set write address for first block
            if self.cmd(25, block_num * self.cdv, 0) != 0:
                raise OSError(5)  # EIO
            # send the data
            offset = 0
            mv = memoryview(buf)
            while nblocks:
                self.write(_TOKEN_CMD25, mv[offset : offset + 512])
                offset += 512
                nblocks -= 1
            self.write_token(_TOKEN_STOP_TRAN)

    def ioctl(self, op, arg):
        if op == 4:  # get number of blocks
            return self.sectors

###########################################
################## GPS ####################
###########################################

class gps_data:
    add:int = 0
    n_sat = 0 
    long = 0
    lat = 0
    
    def __str__(self):
        return json.dumps(
            {
                "add": self.add,
                "n_sat": self.n_sat,
                "long": self.long,
                "lat": self.lat,
            }
        )

class gps_sensor:
    data = gps_data()
    
    def __init__(self, add:int):
        self.sda = i2c_sda
        self.scl = i2c_scl
        self.add = add

        self.i2c = SoftI2C(
            scl=Pin(self.scl), 
            sda=Pin(self.sda), 
            freq=100_000)

    def get_data(self):
        n_bytes = self._read_nbr_availlable_bytes()
        raw_data = self._read_bytes(n_bytes)
        self._convert_data(raw_data)
        self.data.add = self.add
        return self.data

    def _read_nbr_availlable_bytes(self) -> int:
        n_bytes_msb = int.from_bytes(self.i2c.readfrom_mem(self.add, 0xFD, 1), "big")
        n_bytes_lsb = int.from_bytes(self.i2c.readfrom_mem(self.add, 0xFE, 1), "big")
        return (n_bytes_msb << 8) + n_bytes_lsb

    def _read_bytes(self, n_bytes):
        print(n_bytes)
        if not n_bytes == 0:
            #return self.i2c.readfrom_mem(self.add, 0xFF, n_bytes)
            data = ""
            while not data == 0xff:
                data = self.i2c.readfrom(self.add, 1).decode("utf-8")
                print(data)

    def _convert_data(self, raw_data):
        pass


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

    def set_percent(self, percent):
        range = pwm_max - pwm_min
        value = (percent * range) // 100
        value = value + pwm_min
        self.pwm.duty_u16(value)

    def open(self):
        #self.pwm.duty_u16(43690)
        self.set_percent(98)
    
    def middle(self):
        #self.pwm.duty_u16(32767)
        self.set_percent(45)

    def close(self):
        #self.pwm.duty_u16(21845)
        self.set_percent(0)


###########################################
############# POWER SENSOR ################
###########################################

class power_supplie_data:
    add = 0
    power = 0
    current = 0
    voltage = 0

class power_supplie_sensor:
    data = power_supplie_data()
    current_lsb = max_current/(2**15)
    
    def get_data(self) -> power_supplie_data:
        self.data.add = current_sensor_add
        return self.data
    
        self._write_calibration()
        
        self._read_voltage_bytes()
        self._read_current_bytes()
        self._read_power_bytes()
        
        self._convert_voltage()
        self._convert_current()
        self._convert_power()
        
        return self.data

    def _convert_current(self):
        self.data.current = self.data.current * self.current_lsb
        
    def _convert_power(self):
        self.data.power = self.data.power * (self.current_lsb * 20)

    def _convert_voltage(self):
        self.data.voltage = self.data.voltage >> 3
        self.data.voltage = self.data.voltage * 0.004

    def _write_calibration(self):
        i2c = I2C(0, scl=Pin(i2c_scl), sda=Pin(i2c_sda), freq=100_000)
        reg = current_sensor_reg_cal
        cal = self._get_cal()
        i2c.writeto_mem(current_sensor_add, reg, cal)

    def _read_current_bytes(self):
        i2c = I2C(0, scl=Pin(i2c_scl), sda=Pin(i2c_sda), freq=100_000)
        reg = current_sensor_reg_current
        self.data.current = int.from_bytes(i2c.readfrom_mem(current_sensor_add, reg, 2), "big")

    def _read_power_bytes(self):
        i2c = I2C(0, scl=Pin(i2c_scl), sda=Pin(i2c_sda), freq=100_000)
        reg = current_sensor_reg_power
        self.data.power = int.from_bytes(i2c.readfrom_mem(current_sensor_add, reg, 2), "big")

    def _read_voltage_bytes(self):
        i2c = I2C(0, scl=Pin(i2c_scl), sda=Pin(i2c_sda), freq=100_000)
        reg = current_sensor_reg_bus_voltage
        self.data.voltage = int.from_bytes(i2c.readfrom_mem(current_sensor_add, reg, 2), "big")

    def _get_cal(self):
        value = trunc(0.04096 / (self.current_lsb * r_shunt))
        return bytearray(value.to_bytes(2, "big"))


###########################################
############## M32 SENSOR #################
###########################################

class m32_data:
    add = 0
    temp = 0
    status = 0
    pressure = 0

class m32_sensor:
    data = m32_data()

    def get_data(self, i2c_line):
        self.data.add = m32_add
        
        self._read_bytes(i2c_line)
        self._convert_press()
        self._convert_temp()
        return self.data

    def _convert_temp(self):
        self.data.temp = (5370619521453261/54975581388800000) * self.data.temp - (25009/500)

    def _convert_press(self):
        self.data.pressure = (785370160604119/109951162777600000) * self.data.pressure - (71429/10000)
        self.data.pressure = (self.data.pressure * 350) / 100
        self.data.pressure = self.data.pressure * 14.506
        
    def _read_bytes(self, i2c_line):
        i2c = None
        
        if(i2c_line == 0):
            i2c = I2C(0, scl=Pin(i2c_scl0), sda=Pin(i2c_sda0), freq=100_000)
        else:
            i2c = I2C(1, scl=Pin(i2c_scl1), sda=Pin(i2c_sda1), freq=100_000)
            
        #i2c.writeto(m32_add, m32_add.to_bytes(2, 'big'))
        mybytes = i2c.readfrom(m32_add, 4, True)

        self.data.status = mybytes[0] >> 6
        self.data.pressure = ((mybytes[0] << 8) + mybytes[1]) & 0x3FFF
        self.data.temp = ((mybytes[2] << 8) + mybytes[3]) >> 5
    

###########################################
############### system_data ###############
###########################################

class system_data:
    tank1 = m32_data()
    tank2 = m32_data()
    power = power_supplie_data()
    battery = False
    armed = False
    ignition = False
    purge_valve = False
    main_valve = False

    def __str__(self):
        return json.dumps(
            {
                'tank1_add': self.tank1.add,
                'tank1_temp': self.tank1.temp,
                'tank1_status': self.tank1.status,
                'tank1_pressure': self.tank1.pressure,
                'tank2_add': self.tank2.add,
                'tank2_temp': self.tank2.temp,
                'tank2_status': self.tank2.status,
                'tank2_pressure': self.tank2.pressure,
                'power_add': self.power.add,
                'power_power': self.power.power,
                'power_current': self.power.current,
                'power_voltage': self.power.voltage,
                'battery': self.battery,
                'armed': self.armed,
                'ignition': self.ignition,
                'purge_valve': self.purge_valve,
                'main_valve': self.main_valve,
            }
        )
        
data = system_data()


###########################################
################# LOGGING #################
###########################################

class logger:
    def __init__(self, filename, uart):
        self.filename = filename   
        self.uart = uart
    
    def write_to_serial(self):
        self.uart.write(_serial_start + str(data) + _serial_stop)
    
    def write_to_uSD(self):
        msg = str(data)
        
        self.spi = SoftSPI(
            baudrate=1000000,
            polarity=0,
            phase=0,
            bits=8,
            firstbit=SoftSPI.MSB,
            sck=Pin(uSD_sck),
            mosi=Pin(uSD_mosi),
            miso=Pin(uSD_miso)
        )

        self.cs = Pin(uSD_cs, Pin.OUT)
        
        # Initialize SD card
        sd = SDCard(self.spi, self.cs)

        # Mount filesystem
        if "sd" not in os.listdir():
            os.mount(sd, "/sd")
        
        rtc = RTC()
        
        path = "/sd/" + self.filename + ".txt"
        msg = "<"+ str(rtc.datetime()) + "> " + msg + "\r\n"
        
        # Create a file and write something to it
        with open(path, "a") as file:
            print(msg, file=file)
            
log = logger("test", uart)


###########################################
################ CONTROL ##################
###########################################

def purge_open():
    purge_valve = Pin(purge_valve_pin, Pin.OUT)
    purge_valve.on()
    data.purge_valve = True


def purge_close():
    purge_valve = Pin(purge_valve_pin, Pin.OUT)
    purge_valve.off()
    data.purge_valve = False


def main_valve_open():
    valve = servo(servo_pin)
    data.main_valve = True
    valve.middle()
    time.sleep(1)
    valve.open()
    time.sleep(6)
    valve.close()
    

def main_valve_close():
    valve = servo(servo_pin)
    valve.close()
    data.main_valve = False


def emergency_stop():
    valve = servo(servo_pin)
    valve.close()
    purge_open()


def ignition():
    ignition = Pin(ignition_pin, Pin.OUT)
    ignition.on()
    data.ignition = True
    


###########################################
################# MAIN ####################
###########################################

def refresh_data(t):
    m32 = m32_sensor()
    #power_sensor = power_supplie_sensor()

        data.tank1 = m32.get_data(0)
    #data.tank2 = m32.get_data(1)
    #data.power = power_sensor.get_data()

    # Update purge
    #purge_valve = Pin(purge_valve_pin, Pin.IN)
    #data.purge_valve = purge_valve.value()
    
    #log.write_to_uSD()


class serial:
    # Here we read the uart buffer and add any character to uart_rx if we find any command,
    # we remove it from uart_rx and execute the associated code. We do this because we can
    # read the uart rx buffer mid transmission, cutting a command in half.
    
    commands_list = ["emergency_stop", 
                     "ignition", 
                     "main_valve_open", 
                     "main_valve_close", 
                     "purge_open", 
                     "purge_close",
                     "data"]

    buffer = ""
    
    def __init__(self, uart):
        self.uart = uart

    def read(self):
        if self.uart.any() > 0:
            self.buffer += self.uart.read().decode("utf-8")
            
            for cmd in self.commands_list:
                if cmd in self.buffer:
                    self.buffer = self.buffer.replace(cmd, "")
                    return cmd
    
    def write_data():
        uart.write(data)
        self.buffer = ""

    def write(self, out):
        uart.write(out)
        self.buffer = ""
        
serialPort = serial(uart)

def exec_cmd(cmd):
    if cmd == "emergency_stop":
        emergency_stop()

    elif cmd == "ignition":
        ignition()

    elif cmd == "main_valve_open":
        main_valve_open()

    elif cmd == "main_valve_close":
        main_valve_close()

    elif cmd == "purge_open":
        purge_open()

    elif cmd == "purge_close":
        purge_close()
        
    elif cmd == "data":
        log.write_to_serial()


def commands(t):
    command = serialPort.read()

    if command:
        print(command)
        exec_cmd(command)

def init():
    Pin(purge_valve_pin, Pin.OUT).off()
    Pin(ignition_pin, Pin.OUT).off()
    main_valve_close()
    purge_close()

def main():
    #init()
    
    timerRefreshData = Timer()
    timerCommands = Timer()
    
    timerRefreshData.init(period=100, callback=refresh_data)
    timerCommands.init(period=50, callback=commands)
    
    while True:
        time.sleep(1)
        print("alive")


if __name__ == "__main__":
    main()
    