import uos
import json
import constants
import sub_systems
from logging import logger
from machine import Pin, UART, RTC
from m32 import m32_sensor, m32_data
from gps import gps_sensor, gps_data
from power import power_supplie_sensor, power_supplie_data

# system data
class system_data:
    tank1:m32_data
    tank2:m32_data
    engine:m32_data
    power:power_supplie_data
    gps:gps_data
    armed = False
    ignition = False
    purge_valve = False
    main_valve = False
    
    def __str__():
        return json.dumps(
            {
                'tank1': self.tank1,
                'tank2': self.tank2,
                'engine': self.engine,
                'power': self.power,
                'gps': self.gps,
                'systems_status': {
                    'armed': self.armed,
                    'ignition': self.ignition,
                    'purge_valve': self.purge_valve,
                    'main_valve': self.main_valve
                },
                'timestamp': RTC.datetime()
            }
        )
    
data = system_data()

# sensors definitions
tank1_sensor = m32_sensor(constants.m32_add_tank_1)
tank2_sensor = m32_sensor(constants.m32_add_tank_2)
engine_sensor = m32_sensor(constants.m32_add_engine)
power_sensor = power_supplie_sensor(constants.gps_add)
gps_sensor_1 = gps_sensor(constants.gps_add)


def refresh_data():
    data.tank1 = tank1_sensor.get_data()
    data.tank2 = tank2_sensor.get_data()
    data.engine = engine_sensor.get_data()
    data.power = power_sensor.get_data()
    data.gps = gps_sensor_1.get_data()

class serial_reader:
    # Here we read the uart buffer and add any character to uart_rx if we find any command,
    # we remove it from uart_rx and execute the associated code. We do this because we can
    # read the uart rx buffer mid transmission, cutting a command in half.
    
    commands_list = ['emergency_stop', 'ignition', 'launch', 'purge_open', 'purge_close']
    buffer = 'uart buffer:'
    
    def __init__(self, uart):
        self.uart = uart

    def read_serial(self):
        if self.uart.any() > 0:
            self.buffer += self.uart.read().decode("utf-8")
            
            for cmd in self.commands_list:
                if cmd in self.buffer:
                    self.buffer = self.buffer.replace(cmd, '')
                    return cmd

def exec_cmd(cmd):
    if cmd == 'emergency_stop':
        log.write('Emergency stop engaging \n\r')
        sub_systems.emergency_stop()
    elif cmd == 'ignition':
        log.write('Ignition initiated \n\r')
        sub_systems.ignition()
    elif cmd == 'launch':
        log.write('Launch started \n\r')
        sub_systems.launch()
    elif cmd == 'purge_open':
        log.write('Purge opening \n\r')
        sub_systems.purge_open()
    elif cmd == 'purge_close':
        log.write('Purge closing \n\r')
        sub_systems.purge_close()

            
uart = UART(0, baudrate=9600, tx=Pin(constants.uart_tx), rx=Pin(constants.uart_rx), bits=8, parity=None, stop=1)
log = logger("test", uart)

def main():
    b = serial_reader(uart)
    while True:
        #refresh_data()
        cmd = b.read_serial()
        if cmd:
            print(cmd)
            exec_cmd(cmd)
        #log.write(data)
        
    
if __name__ == "__main__":
    main()
    