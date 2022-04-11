from servomotor import servo
import constants

def purge_open():
    pass


def purge_close():
    pass  


def emergency_stop():
    valve = servo(constants.servo_pin)
    valve.close()
    
    purge_open()


def ignition():
    pass


def launch():
    valve = servo(constants.servo_pin)
    valve.open()
