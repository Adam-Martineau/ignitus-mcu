from servomotor import servo

def purge_open():
    pass


def purge_close():
    pass  


def emergency_stop():
    valve = servo()
    valve.close()
    
    purge_open()


def ignition():
    pass


def launch():
    valve = servo()
    valve.open()
