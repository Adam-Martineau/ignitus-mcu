from machine import Pin, PWM

class servo:
    def __init__(self, pin):
        self.pin = Pin(pin)
        self.pwm = PWM(self.pin)
        self.pwm.freq(333)

    def open(self):
        self.pwm.duty_u16(43690)
    
    def middle(self):
        self.pwm.duty_u16(32767)

    def close(self):
        self.pwm.duty_u16(21845)
