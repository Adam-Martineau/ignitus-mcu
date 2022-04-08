from machine import Pin, PWM

# pwm range goes from 0 to 65535
# at 333hz we have a 3000us period
# the servo pwm up time goes from 1000us to 2000us a 1/3 ratio
# for the pwm function, from 21845 to 43690, for 100 deg of travel

pwm_min = 21845
pwm_max = 43690

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
        
    def set_percent(self, percent):
        range = pwm_max - pwm_min
        value = (percent * range) // 100
        value = value + pwm_min
        self.pwm.duty_u16(value)
