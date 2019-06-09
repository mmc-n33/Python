"""
actuator.py
Classes to control motors and servo (via PWM controller)
"""

class PWMController:
    """PWM Controller for PCA9685 board"""

    def __init__(self, channel, frequency=60):
        import Adafruit_PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685()   # Initialize PCA9685 using default address (0x40)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pwm(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)
