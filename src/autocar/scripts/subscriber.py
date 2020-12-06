
#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Float64

def map_range(x, X_min, X_max, Y_min, Y_max):
    '''
    Linear mapping between two ranges of values
    '''
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range/Y_range

    y = ((x-X_min) / XY_ratio + Y_min) // 1

    return int(y)

class PCA9685:
    ''' 
    PWM motor controler using PCA9685 boards. 
    This is used for most RC Cars
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            # replace the get_bus function with our own
            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay) # "Tamiya TBLE-02" makes a little leap otherwise

    def set_pulse(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pulse(pulse)

class PWMSteering:
    """
    Wrapper over a PWM motor controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self,controller=None,left_pulse=290,right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.pulse = map_range(0, self.LEFT_ANGLE, self.RIGHT_ANGLE, self.left_pulse, self.right_pulse)
        self.running = True
        print('PWM Steering created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run(self, angle):
        self.pulse = map_range(angle, self.LEFT_ANGLE, self.RIGHT_ANGLE, self.left_pulse, self.right_pulse)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # set steering straight
        self.pulse = 0
        time.sleep(0.3)
        self.running = False


class PWMThrottle:
    """
    Wrapper over a PWM motor controller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self, controller=None, max_pulse=300,min_pulse=490, zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.pulse = zero_pulse

        # send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_pulse(self.max_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.min_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)
        self.running = True
        print('PWM Throttle created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run(self, throttle):
        if throttle > 0:
            self.pulse = map_range(throttle, 0, self.MAX_THROTTLE, self.zero_pulse, self.max_pulse)
        else:
            self.pulse = map_range(throttle, self.MIN_THROTTLE, 0, self.min_pulse, self.zero_pulse)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # stop vehicle
        self.run(0)
        self.running = False

class Config():
    def __init__(self, STEERING_CHANNEL = 1,
                       PCA9685_I2C_ADDR = 0x40, 
                       PCA9685_I2C_BUSNUM = 1, 
                       STEERING_LEFT_PWM = 490, 
                       STEERING_RIGHT_PWM = 270, 
                       THROTTLE_CHANNEL = 2, 
                       THROTTLE_FORWARD_PWM = 490, 
                       THROTTLE_STOPPED_PWM = 360, 
                       THROTTLE_REVERSE_PWM = 280):
        self.STEERING_CHANNEL  = STEERING_CHANNEL
        self.PCA9685_I2C_ADDR = PCA9685_I2C_ADDR
        self.PCA9685_I2C_BUSNUM = PCA9685_I2C_BUSNUM
        self.STEERING_LEFT_PWM = STEERING_LEFT_PWM
        self.STEERING_RIGHT_PWM = STEERING_RIGHT_PWM
        self.THROTTLE_CHANNEL = THROTTLE_CHANNEL
        self.THROTTLE_FORWARD_PWM = THROTTLE_FORWARD_PWM 
        self.THROTTLE_STOPPED_PWM = THROTTLE_STOPPED_PWM
        self.THROTTLE_REVERSE_PWM = THROTTLE_REVERSE_PWM

cfg = Config()

steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
steering = PWMSteering(controller=steering_controller, left_pulse=cfg.STEERING_LEFT_PWM, right_pulse=cfg.STEERING_RIGHT_PWM)

throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
throttle = PWMThrottle(controller=throttle_controller, max_pulse=cfg.THROTTLE_FORWARD_PWM, zero_pulse=cfg.THROTTLE_STOPPED_PWM, min_pulse=cfg.THROTTLE_REVERSE_PWM)


def throttle_callback(data):
    throttle.run(data)

def steering_callback(data):
    steering.run(data)


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('steering', Float64, throttle_callback)
    rospy.Subscriber('throttle', Float64, steering_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

#     V.add(steering, inputs=['angle'], threaded=True)
#     V.add(throttle, inputs=['throttle'], threaded=True)
