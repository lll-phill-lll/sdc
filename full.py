import os
import time
import os.path
import sys

# result of Lidar.calibrate()
LIDAR_CALIBRATION_WITHOUT_MANIPULATOR = {
    'angle_fix': 3.0810609316737434,  # lidar orientation (radians)
    'fl_x': 0.2041686441507201,  # front left corner coords
    'fl_y': 0.10426277741236079,
    'rr_x': -0.08011094659163859,  # rear right corner coords
    'rr_y': -0.0988675829003773,
}
LIDAR_CALIBRATION = LIDAR_CALIBRATION_WITHOUT_MANIPULATOR
LIDAR_CALIBRATION_RANGE_LIMIT = 0.3
TURN_DIAMETER = 0.88
FROM_LIDAR_TO_CENTER = 0.07

# meters per count of primary or secondary peaks
ODOMETRY_METERS_PER_ROTATION = 3.9 / 47.

# ODOMETRY_PRIMARY_PIN = 18  # BOARD pin 12 T200 default
ODOMETRY_PRIMARY_PIN = 20  # BOARD pin 38 T208 default

ODOMETRY_SECONDARY_PIN = 19  # BOARD pin 35

PCA9685_I2C_BUSNUM = 1  # for T208 and pins 27, 28
# PCA9685_I2C_BUSNUM = 1  # for T200 and pins 3, 5
PCA9685_I2C_ADDR = 0x40  # 9865, over rides only if needed, ie. TX2..

STEERING_CHANNEL = 0
THROTTLE_CHANNEL = 1

# steering
SERVOS_LEFT_PULSE = 450
SERVOS_RIGHT_PULSE = 330

# throttle
SERVOS_MIN_PULSE = 330
SERVOS_MAX_PULSE = 390
SERVOS_ZERO_PULSE = 360

LASER_SENSOR_FRONT_ENABLED = True
LASER_SENSOR_FRONT_OFFSET = 190  # mm from lidar center
LASER_SENSOR_REAR_ENABLED = True
LASER_SENSOR_REAR_OFFSET = 60  # mm from lidar center

MANIPULATOR = {
    "enabled": False,
    "has_camera": False,
}

ENABLE_MULTIPROCESSING = True

if os.path.isfile(os.environ['HOME'] + '/ballsbot_config_override.py'):
    sys.path.append(os.environ['HOME'])
    # noinspection PyUnresolvedReferences
    from ballsbot_config_override import *
    if sys.path[-1] == os.environ['HOME']:
        sys.path.pop()

CAR_WIDTH = 2. * max(abs(LIDAR_CALIBRATION['fl_y']), abs(LIDAR_CALIBRATION['rr_y']))




def map_range(x, X_min, X_max, Y_min, Y_max):
    '''
    Linear mapping between two ranges of values
    '''
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range / Y_range

    y = ((x - X_min) / XY_ratio + Y_min) // 1

    return int(y)


def scaled_map_range(value, from_min, from_max, to_min, to_max, scale=1.):
    result = map_range(value, from_min / scale, from_max / scale, to_min, to_max)

    if to_min > to_max:
        to_min, to_max = to_max, to_min

    if result < to_min:
        result = to_min
    elif result > to_max:
        result = to_max

    return result


class PCA9685:
    '''
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    '''

    def __init__(self, channel, address=PCA9685_I2C_ADDR, frequency=60, busnum=PCA9685_I2C_BUSNUM, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (PCA9685_I2C_ADDR).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise

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

    def __init__(self,
                 controller=None,
                 left_pulse=SERVOS_LEFT_PULSE,
                 right_pulse=SERVOS_RIGHT_PULSE):
        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.scale = 2.
        self.pulse = scaled_map_range(
            0,
            self.LEFT_ANGLE, self.RIGHT_ANGLE,
            self.left_pulse, self.right_pulse,
            self.scale
        )
        self.running = True
        print('PWM Steering created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, angle):
        # map absolute angle to angle that vehicle can implement.
        self.pulse = scaled_map_range(
            angle,
            self.LEFT_ANGLE, self.RIGHT_ANGLE,
            self.left_pulse, self.right_pulse,
            self.scale
        )

    def run(self, angle):
        self.run_threaded(angle)
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

    def __init__(self,
                 controller=None,
                 max_pulse=SERVOS_MAX_PULSE,
                 min_pulse=SERVOS_MIN_PULSE,
                 zero_pulse=SERVOS_ZERO_PULSE):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.pulse = zero_pulse
        self.scale = 2.
        self.throttle = 0.
        self.zero_throttle = map_range(zero_pulse, self.min_pulse, self.max_pulse, self.MIN_THROTTLE, self.MAX_THROTTLE)

        # send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)
        self.running = True

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, throttle):
        # self.throttle = throttle
        # self.pulse = map_range(throttle,
        #                        self.MIN_THROTTLE, self.MAX_THROTTLE,
        #                        self.min_pulse, self.max_pulse)
        if abs(throttle - self.zero_throttle) < 0.25:
            self.throttle = 0.
            self.pulse = self.zero_pulse
        else:
            self.pulse = scaled_map_range(
                throttle,
                self.MIN_THROTTLE, self.MAX_THROTTLE,
                self.min_pulse, self.max_pulse,
                self.scale
            )

    def run(self, throttle):
        self.run_threaded(throttle)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # stop vehicle
        self.run(0)
        self.running = False


def get_controls():
    steering_controller = PCA9685(STEERING_CHANNEL)
    steering = PWMSteering(
        controller=steering_controller,
    )

    throttle_controller = PCA9685(THROTTLE_CHANNEL)
    throttle = PWMThrottle(
        controller=throttle_controller,
    )

    throttle.run(0)
    steering.run(0)

    return {
        'steering': steering,
        'throttle': throttle,
    }


controls = get_controls()
controls['steering'].run(0.2)
time.sleep(2)
controls['steering'].run(0.)
time.sleep(2)
