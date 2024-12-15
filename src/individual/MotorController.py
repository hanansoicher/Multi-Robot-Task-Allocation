# mpremote connect /dev/ttyUSB0 mip install machine to install on robots
import mip
mip.install("machine")
from machine import Pin, PWM, UART, mem16, mem32
import time

MAX_SPEED = 6000

class MotorController:
    def __init__(self):
        self.motors = Motors()
        
        self.FORWARD_SPEED = MAX_SPEED*0.7
        self.TURN_SPEED = MAX_SPEED*0.5
        self.SPIN_SPEED = MAX_SPEED*0.5
        
        # Initialize UART for HM-10 Bluetooth module
        self.uart = UART(1, baudrate=9600, tx=Pin(20), rx=Pin(21))
        
        # Status LED
        self.led = Pin(25, Pin.OUT)
        
    def stop(self):
        self.motors.off()
            
    def move_forward(self, duration_ms):
        self.motors.set_speeds(self.FORWARD_SPEED, self.FORWARD_SPEED)
        time.sleep_ms(duration_ms)
        self.stop()
        
    def turn_left(self, duration_ms):
        self.motors.set_speeds(-self.TURN_SPEED, self.TURN_SPEED)
        time.sleep_ms(duration_ms)
        self.stop()
        
    def turn_right(self, duration_ms):
        self.motors.set_speeds(self.TURN_SPEED, -self.TURN_SPEED)
        time.sleep_ms(duration_ms)
        self.stop()
        
    def spin(self, duration_ms=1000):
        self.motors.set_speeds(self.SPIN_SPEED, -self.SPIN_SPEED)
        time.sleep_ms(duration_ms)
        self.stop()

# Copied from Pololu micropython demo code
_PWM_BASE = 0x40050000
_CH7_DIV = _PWM_BASE + 0x90
_CH7_CC = _PWM_BASE + 0x98
_CH7_TOP = _PWM_BASE + 0x9c

class Motors:
    def __init__(self):
        self.right_motor_dir = Pin(10, Pin.OUT, value=0)
        self.left_motor_dir = Pin(11, Pin.OUT, value=0)
        self.right_motor_pwm_pin = Pin(14, Pin.OUT, value=0)
        self.left_motor_pwm_pin = Pin(15, Pin.OUT, value=0)
        self.right_motor_pwm = PWM(self.right_motor_pwm_pin, freq=20833, duty_u16=0)
        self.left_motor_pwm = PWM(self.left_motor_pwm_pin, freq=20833, duty_u16=0)

        # Make sure there are 6000 different speeds, even if the
        # RP2040 is running at a non-standard frequency.
        mem32[_CH7_DIV] = 16             # do not divide clock
        mem32[_CH7_TOP] = MAX_SPEED - 1  # 6000 different speeds, 20833 Hz

        # You can edit these lines if your motors are reversed.
        self._flip_left_motor = False
        self._flip_right_motor = False
        
    def flip_left(self, flip):
        self._flip_left_motor = flip
        
    def flip_right(self, flip):
        self._flip_right_motor = flip
        
    def _set_dir_left(self, speed):
        speed = round(speed)
        if speed < 0:
            if speed < -MAX_SPEED: speed = -MAX_SPEED
            self.left_motor_dir.value(not self._flip_left_motor)
            return -speed
        elif speed > 0:
            if speed > MAX_SPEED: speed = MAX_SPEED
            self.left_motor_dir.value(self._flip_left_motor)
            return speed
        return 0
        
    def _set_dir_right(self, speed):
        speed = round(speed)
        if speed < 0:
            if speed < -MAX_SPEED: speed = -MAX_SPEED
            self.right_motor_dir.value(not self._flip_right_motor)
            return -speed
        elif speed > 0:
            if speed > MAX_SPEED: speed = MAX_SPEED
            self.right_motor_dir.value(self._flip_right_motor)
            return speed
        return 0
    
    def set_speeds(self, left, right):
        left = self._set_dir_left(left)
        right = self._set_dir_right(right)
        cc = (left << 16) | right
        mem32[_CH7_CC] = cc

    def set_left_speed(self, speed):
        speed = self._set_dir_left(speed)
        mem32[_CH7_CC] = (speed << 16) | (mem32[_CH7_CC] & 0xffff)
        
    def set_right_speed(self, speed):
        speed = self._set_dir_right(speed)
        mem32[_CH7_CC] = (mem32[_CH7_CC] & 0xffff0000) | speed

    def off(self):
        self.set_speeds(0, 0)