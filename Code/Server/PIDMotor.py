import math
import time
from PCA9685 import PCA9685
from ADC import *
from collections import deque

# Constants for improved control
MAX_SPEED = 4095
MIN_SPEED = -4095
ACCELERATION = 200  # Acceleration rate per cycle
DECELERATION = 400  # Deceleration rate per cycle
TURN_FACTOR = 0.8  # Factor to reduce inner wheel speed during turns
PID_SAMPLE_TIME = 0.01  # 10ms sample time for PID controller

# Improved PID controller for smoother line following
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt >= PID_SAMPLE_TIME:
            self.integral += error * dt
            derivative = (error - self.previous_error) / dt
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.previous_error = error
            self.last_time = current_time
            return output
        return 0

# Speed profile for smoother acceleration and deceleration
class SpeedProfile:
    def __init__(self, max_speed):
        self.max_speed = max_speed
        self.current_speed = 0
        self.acceleration = ACCELERATION
        self.deceleration = DECELERATION

    def update(self, target_speed):
        if target_speed > self.current_speed:
            self.current_speed = min(self.current_speed + self.acceleration, target_speed, self.max_speed)
        elif target_speed < self.current_speed:
            self.current_speed = max(self.current_speed - self.deceleration, target_speed, -self.max_speed)
        return self.current_speed

# Explanation of the code above:

# This code defines two important classes for improved motor control in a line-following robot:

# 1. PIDController:
#    - Implements a Proportional-Integral-Derivative (PID) controller
#    - Used for precise control of the robot's movement based on the error from the desired path
#    - Has tunable parameters: kp (proportional), ki (integral), and kd (derivative)
#    - Computes control output based on the current error and time since last computation
#    - Helps in maintaining smooth and accurate line following

# 2. SpeedProfile:
#    - Manages the acceleration and deceleration of the robot
#    - Ensures smooth changes in speed, preventing abrupt movements
#    - Uses defined acceleration and deceleration rates
#    - Updates the current speed towards a target speed within defined limits

# These classes work together to provide:
# - Smoother motion control
# - More precise line following
# - Better handling of turns and speed changes

# The code also defines several constants for motor control:
# - Speed limits (MAX_SPEED, MIN_SPEED)
# - Acceleration and deceleration rates
# - Turn factor for reducing inner wheel speed during turns
# - PID sample time for consistent control intervals

# This setup allows for more advanced and refined control of the robot's movement,
# potentially resulting in better performance in line following tasks.
import Motor

class PIDMotor(Motor):
    def __init__(self):
        super().__init__()
        self.pid_controller = PIDController(KP, KI, KD)
        self.speed_profile = SpeedProfile(MAX_SPEED)
        self.base_speed = BASE_SPEED

    def set_motor_speeds(self, left_speed, right_speed):
        # Apply speed profile for smooth acceleration/deceleration
        left_speed = self.speed_profile.update(left_speed)
        right_speed = self.speed_profile.update(right_speed)

        # Use the existing setMotorModel method from the parent Motor class
        self.setMotorModel(int(left_speed), int(left_speed), int(right_speed), int(right_speed))

    def follow_line(self, error):
        current_time = time.time()
        dt = current_time - self.pid_controller.last_time

        if dt >= PID_SAMPLE_TIME:
            # Compute PID output
            pid_output = self.pid_controller.compute(error, dt)

            # Calculate motor speeds based on PID output
            left_speed = self.base_speed - pid_output
            right_speed = self.base_speed + pid_output

            # Apply turn factor for smoother turns
            if pid_output > 0:  # Turning right
                left_speed *= (1 + TURN_FACTOR)
            else:  # Turning left
                right_speed *= (1 + TURN_FACTOR)

            # Ensure speeds are within limits
            left_speed = max(MIN_SPEED, min(MAX_SPEED, left_speed))
            right_speed = max(MIN_SPEED, min(MAX_SPEED, right_speed))

            # Set motor speeds
            self.set_motor_speeds(left_speed, right_speed)

    def stop(self):
        self.set_motor_speeds(0, 0)

# Create an instance of PIDMotor
pid_motor = PIDMotor()
