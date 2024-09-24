# analyze the following code. It's controlling the smart car with 3 light sensor at the front of the car, at left, middle, right. The line is created by the black tap on the ground. 
# Analyze t, and try to archive the following goals - the track is a round rectangle.
# - Finish the laps 
# - Flinish the the laps fast
# - The faster one finishes win the game. 

import time
from Motor import *
import Line_Tracking
import RPi.GPIO as GPIO

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class ImprovedLineTracking(Line_Tracking):
    def __init__(self):
        super().__init__()
        self.pid = PIDController(kp=0.5, ki=0.1, kd=0.2)
        self.base_speed = 1000
        self.max_speed = 2000

    def get_line_position(self):
        if self.LMR == 0b010:  # Center
            return 0
        elif self.LMR == 0b100:  # Left
            return -1
        elif self.LMR == 0b001:  # Right
            return 1
        elif self.LMR == 0b110:  # Left-Center
            return -0.5
        elif self.LMR == 0b011:  # Right-Center
            return 0.5
        elif self.LMR == 0b111:  # All sensors
            return 0
        else:  # No line detected
            return None

    def run(self):
        last_time = time.time()
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            self.LMR = 0x00
            if GPIO.input(self.IR01):
                self.LMR |= 4
            if GPIO.input(self.IR02):
                self.LMR |= 2
            if GPIO.input(self.IR03):
                self.LMR |= 1

            line_position = self.get_line_position()
            
            if line_position is not None:
                pid_output = self.pid.compute(line_position, dt)
                left_speed = self.base_speed - pid_output
                right_speed = self.base_speed + pid_output

                # Ensure speeds are within limits
                left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
                right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

                PWM.setMotorModel(int(left_speed), int(left_speed), int(right_speed), int(right_speed))
            else:
                # If no line is detected, slow down and search
                PWM.setMotorModel(-200, -200, 200, 200)

            time.sleep(0.01)  # Small delay to prevent CPU overuse

# Replace the original Line_Tracking instance with the improved version
infrared = ImprovedLineTracking()

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        infrared.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        PWM.setMotorModel(0,0,0,0)
