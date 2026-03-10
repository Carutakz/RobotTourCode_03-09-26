import time
from config import *

# State variables
robot_angle = 0.0
target_angle = 0.0
drive_motors = False
last_time_gyro_reading = None
last_time_far_from_target = None
turn_rate = 0.0  # degrees/sec

class GyroController:
    def __init__(self, line_sensors, motors, encoders, display, imu):
        self.line_sensors = line_sensors
        self.motors = motors
        self.encoders = encoders
        self.display = display
        self.imu = imu

    def update_gyro(self):
        global robot_angle, last_time_gyro_reading, turn_rate
        if self.imu.gyro.data_ready():
            self.imu.gyro.read()
            turn_rate = self.imu.gyro.last_reading_dps[2]  # yaw
            now = time.ticks_us()
            if last_time_gyro_reading:
                dt = time.ticks_diff(now, last_time_gyro_reading)
                robot_angle += turn_rate * dt / 1_000_000
            last_time_gyro_reading = now

    def normalize_angle(self, angle):
        while angle > 180: angle -= 360
        while angle < -180: angle += 360
        return angle

    def draw_angle_display(self):
        """
        Draw current robot angle and target angle on the display.
        """
        self.display.fill_rect(0, 0, 128, 16, 0)  # Clear previous text
        self.display.text(f"Angle: {robot_angle:>6.1f}", 0, 0, 1)
        self.display.text(f"Target: {target_angle:>6.1f}", 0, 10, 1)
        self.display.show()

    def turn_to_angle(self, angle):
        """
        Turn robot to a specific angle using gyro feedback.
        Optionally display angle on screen if `display` is provided.
        """
        global target_angle, drive_motors, last_time_far_from_target, last_time_gyro_reading
        target_angle = angle
        drive_motors = True
        last_time_far_from_target = time.ticks_ms()
        last_time_gyro_reading = time.ticks_us()

        while drive_motors:
            self.update_gyro()
            angle_error = self.normalize_angle(target_angle - robot_angle)

            if abs(angle_error) <= GYRO_ANGLE_TOLERANCE and \
               time.ticks_diff(time.ticks_ms(), last_time_far_from_target) > 250:
                drive_motors = False
                break

            turn_speed = angle_error * GYRO_KP - turn_rate * GYRO_KD
            turn_speed = max(-GYRO_MAX_SPEED, min(GYRO_MAX_SPEED, turn_speed))
            self.motors.set_speeds(-turn_speed, turn_speed)
            self.draw_angle_display()  # update display in real time

            time.sleep(0.01)

        self.motors.off()

    def turn_left_90(self):
        self.turn_to_angle(robot_angle - 90)

    def turn_right_90(self):
        self.turn_to_angle(robot_angle + 90)