# target_distance.py

import time
import config


# ---------------- Line Follower Class ----------------
class LineFollower:
    def __init__(self, line_sensors, motors, encoders, display):
        self.line_sensors = line_sensors
        self.motors = motors
        self.encoders = encoders
        self.display = display
        self.last_p = 0
        self.distance_cm = 0
        self.lost_line_counter = 0
        self.max_lost_time_ms = 1000  # stop if line lost for more than 1 sec
        self.running = False

    def follow(self, target_cm=50):
        self.encoders.get_counts(reset=True)
        self.distance_cm = 0
        self.running = True
        self.lost_line_counter = 0
        self.last_p = 0

        while self.running:
            # Start async read first
            self.line_sensors.start_read()
            time.sleep(0.001)
            line = self.line_sensors.read_calibrated()
            total = sum(line)

            # Check if line is detected
            if total < 1000:
                self.lost_line_counter += 1
                if self.lost_line_counter * 5 > self.max_lost_time_ms:
                    self.motors.off()
                    self.display.fill(0)
                    self.display.text("Line Lost!", 0, 10)
                    self.display.show()
                    break
                position = self.last_p  # reuse last known position
            else:
                self.lost_line_counter = 0
                position = sum(line[i] * config.SENSOR_POS[i] for i in range(5)) / total

            # PID calculations
            p = position
            d = p - self.last_p
            self.last_p = p
            correction = (p * config.LINE_KP + d * config.LINE_KD) * config.SCALE_FACTOR

            # Encoder feedback
            left_ticks, right_ticks = self.encoders.get_counts()
            encoder_error = (left_ticks - right_ticks) * config.ENCODER_KP

            # Calculate motor speeds
            left_speed = config.MAX_SPEED - correction - encoder_error
            right_speed = config.MAX_SPEED + correction + encoder_error

            # Clip to motor range
            left_speed = self.clip_speed(left_speed+config.MOTOR_LEFT_TRIM)
            right_speed = self.clip_speed(right_speed+config.MOTOR_RIGHT_TRIM)

            self.motors.set_speeds(left_speed, right_speed)

            # Compute distance traveled
            avg_ticks = (left_ticks + right_ticks) / 2
            self.distance_cm = avg_ticks * config.WHEEL_CM_PER_COUNT

            # Optional: show distance on display
            self.display.fill(0)
            self.display.text(f"Dist: {self.distance_cm:.1f} cm", 0, 10)
            self.display.show()

            if self.distance_cm >= target_cm:
                self.motors.off()
                self.display.fill(0)
                self.display.text("Target Reached!", 0, 10)
                self.display.show()
                self.running = False
                break

            time.sleep(0.005)

    def clip_speed(self, speed):
        if config.ALLOW_BACKWARD:
            return max(-config.MAX_SPEED, min(config.MAX_SPEED, int(speed)))
        else:
            return max(0, min(config.MAX_SPEED, int(speed)))

    # ---------------- Calibration ----------------
    def calibrate_line_sensors(self):
        self.display.fill(0)
        self.display.text("Line Follower", 0, 0)
        self.display.text("Place on line", 0, 10)
        self.display.text("Calibrating...", 0, 20)
        self.display.show()
        time.sleep(0.5)

        # rotate robot to calibrate sensors
        self.motors.set_speeds(config.LINE_CALIBRATION_SPEED, -config.LINE_CALIBRATION_SPEED)
        for _ in range(config.LINE_CALIBRATION_COUNT // 4):
            self.line_sensors.calibrate()
        self.motors.off()
        time.sleep(0.2)

        self.motors.set_speeds(-config.LINE_CALIBRATION_SPEED, config.LINE_CALIBRATION_SPEED)
        for _ in range(config.LINE_CALIBRATION_COUNT // 2):
            self.line_sensors.calibrate()
        self.motors.off()
        time.sleep(0.2)

        self.motors.set_speeds(config.LINE_CALIBRATION_SPEED, -config.LINE_CALIBRATION_SPEED)
        for _ in range(config.LINE_CALIBRATION_COUNT // 4):
            self.line_sensors.calibrate()
        self.motors.off()

        self.display.fill(0)
        self.display.text("Calibration Done!", 0, 10)
        self.display.text("Starting track", 0, 20)
        self.display.show()
        time.sleep(0.5)

    def wait_until_line_detected(self,timeout_sec=2.0):
        start = time.time()
        while time.time() - start < timeout_sec:
            self.line_sensors.start_read()
            time.sleep(0.002)
            line = self.line_sensors.read_calibrated()
            if sum(line) > 1000:
                return True
        return False