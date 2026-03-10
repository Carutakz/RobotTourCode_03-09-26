# motor_controls.py - ORIGINAL CODE
import time
from zumo_2040_robot import robot
import config
robot_angle = 0.0  # global heading tracker

class TurnController:
    def __init__(self, motors, encoders, display, turn_speed, turn_ticks_90):
        self.motors = motors
        self.encoders = encoders
        self.display = display
        self.turn_speed = turn_speed
        self.turn_ticks_90 = turn_ticks_90

    # ---------------- Turn Function ----------------
    def turn_angle(self, direction, degrees):
        """
        Turn the robot in place by a specified number of degrees.

        direction: "L" for left, "R" for right
        degrees: angle to turn (any positive number)
        """
        # CHANGE #1: Add 0.6 degrees for right turns, 1.0 degree for left turns
        if direction.upper() == "R":
            degrees = degrees + 0.6
        elif direction.upper() == "L":
            degrees = degrees + 1.0
            
        # Calculate target ticks for this turn
        ticks_for_90 = config.TURN_TICKS_90  # encoder counts for 90°
        target_ticks = ticks_for_90 * (degrees / 90)

        # Reset encoders
        self.encoders.get_counts(reset=True)

        # Set motor speeds for turning
        if direction.upper() == "L":
            self.motors.set_speeds(-config.TURN_SPEED, config.TURN_SPEED)
        elif direction.upper() == "R":
            self.motors.set_speeds(config.TURN_SPEED, -config.TURN_SPEED)
        else:
            print("Invalid turn direction")
            return

        # Monitor encoder counts with timeout
        start_time = time.time()
        timeout = 5  # seconds
        while True:
            left_ticks, right_ticks = self.encoders.get_counts()
            avg_ticks = (abs(left_ticks) + abs(right_ticks)) / 2

            if avg_ticks >= target_ticks:
                break

            if time.time() - start_time > timeout:
                print("Turn timeout reached!")
                break

            time.sleep(0.001)  # short delay for responsiveness

        # Stop motors and give a brief pause
        self.motors.off()
        time.sleep(0.1)

    def smart_turn(self, direction, degrees, line_sensor):
        self.turn_angle(direction, degrees)
        if not line_sensor.wait_until_line_detected(timeout_sec=2.0):
            print("Warning: Line not found after turn.")

    def gyro_turn(self, angle_to_turn, imu, kp=350, kd=7, max_speed=6000):
        # CHANGE #2: Add 0.6 degrees for right turns (negative), 1.0 degree for left turns (positive)
        if angle_to_turn < 0:
            angle_to_turn = angle_to_turn - 0.6
        elif angle_to_turn > 0:
            angle_to_turn = angle_to_turn + 1.0

        print(f"[gyro_turn] Turning {angle_to_turn} degrees...")

        target_angle = angle_to_turn
        last_time_gyro_reading = time.ticks_us()
        last_time_far_from_target = time.ticks_ms()

        while True:
            if imu.gyro.data_ready():
                imu.gyro.read()
                turn_rate = imu.gyro.last_reading_dps[2]  # degrees/sec

                now = time.ticks_us()
                dt = time.ticks_diff(now, last_time_gyro_reading) / 1_000_000
                last_time_gyro_reading = now

                robot_angle += turn_rate * dt  # integrate

                # PD control
                error = target_angle - robot_angle
                turn_speed = kp * error - kd * turn_rate

                # Clamp turn speed
                turn_speed = max(-max_speed, min(max_speed, turn_speed))

                # Enforce minimum turning speed if still far
                if abs(error) > 2 and abs(turn_speed) < 1000:
                    turn_speed = 1000 * (1 if error > 0 else -1)

                self.motors.set_speeds(int(-turn_speed), int(turn_speed))

                print(
                    f"[gyro] angle={robot_angle:.2f}, error={error:.2f}, rate={turn_rate:.2f}, speed={turn_speed:.2f}")

                # Stop condition: in range and stable for 300 ms
                if abs(error) > 2:
                    last_time_far_from_target = time.ticks_ms()
                elif time.ticks_diff(time.ticks_ms(), last_time_far_from_target) > 300:
                    break

            time.sleep(0.005)

        self.motors.off()
        print("[gyro_turn] Done")

    def gyro_turn_with_line_lock(self, angle_to_turn, imu):
        global robot_angle
        # CHANGE #3: Add 1.6 degrees for right turns, 1.0 degree for left turns
        RIGHT_EXTRA_DEG = 4
        LEFT_EXTRA_DEG = 1.0
        
        if angle_to_turn < 0:   # right turns are usually negative
            angle_to_turn -= RIGHT_EXTRA_DEG
        elif angle_to_turn > 0:  # left turns are positive
            angle_to_turn += LEFT_EXTRA_DEG
        
        print(f"[gyro_turn] Turning {angle_to_turn} degrees")
        target_angle = robot_angle + angle_to_turn
        last_time_gyro_reading = time.ticks_us()
        while True:
            if imu.gyro.data_ready():
                imu.gyro.read()
                turn_rate = imu.gyro.last_reading_dps[2]  # z-axis

                now = time.ticks_us()
                dt = time.ticks_diff(now, last_time_gyro_reading) / 1_000_000
                last_time_gyro_reading = now

                robot_angle += turn_rate * dt

                # PD control
                error = target_angle - robot_angle
                turn_speed = config.GYRO_TURN_KP * error - config.GYRO_TURN_KD * turn_rate

                # Force minimum speed if angle is far but turning too slow
                if abs(error) > 2 and abs(turn_speed) < 10:
                    turn_speed = 10 * (1 if error > 0 else -1)

                turn_speed = max(-config.TURN_SPEED, min(config.TURN_SPEED, turn_speed))
                self.motors.set_speeds(int(-turn_speed+config.MOTOR_LEFT_TRIM), int(turn_speed+config.MOTOR_RIGHT_TRIM))

                print(f"[gyro] angle={robot_angle:.2f}, error={error:.2f}, speed={turn_speed:.2f}")

                if abs(error) < 2:
                    break

            time.sleep(0.005)

        self.motors.off()
        print("[gyro_turn] Done")
        self.straighten_after_turn(duration=0.15)
        self.motors.off()
        time.sleep(0.1)

    def straighten_after_turn(self,duration=0.15):
        """Move forward briefly to stabilize direction after turning."""
        left_speed = config.MIN_SPEED + config.MOTOR_LEFT_TRIM
        right_speed = config.MIN_SPEED  + config.MOTOR_RIGHT_TRIM
        self.motors.set_speeds(left_speed, right_speed)
        time.sleep(duration)
        self.motors.off()
        time.sleep(0.1)  # Small pause to settle

    def follow_straight_with_heading_hold(self, imu, obstacle_detector, target_cm=50, direction="F",
                                          turn_direction=None, turn_angle=90):
        global robot_angle
        self.encoders.get_counts(reset=True)
        running = True
        last_time_gyro = time.ticks_us()
        start_angle = robot_angle

        while running:
            # ----------- Gyro update -----------
            if imu.gyro.data_ready():
                imu.gyro.read()
                turn_rate = imu.gyro.last_reading_dps[2]

                now = time.ticks_us()
                dt = time.ticks_diff(now, last_time_gyro) / 1_000_000
                last_time_gyro = now

                robot_angle += turn_rate * dt

            # ----------- Heading correction -----------
            heading_error = start_angle - robot_angle
            gyro_correction = heading_error * config.GYRO_TURN_KP 

            # ----------- Encoder-based correction -----------
            left_ticks, right_ticks = self.encoders.get_counts()
            encoder_error = (right_ticks - left_ticks) * config.ENCODER_KP
            avg_ticks = abs((left_ticks + right_ticks) / 2)
            distance_cm = avg_ticks * config.WHEEL_CM_PER_COUNT

            # ----------- Check for obstacle -----------
            if not config.IGNORE_OBSTACLES and obstacle_detector.is_obstacle_ahead():
                print("[Obstacle] Detected. Stopping and reacting.")
                self.motors.off()

                self.display.fill(0)
                self.display.text("Obstacle!", 0, 0)
                self.display.show()
                # Turn if direction is provided
                if turn_direction:
                    # Back off slightly
                    self.motors.set_speeds(-config.MIN_SPEED, -config.MIN_SPEED)
                    time.sleep(0.1)
                    self.motors.off()
                    time.sleep(0.1)

                    angle = -turn_angle if turn_direction.upper() == "R" else turn_angle
                    # CHANGE #4: Add 0.6 degrees for right, 1.0 degree for left obstacle avoidance turns
                    if turn_direction.upper() == "R":
                        angle = angle - 0.6
                    elif turn_direction.upper() == "L":
                        angle = angle + 1.0
                    self.gyro_turn_with_line_lock(angle, imu)
                    self.motors.off()
                    self.display.fill(0)
                    self.display.text("Target Reached!", 0, 10)
                    self.display.show()
                    running = False
                    return "obstacle_handled"
                else:
                    print("[Warning] No turn_direction provided!, last step for the track")
                    # ----------- Speed computation -----------
                    self.compute_speeds(direction, gyro_correction, encoder_error, distance_cm)
                    # ----------- Distance reached check -----------
                    if self.check_distance_reached(distance_cm, target_cm):
                        running = False
                        break
                    time.sleep(0.005)
            else:
                # ----------- Speed computation -----------
                self.compute_speeds(direction, gyro_correction, encoder_error, distance_cm)
                # ----------- Distance reached check -----------
                if self.check_distance_reached(distance_cm, target_cm):
                    running = False
                    break
                time.sleep(0.005)

        self.motors.off()

    def compute_speeds(self, direction, gyro_correction, encoder_error, distance_cm):
        multiplier = 1 if direction == "F" else -1

        if direction == "F":
            corrected_left = config.GYRO_MAX_SPEED - gyro_correction - encoder_error
            corrected_right = config.GYRO_MAX_SPEED + gyro_correction + encoder_error
        else:
            corrected_left = config.GYRO_MAX_SPEED + gyro_correction + encoder_error
            corrected_right = config.GYRO_MAX_SPEED - gyro_correction - encoder_error

        left_speed = corrected_left + config.MOTOR_LEFT_TRIM
        right_speed = corrected_right + config.MOTOR_RIGHT_TRIM

        left_speed = self.clip_speed(left_speed)
        right_speed = self.clip_speed(right_speed)

        self.motors.set_speeds(multiplier * left_speed, multiplier * right_speed)

        # Update OLED display
        self.display.fill(0)
        self.display.text(f"Dist: {distance_cm:.1f} cm", 0, 10)
        self.display.show()

    def check_distance_reached(self, distance_cm, target_cm):
        if distance_cm >= target_cm:
            self.motors.off()
            self.display.fill(0)
            self.display.text("Target Reached!", 0, 10)
            self.display.show()
            return True
        return False

    def clip_speed(self, speed):
        if config.ALLOW_BACKWARD:
            return max(-config.GYRO_MAX_SPEED, min(config.GYRO_MAX_SPEED, int(speed)))
        else:
            return max(0, min(config.GYRO_MAX_SPEED, int(speed)))