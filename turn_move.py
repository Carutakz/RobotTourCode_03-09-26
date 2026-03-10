from zumo_2040_robot import motors, proximity_sensors, rgb_leds
import time

# Initialize modules
motor_instance = motors.Motors()
proximity = proximity_sensors.ProximitySensors()
led = rgb_leds.RGBLEDs()

# Constants
OBJECT_THRESHOLD = 5     # More sensitive
FORWARD_SPEED = 3500
TURN_SPEED = 3500
TURN_DURATION = 0.57        # Adjust for 90° turn

# Motor trim — adjust to make robot drive straighter
LEFT_TRIM = 0                # Positive = slow left motor
RIGHT_TRIM = -120            # Negative = speed up right motor

def detect_obstacle():
    """Return True if front sensors detect an object."""
    proximity.read()    
    left = proximity.front_counts_with_left_leds()
    right = proximity.front_counts_with_right_leds()
    print(f"Sensor L: {left} | R: {right}")  # Debugging output
    return (left is not None and left > OBJECT_THRESHOLD) or \
           (right is not None and right > OBJECT_THRESHOLD)

def turn_right_90():
    """Turn robot in place ~90 degrees to the right."""
    motor_instance.set_speeds(TURN_SPEED, -TURN_SPEED)
    led.set(0, [255, 165, 0])  # Orange = turning
    led.show()
    time.sleep(TURN_DURATION)
    motor_instance.off()
    time.sleep(0.2)

try:
    while True:
        # Calibrated forward movement
        left_speed = FORWARD_SPEED + LEFT_TRIM
        right_speed = FORWARD_SPEED + RIGHT_TRIM
        motor_instance.set_speeds(left_speed, right_speed)
        led.set(0, [0, 255, 0])  # Green = moving
        led.show()

        if detect_obstacle():
            print("Obstacle detected — stopping.")
            motor_instance.off()
            time.sleep(0.2)

            # Try up to 3 right turns to find clear path
            for i in range(3):
                turn_right_90()
                if not detect_obstacle():
                    print(f"Path cleared after {i+1} turn(s). Moving forward.")
                    break

            else:
                print("Still blocked after 3 turns. Waiting...")
                led.set(0, [255, 0, 0])  # Red = blocked
                led.show()
                time.sleep(1)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    motor_instance.off()
    led.off()

