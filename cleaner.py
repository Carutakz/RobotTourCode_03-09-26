from zumo_2040_robot import robot
import time

# --- Hardware ---
motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()

# --- Config ---
MAX_SPEED = 2000  # safe forward speed

# --- Main Loop ---
display.fill(0)
display.text("Infinite Run", 0, 0)
display.text("Straight ahead", 0, 10)
display.show()

try:
    motors.set_speeds(MAX_SPEED, MAX_SPEED)  # move forward forever
    while True:
        # Optional: show encoder distance
        counts = encoders.get_counts()
        left, right = counts[0], counts[1]
        avg_counts = (left + right) // 2

        display.fill(0)
        display.text("Infinite Run", 0, 0)
        display.text(f"L={left}", 0, 10)
        display.text(f"R={right}", 0, 20)
        display.text(f"Avg={avg_counts}", 0, 30)
        display.show()

        time.sleep(0.1)

except KeyboardInterrupt:
    motors.off()
