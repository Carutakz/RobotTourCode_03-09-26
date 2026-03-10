# motor_diag.py
# Bare minimum test — no config, no odometry, no proximity, no track.
# Just spins the motors and reads encoders.
# If the robot moves: motors and encoders are fine, the bug is elsewhere.
# If it doesn't move: hardware/power issue.

from zumo_2040_robot import robot
import time

motors   = robot.Motors()
encoders = robot.Encoders()
display  = robot.Display()

display.fill(0)
display.text("Motor diag", 0, 0)
display.text("Running...", 0, 15)
display.show()

print("--- Motor Diagnostic ---")
print("Encoder counts before:", encoders.get_counts())

# Drive for 1 second at speed 2000
print("Setting motors to 2000...")
motors.set_speeds(2000, 2000)
time.sleep(1)
motors.off()

counts = encoders.get_counts()
print("Encoder counts after 1s at speed 2000:", counts)
print("Left ticks: {}  Right ticks: {}".format(counts[0], counts[1]))

if abs(counts[0]) < 10 and abs(counts[1]) < 10:
    print("PROBLEM: Encoders barely moved. Motors may not be running.")
    print("  Check: battery charged? Motor wiring connected?")
    display.fill(0)
    display.text("NO MOVEMENT", 0, 0)
    display.text("Check battery", 0, 15)
    display.text("& wiring", 0, 25)
    display.show()
else:
    print("OK: Motors and encoders working.")
    print("  Expected ~{}+ ticks for 1s at speed 2000".format(int(18 * 19)))
    display.fill(0)
    display.text("OK! Moved.", 0, 0)
    display.text("L:{}".format(counts[0]), 0, 15)
    display.text("R:{}".format(counts[1]), 0, 25)
    display.show()