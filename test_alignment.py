# test_alignment.py
# Simple test script to verify perpendicular alignment to obstacles

from zumo_2040_robot import robot
from obstacle_aligner import ObstacleAligner
import time

def main():
    """
    Test the obstacle alignment feature.
    
    Instructions:
    1. Place robot ~10-15cm in front of an obstacle (like a box or wall)
    2. Robot may be at an angle
    3. Run this script
    4. Robot will align itself perpendicular to the obstacle
    """
    
    motors = robot.Motors()
    proximity = robot.ProximitySensors()
    display = robot.Display()
    
    # Create aligner
    aligner = ObstacleAligner(motors, proximity, display)
    
    # Tune these parameters if needed:
    aligner.ALIGNMENT_TOLERANCE = 50  # Lower = more precise (but may not converge)
    aligner.TURN_SPEED = 800         # Lower = slower, more precise
    aligner.MAX_ITERATIONS = 50       # Maximum attempts
    
    # Display instructions
    display.fill(0)
    display.text("Alignment Test", 0, 0)
    display.text("Place in front", 0, 15)
    display.text("of obstacle", 0, 25)
    display.text("Starting in 3s", 0, 40)
    display.show()
    
    print("=" * 40)
    print("OBSTACLE ALIGNMENT TEST")
    print("=" * 40)
    print("Place robot in front of obstacle")
    print("Robot can be at an angle")
    print("Starting in 3 seconds...")
    print("=" * 40)
    
    time.sleep(3)
    
    # Test 1: Basic alignment
    print("\n[TEST 1] Basic perpendicular alignment")
    success = aligner.align_perpendicular(
        approach_first=False,  # Don't approach, just align from current position
        approach_distance_sec=0
    )
    
    if success:
        print("[TEST 1] ✓ SUCCESS - Robot is aligned!")
        display.fill(0)
        display.text("SUCCESS!", 20, 20)
        display.text("Aligned!", 25, 35)
        display.show()
    else:
        print("[TEST 1] ✗ FAILED - Could not align")
        display.fill(0)
        display.text("FAILED", 25, 20)
        display.text("Check sensors", 5, 35)
        display.show()
    
    time.sleep(2)
    
    # Test 2: Show sensor readings
    print("\n[TEST 2] Displaying sensor readings for 5 seconds...")
    display.fill(0)
    display.text("Sensor Monitor", 5, 0)
    display.show()
    
    for i in range(50):  # 5 seconds at 0.1s intervals
        left, right = aligner._get_front_readings()
        diff = left - right if (left and right) else 0
        
        display.fill(0)
        display.text("Sensor Monitor", 5, 0)
        display.text(f"Left:  {left}", 0, 20)
        display.text(f"Right: {right}", 0, 35)
        display.text(f"Diff:  {diff}", 0, 50)
        display.show()
        
        print(f"Left={left}, Right={right}, Diff={diff}")
        time.sleep(0.1)
    
    # Done
    display.fill(0)
    display.text("Test Complete!", 5, 25)
    display.show()
    print("\n" + "=" * 40)
    print("Test complete!")
    print("=" * 40)


if __name__ == "__main__":
    main()