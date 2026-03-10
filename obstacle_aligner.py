# obstacle_aligner.py
# Aligns the robot perpendicular to an obstacle using proximity sensors

import time
from zumo_2040_robot import robot
import config

class ObstacleAligner:
    def __init__(self, motors, proximity_sensors, display=None):
        self.motors = motors
        self.proximity = proximity_sensors
        self.display = display
        
        # Tunables
        self.ALIGNMENT_TOLERANCE = 50  # difference threshold between left/right sensors
        self.TURN_SPEED = 800  # slow turning speed for precision
        self.MAX_ITERATIONS = 50  # prevent infinite loops
        self.SETTLE_TIME = 0.05  # time to wait between adjustments
        
    def _safe_set_speeds(self, left, right):
        """Safely set motor speeds."""
        try:
            self.motors.set_speeds(int(left), int(right))
        except Exception as e:
            print(f"Motor speed error: {e}")
    
    def _safe_off(self):
        """Safely turn off motors."""
        try:
            self.motors.off()
        except:
            self._safe_set_speeds(0, 0)
    
    def _get_front_readings(self):
        """Get left and right proximity sensor readings from the front pair."""
        try:
            self.proximity.read()
            # Assuming front sensors are at index 1 (adjust if needed)
            front_pair = self.proximity.counts[config.PROX_FRONT_INDEX if hasattr(config, 'PROX_FRONT_INDEX') else 1]
            left_val = int(front_pair[0])
            right_val = int(front_pair[1])
            return left_val, right_val
        except Exception as e:
            print(f"Proximity read error: {e}")
            return None, None
    
    def align_perpendicular(self, approach_first=True, approach_distance_sec=0.3):
        """
        Align the robot perpendicular to the obstacle in front.
        
        Args:
            approach_first: If True, move forward slightly to get closer to obstacle
            approach_distance_sec: How long to move forward (in seconds)
        
        Returns:
            True if alignment succeeded, False otherwise
        """
        print("[Aligner] Starting perpendicular alignment...")
        
        # Optional: approach the obstacle first to get better readings
        if approach_first:
            print("[Aligner] Approaching obstacle...")
            self._safe_set_speeds(1200, 1200)
            time.sleep(approach_distance_sec)
            self._safe_off()
            time.sleep(0.2)
        
        iterations = 0
        aligned = False
        
        while iterations < self.MAX_ITERATIONS:
            # Get current sensor readings
            left_val, right_val = self._get_front_readings()
            
            if left_val is None or right_val is None:
                print("[Aligner] Failed to read sensors")
                return False
            
            # Calculate difference
            diff = left_val - right_val
            
            # Display status if available
            if self.display:
                self.display.fill(0)
                self.display.text("Aligning...", 0, 0)
                self.display.text(f"L:{left_val} R:{right_val}", 0, 15)
                self.display.text(f"Diff:{diff}", 0, 30)
                self.display.show()
            
            print(f"[Aligner] L={left_val}, R={right_val}, diff={diff}")
            
            # Check if aligned
            if abs(diff) <= self.ALIGNMENT_TOLERANCE:
                print("[Aligner] Aligned!")
                aligned = True
                break
            
            # Turn to equalize readings
            # If left > right, robot is angled left, so turn right
            # If right > left, robot is angled right, so turn left
            if diff > 0:
                # Turn right (clockwise) to reduce left sensor value
                print("[Aligner] Turning right...")
                self._safe_set_speeds(self.TURN_SPEED, -self.TURN_SPEED)
            else:
                # Turn left (counter-clockwise) to reduce right sensor value
                print("[Aligner] Turning left...")
                self._safe_set_speeds(-self.TURN_SPEED, self.TURN_SPEED)
            
            # Brief turn pulse
            turn_duration = min(0.1, abs(diff) / 5000.0)  # scale turn time with difference
            time.sleep(turn_duration)
            self._safe_off()
            time.sleep(self.SETTLE_TIME)
            
            iterations += 1
        
        self._safe_off()
        
        if aligned:
            if self.display:
                self.display.fill(0)
                self.display.text("Aligned!", 20, 20)
                self.display.show()
            print("[Aligner] Successfully aligned perpendicular")
            return True
        else:
            print(f"[Aligner] Failed to align after {iterations} iterations")
            if self.display:
                self.display.fill(0)
                self.display.text("Align failed", 10, 20)
                self.display.show()
            return False
    
    def align_and_approach(self, final_distance_cm=10):
        """
        Align perpendicular to obstacle, then approach to a specific distance.
        
        Args:
            final_distance_cm: Target distance from obstacle in cm (estimated)
        """
        # First alignment
        if not self.align_perpendicular(approach_first=True):
            return False
        
        # Optional: move forward to final position
        # This is approximate - you may want to use encoders for precision
        forward_time = final_distance_cm * 0.08  # rough estimate
        print(f"[Aligner] Approaching to {final_distance_cm}cm...")
        self._safe_set_speeds(1200, 1200)
        time.sleep(forward_time)
        self._safe_off()
        
        # Fine-tune alignment again
        return self.align_perpendicular(approach_first=False)


def test_alignment():
    """Test function for standalone use."""
    motors = robot.Motors()
    proximity = robot.ProximitySensors()
    display = robot.Display()
    
    aligner = ObstacleAligner(motors, proximity, display)
    
    print("Place robot in front of obstacle and it will align perpendicular.")
    print("Starting in 2 seconds...")
    time.sleep(2)
    
    aligner.align_perpendicular(approach_first=True, approach_distance_sec=0.5)
    
    print("Alignment complete!")


if __name__ == "__main__":
    test_alignment()