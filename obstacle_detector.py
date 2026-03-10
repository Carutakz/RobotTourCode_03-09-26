
# obstacle_detector.py

OBJECT_THRESHOLD = 6  # Set your threshold here

class ObstacleDetector:
    def __init__(self, proximity_sensor):
        self.proximity = proximity_sensor

    def is_obstacle_ahead(self):
        self.proximity.read()
        left = self.proximity.front_counts_with_left_leds()
        right = self.proximity.front_counts_with_right_leds()
        print(f"[Proximity] Left: {left}, Right: {right}")
        return (left is not None and left > OBJECT_THRESHOLD) or \
            (right is not None and right > OBJECT_THRESHOLD)
