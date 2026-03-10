# track_main.py - ORIGINAL CODE
import json
from zumo_2040_robot import robot, proximity_sensors
import motor_controls as turns
import target_distance as move
import config
import time
import gyro_control as gyroc
from obstacle_detector import ObstacleDetector

CONFIG_FILE = "track_map.json"

# ---------------- Hardware ----------------
display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
encoders = robot.Encoders()
imu = robot.IMU()
imu.reset()
imu.enable_default()
time.sleep(0.25)  # small delay for sensor to stabilize

proximity = proximity_sensors.ProximitySensors()
# Create objects
turner = turns.TurnController(
    motors=motors,
    encoders=encoders,
    display=display,
    turn_speed=config.TURN_SPEED,
    turn_ticks_90=config.TURN_TICKS_90
)

follower = move.LineFollower(
    line_sensors=line_sensors,
    motors=motors,
    encoders=encoders,
    display=display
)

gyro =gyroc.GyroController(
    line_sensors=line_sensors,
    motors=motors,
    encoders=encoders,
    display=display,
    imu=imu
)
# Somewhere in your  setup/init
obstacle_detector = ObstacleDetector(proximity)

def load_track(filename=CONFIG_FILE):
    """Load the track steps from JSON."""
    with open(filename, "r") as f:
        return json.load(f)

def set_ignore_obstacles(state: bool):
    """Temporarily enable/disable obstacle detection."""
    config.IGNORE_OBSTACLES = state
    print(f"[Config] IGNORE_OBSTACLES set to {state}")

def run_track(track):
    """Execute each step from the loaded track list."""
    i = 0
    while i < len(track):
        step = track[i]

        # --- Normalize the step into (cmd, value, ignore) ---
        ignore = False
        if isinstance(step, dict):
            cmd = step.get("cmd")
            value = step.get("value")
            ignore = bool(step.get("ignore", False))
        else:
            # list/tuple form: ["F", 45] or ["F", 45, {"ignore": true}]
            cmd = step[0]
            value = step[1]
            if len(step) >= 3 and isinstance(step[2], dict):
                ignore = bool(step[2].get("ignore", False))

        # Predict next turn direction for obstacle avoidance
        next_turn_direction = None
        next_turn_angle = None  # make sure it's always defined
        if i + 1 < len(track):
            nxt = track[i + 1]
            if isinstance(nxt, dict):
                nxt_cmd = nxt.get("cmd")
                nxt_val = nxt.get("value")
            else:
                nxt_cmd = nxt[0]
                nxt_val = nxt[1]

            if nxt_cmd in ("L", "R"):
                next_turn_direction = nxt_cmd
                next_turn_angle = nxt_val

        # Apply the per-step ignore flag (on for this step only)
        set_ignore_obstacles(ignore)

        if cmd in ("F", "B"):
            print(f"{'Forward' if cmd == 'F' else 'Backward'} {value} cm")
            result = turner.follow_straight_with_heading_hold(
                imu=imu,
                obstacle_detector=obstacle_detector,
                target_cm=value,
                direction=cmd,
                turn_direction=next_turn_direction,
                turn_angle=next_turn_angle
            )
            # After the move, restore default (detect obstacles)
            set_ignore_obstacles(False)

            if result == "obstacle_handled":
                print("[Track] Obstacle handled. Skipping next turn.")
                i += 2  # Skip the next turn – it was already done
            else:
                i += 1

        elif cmd == "L":
            print(f"Left turn {value}°")
            turner.gyro_turn_with_line_lock(value, imu)
            # Restore default after the step
            set_ignore_obstacles(False)
            i += 1

        elif cmd == "R":
            print(f"Right turn {value}°")
            turner.gyro_turn_with_line_lock(-value, imu)
            # Restore default after the step
            set_ignore_obstacles(False)
            i += 1

        else:
            print("Unknown command:", cmd)
            # Restore default just in case
            set_ignore_obstacles(False)
            i += 1

def run_track1(track):
    """Execute each step from the loaded track list."""
    for cmd, value in track:
        if cmd == "F" or cmd == "B":
            print(f"Forward {value} cm")
            # follower.follow_straight(value, cmd)
            turner.follow_straight_with_heading_hold(imu,obstacle_detector,value,cmd)
            # move.follow_line(value)  # moves forward exact distance
        elif cmd == "L":
            print(f"Left turn {value}°")
            # turner.smart_turn("L", value, follower)  # turn finishes internally
            turner.gyro_turn_with_line_lock(value,imu)
            # turner.gyro_turn(value, imu)
            # gyro.turn_left_90()
        elif cmd == "R":
            print(f"Right turn {value}°")
            # turner.smart_turn("R", value, follower)  # turn finishes internally
            turner.gyro_turn_with_line_lock(-value,imu)
            # turner.gyro_turn(-value, imu)
            # gyro.turn_right_90()
        else:
            print("Unknown command:", cmd)

        # No sleep, no "Done" display – moves immediately to next step


# ---------------- Program Start ----------------
# Load route and run
routes = load_track()
route = routes.get(config.RUN_TRACK_NAME)

# Calibrate only ONCE
# follower.calibrate_line_sensors()

run_track(route)

# Finished
display.fill(0)
display.text("Finished!", 20, 20)
display.show()
print("Track complete ")