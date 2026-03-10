# detect_object_counts_turn90.py
# Proximity-only object avoidance with alternating 90-degree turns:
#  - Calibrate baseline on front counts
#  - Drive forward forever
#  - On detect: stop -> back up ~15 cm -> turn 90 deg (R then L then R...) -> resume

import time
from zumo_2040_robot import robot
import config

# ------------------ TUNABLES ------------------
FORWARD_SPEED = max(2400, getattr(config, "DRIVE_SPEED", 2000))
SEC_PER_CM    = 0.08     # tune for your robot

TURN_SPEED            = 1200
TURN_DEGREES          = 90      # target angle both directions
SEC_PER_DEGREE_RIGHT  = 0.020   # seconds per degree (RIGHT) at TURN_SPEED
SEC_PER_DEGREE_LEFT   = 0.0133  # seconds per degree (LEFT)  at TURN_SPEED  (scaled 0.020 * 90/135)

CAL_TIME_S    = 1.0
POLL_PERIOD   = 0.04
REQUIRED_HITS = 2
HYST_FRACTION = 0.35
DEBUG         = True

PROX_FRONT_INDEX = getattr(config, "PROX_FRONT_INDEX", 1)

# Sensitivity
SENSITIVITY_MODE = getattr(config, "SENSITIVITY_MODE", "near")
SENSITIVITY_CONFIG = {
    "near":   {"brightness_levels": [313, 800, 1500, 2500, 3500, 5000], "min_delta": 12.0, "pct_delta": 1.10},
    "medium": {"brightness_levels": [313, 1000, 2063, 3500, 5375, 7563], "min_delta": 3.0, "pct_delta": 0.40},
    "far":    {"brightness_levels": [500, 1500, 3000, 5000, 8000, 10000], "min_delta": 2.0, "pct_delta": 0.25},
}
CONF = SENSITIVITY_CONFIG.get(SENSITIVITY_MODE, SENSITIVITY_CONFIG["medium"])
DEFAULT_BRIGHTNESS_LEVELS = CONF["brightness_levels"]
MIN_DELTA = CONF["min_delta"]
PCT_DELTA = CONF["pct_delta"]
# ------------------------------------------------


# ---------- SAFE HELPERS ----------
def safe_set_speeds(motors, left, right):
    for nm in ("set_speeds", "setMotorSpeeds", "setSpeed", "run"):
        fn = getattr(motors, nm, None)
        if callable(fn):
            try:
                return fn(left, right)
            except TypeError:
                return fn(int(left), int(right))
    raise RuntimeError("Motors driver has no valid speed setter method.")

def safe_off(motors):
    for nm in ("off", "stop", "brake"):
        fn = getattr(motors, nm, None)
        if callable(fn):
            try:
                return fn()
            except:
                pass
    return safe_set_speeds(motors, 0, 0)
# ----------------------------------


# ---------- PROX HELPERS ----------
def _front_sum_from_counts(counts, front_index=PROX_FRONT_INDEX):
    try:
        pair = counts[front_index]
        return int(pair[0]) + int(pair[1])
    except:
        return None
# ----------------------------------


# ---------- CALIBRATION ----------
def calibrate_baseline_counts(prox, seconds=CAL_TIME_S):
    print("Calibrating... clear front area.")
    t0 = time.time()
    samples = []
    while time.time() - t0 < seconds:
        prox.read()
        v = _front_sum_from_counts(prox.counts, PROX_FRONT_INDEX)
        if v is not None:
            samples.append(v)
        time.sleep(0.01)
    if not samples:
        raise RuntimeError("Proximity front channel not readable.")
    base = sum(samples) / len(samples)
    rise_delta = max(MIN_DELTA, base * PCT_DELTA)
    thr_on  = base + rise_delta
    thr_off = base + rise_delta * (1.0 - HYST_FRACTION)
    print("Calibrated: base={:.2f} on={:.2f} off={:.2f}".format(base, thr_on, thr_off))
    return base, thr_on, thr_off
# ----------------------------------


# ---------- ACTIONS ---------#

def turn_degrees(motors, direction, degrees=TURN_DEGREES, speed=TURN_SPEED):
    # Use per-direction timing to hit 90 deg accurately
    if direction.upper() == "R":
        dur = float(degrees) * SEC_PER_DEGREE_RIGHT
        l, r = int(speed), -int(speed)
    else:  # LEFT
        dur = float(degrees) * SEC_PER_DEGREE_LEFT
        l, r = -int(speed), int(speed)
    safe_set_speeds(motors, l, r)
    time.sleep(dur)
    safe_off(motors)
# ----------------------------------


def main():
    motors = robot.Motors()
    prox   = robot.ProximitySensors()

    base, thr_on, thr_off = calibrate_baseline_counts(prox)

    # Alternate turns: R, L, R, L, ...
    next_turn_dir = "R"

    safe_set_speeds(motors, FORWARD_SPEED, FORWARD_SPEED)
    print("Driving forward; on object: back up and turn 90 deg, alternating R/L.")

    near, hits = False, 0
    try:
        while True:
            prox.read()
            val = _front_sum_from_counts(prox.counts, PROX_FRONT_INDEX)
            if val is None:
                time.sleep(POLL_PERIOD)
                continue

            if DEBUG:
                print("Front_sum={} (on={:.1f}/off={:.1f}) next_turn={}".format(val, thr_on, thr_off, next_turn_dir))

            if not near and val >= thr_on:
                hits += 1
                if hits >= REQUIRED_HITS:
                    near, hits = True, 0
            elif near and val <= thr_off:
                near, hits = False, 0

            if near:
                print("Object detected ({}). Backing up and turning 90 deg {}."
                      .format(val, "RIGHT" if next_turn_dir == "R" else "LEFT"))
                safe_off(motors)
                turn_degrees(motors, next_turn_dir, TURN_DEGREES, TURN_SPEED)

                # Alternate direction
                next_turn_dir = "L" if next_turn_dir == "R" else "R"

                # Resume forward and reset detection state
                safe_set_speeds(motors, FORWARD_SPEED, FORWARD_SPEED)
                near, hits = False, 0
                time.sleep(0.2)

            time.sleep(POLL_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        safe_off(motors)
        print("Stopped.")


main()