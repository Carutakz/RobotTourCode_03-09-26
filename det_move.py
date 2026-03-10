# det_move.py
# Execute steps from track_map.json:
#   ["F", <cm>], ["R", <deg>], ["L", <deg>], ...
# While executing "F", if proximity reports "near", abort that forward and
# immediately continue with the next step.
#
# Requires:
#   - zumo_2040_robot: Motors, Encoders, ProximitySensors
#   - config.py       : TICKS_PER_CM, DRIVE_SPEED (optional)
#   - motor_controls.py: turn_angle(direction="L"/"R", angle_degrees)

import json
import time
from zumo_2040_robot import robot
import config
import motor_controls as turns   # uses your calibrated encoder ticks for turning

# ======= Tunables (adjust for your robot) =======
# Distance control
DRIVE_SPEED    = max(1800, getattr(config, "DRIVE_SPEED", 2000))
TICKS_PER_CM   = int(getattr(config, "TICKS_PER_CM", 140))

# Proximity sensitivity (dual-polarity; calibrated at start)
CAL_TIME_S     = 0.8     # seconds to observe baseline (keep front clear)
REQUIRED_HITS  = 1       # 1 = snappier stops; raise to 2–3 to de-bounce more
HYST_FRACTION  = 0.30
POLL_PERIOD    = 0.02

# Make detection happen CLOSER by increasing these:
MIN_DELTA      = 6.0     # absolute minimum delta to trigger
PCT_DELTA      = 0.45    # fraction of baseline that must change
DELTA_MULT     = 1.6     # scale the required change; higher => detects closer

# Safety fallbacks so we never “drive forever” if encoders fail
STALL_TIMEOUT_S = 0.8                    # stall if no tick progress this long
CM_PER_SEC_EST  = 18.0                   # rough cm/s at DRIVE_SPEED
TIMEOUT_MARGIN  = 1.8                    # forward timeout = (cm/est)*margin

CONFIG_FILE = "track_map.json"
DEBUG = False
# ===============================================


# ----------------- Utilities -----------------
def load_track(filename=CONFIG_FILE):
    with open(filename, "r") as f:
        return json.load(f)

def safe_set_speeds(motors, left, right):
    for nm in ("set_speeds", "setMotorSpeeds", "setSpeed", "run"):
        fn = getattr(motors, nm, None)
        if callable(fn):
            try:
                return fn(int(left), int(right))
            except Exception:
                pass
    raise RuntimeError("No motor speed setter found.")

def safe_off(motors):
    for nm in ("off", "stop", "brake"):
        fn = getattr(motors, nm, None)
        if callable(fn):
            try:
                fn(); return
            except Exception:
                pass
    try:
        safe_set_speeds(motors, 0, 0)
    except Exception:
        pass

def _pair_sums(counts):
    """Return [sum(pair0), sum(pair1), ...] from prox.counts (left+right per pair)."""
    out = []
    try:
        for pair in counts:
            out.append(int(pair[0]) + int(pair[1]))
    except Exception:
        pass
    return out
# ---------------------------------------------


# ------------- Proximity calibration -------------
def calibrate_proximity_allpairs(prox, seconds=CAL_TIME_S):
    """
    Calibrate *all* proximity pairs. We create per-pair dual thresholds so we
    trigger when values go UP or DOWN from baseline (some surfaces reflect less).
    Returns: list of dicts per pair: {"on_up","off_up","on_dn","off_dn"}.
    """
    print("Proximity calibration: keep the front clear...")
    t0 = time.time()
    samples = []

    while time.time() - t0 < seconds:
        try:
            prox.read()
            sums = _pair_sums(prox.counts)
        except Exception:
            sums = []
        if sums:
            samples.append([float(v) for v in sums])
        time.sleep(0.01)

    if not samples:
        raise RuntimeError("Proximity not readable — check wiring/driver.")

    num_pairs = max(len(row) for row in samples)
    per_pair = [[] for _ in range(num_pairs)]
    for row in samples:
        for i in range(num_pairs):
            if i < len(row):
                per_pair[i].append(row[i])

    thresholds = []
    for i in range(num_pairs):
        data = per_pair[i]
        if not data:
            thresholds.append({"on_up": 1e9, "off_up": 1e9, "on_dn": -1e9, "off_dn": -1e9})
            continue

        base = sum(data) / float(len(data))
        rise = max(MIN_DELTA, base * PCT_DELTA) * DELTA_MULT

        thresholds.append({
            "on_up":  base + rise,
            "off_up": base + rise * (1.0 - HYST_FRACTION),
            "on_dn":  base - rise,
            "off_dn": base - rise * (1.0 - HYST_FRACTION),
        })

    if DEBUG:
        for i, t in enumerate(thresholds):
            print("P{} up(on/off)={:.1f}/{:.1f}  dn(on/off)={:.1f}/{:.1f}".format(
                i, t["on_up"], t["off_up"], t["on_dn"], t["off_dn"]
            ))
    return thresholds
# -------------------------------------------------


# ------------- Forward (interruptible) -------------
def forward_cm_interruptible(motors, enc, prox, cm, speed, thresholds):
    """
    Drive forward up to 'cm'. If ANY prox pair crosses threshold, stop *immediately*
    and return False so the caller can advance to the next step.
    Return True only if the full distance was reached.
    """
    enc.get_counts(reset=True)
    ticks_target = int(cm * TICKS_PER_CM)

    t0 = time.time()
    last_ticks = 0
    last_change_t = t0
    max_time = (cm / max(1e-6, CM_PER_SEC_EST)) * TIMEOUT_MARGIN

    safe_set_speeds(motors, speed, speed)
    hits = 0

    try:
        while True:
            now = time.time()

            # Distance via encoders
            l, r = enc.get_counts()
            avg = (abs(l) + abs(r)) // 2
            if avg >= ticks_target:
                safe_off(motors)
                return True

            if avg > last_ticks:
                last_ticks = avg
                last_change_t = now

            # Proximity check (ALL pairs, dual polarity)
            near = False
            try:
                prox.read()
                sums = _pair_sums(prox.counts)
            except Exception:
                sums = []

            if sums:
                for i, val in enumerate(sums):
                    if i >= len(thresholds):
                        continue
                    t = thresholds[i]
                    if (val >= t["on_up"]) or (val <= t["on_dn"]):
                        near = True
                        if DEBUG:
                            print("[PROX] pair {} val={} -> NEAR".format(i, val))
                        break

            if near:
                hits += 1
                if hits >= REQUIRED_HITS:
                    safe_off(motors)
                    return False  # interrupted — caller should go to next step
            else:
                hits = 0

            # Stall/timeout guards
            if (now - last_change_t) > STALL_TIMEOUT_S:
                if DEBUG:
                    print("[STALL] finishing remaining distance by time")
                remaining_cm = max(0.0, (ticks_target - avg) / float(max(1, TICKS_PER_CM)))
                safe_set_speeds(motors, speed, speed)
                time.sleep(remaining_cm / max(1e-6, CM_PER_SEC_EST))
                safe_off(motors)
                return True

            if (now - t0) > max_time:
                if DEBUG:
                    print("[TIMEOUT] segment timeout; moving on")
                safe_off(motors)
                return True

            time.sleep(POLL_PERIOD)

    finally:
        safe_off(motors)
# --------------------------------------------------


# ------------- Turns (delegate to your calibrated encoder turn) -------------
def main():
    # Hardware
    motors = robot.Motors()
    enc    = robot.Encoders()
    prox   = robot.ProximitySensors()
    mc = turns.MotorControls()
    # Calibrate proximity once at start
    thresholds = calibrate_proximity_allpairs(prox)

    # Load route from track_map.json (e.g., [["F",100],["R",90],["F",50],["L",90],["F",100]])
    route = load_track()
    print("Route:", route)

    for step in route:
        if not isinstance(step, (list, tuple)) or len(step) != 2:
            print("Skipping malformed step:", step)
            continue

        cmd, value = step
        if cmd == "F":
            print("Forward {} cm (interruptible)".format(value))
            reached = forward_cm_interruptible(motors, enc, prox, float(value), DRIVE_SPEED, thresholds)
            if reached:
                print("  Reached full {} cm".format(value))
            else:
                print("  Object detected — advancing to next step.")
            time.sleep(0.12)

        elif cmd == "R":
            print("Right turn {} deg".format(value))
            mc.turn_angle("R", value)
            time.sleep(0.12)

        elif cmd == "L":
            print("Left turn {} deg".format(value))
            mc.turn_angle("L", value)
            time.sleep(0.12)

        else:
            print("Unknown cmd:", cmd)

    print("Track complete.")


main()