#!/usr/bin/env python3
"""Ball-touch robot: 100% autonomous, Raspberry Pi 4 + 4x mecanum + top camera.

  python3 bot.py         run: count down 5 s, then go
  python3 bot.py --button wait for the start button instead (rule 2.2 - use at the venue)
  python3 bot.py --stream watch the camera from a laptop: http://<pi>:8000/
  python3 bot.py --object find the ball with a YOLO model (best.pt) instead of colour
  python3 bot.py --check  prove numpy/cv2/camera/IMU work on this board
  python3 bot.py --wheels bring-up: wheel directions, WHEELS OFF THE GROUND
  python3 bot.py --dry    no motors, prints what it would do (test on a laptop)
  python3 calibrate.py   tune the ball colour at the venue -> tune.json
  python3 calibrate_frame.py  tune the black walls and goal frame -> tune.json

Rules this code is built around:
  2.2  autonomous only, one button press, no remote / wifi
  5.x  touching and pushing are free; TRAPPING the ball forces the wheels to
       stop for <=5 s, so we never trap - the front stays open on purpose
  10.1 pinning another robot on the wall is a foul -> back off when close
  12.3 don't park in front of your own goal -> the retreat is capped
"""
import json, math, os, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
DRY = "--dry" in sys.argv


def argval(flag, default=None):
    """Value after `flag`, or default. `--stream 8080` -> "8080"."""
    if flag in sys.argv:
        i = sys.argv.index(flag)
        if i + 1 < len(sys.argv) and not sys.argv[i + 1].startswith("-"):
            return sys.argv[i + 1]
    return default


# Counting down is the default because that is how the robot is run on the bench and
# over ssh, where there is nobody standing at the robot to press anything.
#
# AT THE VENUE, PASS --button. Rule 2.2 requires one physical press to start and you
# fail inspection without it; ballbot.service passes it for exactly that reason. A
# toggle switch on GPIO 4 works with --button too - flip it on, then off - but leaving
# it on reads as "pressed" forever, which is what the countdown path is here to escape.
NOBUTTON = "--button" not in sys.argv
START_COUNTDOWN = 5   # seconds, long enough to get your hands clear
# Rule 2.2 also bans wifi during a match, so the video stream is opt-in and never runs
# unless you ask for it. Bench and practice only.
STREAM = "--stream" in sys.argv
STREAM_PORT = int(argval("--stream", 8000))
# --object swaps the HSV colour window for a trained detector. `--object` alone loads
# TUNE["obj_model"]; `--object best_ncnn_model` names another model for this run.
OBJECT = "--object" in sys.argv
OBJECT_MODEL = argval("--object")

# --- pins (BCM). Motor = (forward, backward, enable/pwm). Match your driver board.
# TWO L298N boards, one motor per mecanum wheel, so all four turn independently -
# which is the whole point: strafing needs an X pattern, and ganged wheels can't make
# one. Board 1 carries FL+FR, board 2 carries BL+BR.
#
# On each board ENA gates OUT1/OUT2 (the IN1/IN2 pair) and ENB gates OUT3/OUT4
# (IN3/IN4). Enable is an active-high gate on the H-bridge: the IN pins only choose a
# direction, they pass no current of their own, so an undriven enable means the wheels
# sit dead while every IN pin measures exactly right. Name the enable pin here.
#
# Never write None in the third slot for a pin that is actually wired - that tells
# gpiozero to leave it floating, the gate reads low, and nothing turns. None is only
# for a board whose enable is jumpered to 5 V and has no wire to the Pi at all.
MOTORS = {"FL": (5, 6, 12), "FR": (13, 19, 18), "BL": (16, 20, 21), "BR": (23, 24, 25)}
BUTTON_PIN = 4

# --- everything worth changing trackside lives in tune.json
DEFAULTS = {
    # Measured off the ball photos: hue sits in a tight 38-42 band, but WHITE TILE IS
    # WHAT WE ACTUALLY PLAY ON, and white has no hue - it just has low saturation
    # (floor p98 = 119, ball p2 = 156). So S_lo is the wall that keeps the floor out;
    # H is widened past the measurement to survive the venue's white balance.
    "hsv_lo": [32, 140, 80], "hsv_hi": [50, 255, 255],
    "min_area": 60,          # px, ignore specks
    "min_round": 0.72,       # blob area / enclosing-circle area. Ball ~.85, chassis way under.
                             # Raise it if we chase the rival, lower it if we ignore the ball.
    "close_radius": 34,      # ball radius in px (at 320 wide) that means "we are on it"
    "speed": 0.75,           # base power 0..1
    "strafe_gain": 1.2,      # how hard we slide sideways onto the ball
    "turn_gain": 1.4,        # how hard we rotate onto the goal heading
    "search_spin": 0.45,
    "shoot_deg": 14,         # heading error we accept before charging the goal
    "invert_strafe": 1,      # flip to -1 if it strafes the wrong way
    "invert_turn": 1,        # flip to -1 if it spins the wrong way
    "min_duty": 0.22,        # below this a motor just buzzes
    "blind_std": 12,         # frame contrast below this = lens covered
    "blind_frames": 15,      # ~0.5 s of that before we believe it
    "imu_sign": 1,           # flip to -1 if the MPU-6050 is mounted upside down
    "gyro_settle": 0.3,      # s to let the button shove die out before sampling bias
    "ball_memory": 25,       # frames to keep pushing after the plow hides the ball
    "motion_min": 2.5,       # frame-to-frame pixel change below this = we aren't moving
    "stuck_secs": 2.5,       # how long that has to hold before we thrash free
    # --- the black field frame: walls, goal posts, crossbar. Tuned with
    # calibrate_frame.py. Black is the dark end of the scale and its hue and
    # saturation are mostly sensor noise, so V hi is the number that matters.
    "frame_lo": [0, 0, 0], "frame_hi": [179, 255, 70],
    # --- goal detection: the opening in the black frame. find_goal() only trusts a
    # gap it can see BOTH edges of, so it never mistakes "wall ran off the edge of
    # the picture" for an open goal.
    "goal_wall_frac": 0.3,   # a row counts as "wall" once this fraction of it is black
    "goal_min_gap": 10,      # px wide (at 320 wide), ignore anything narrower as noise
    "goal_gain": 1.3,        # like turn_gain but keyed off screen fraction, not degrees
    "shoot_dx": 0.12,        # goal off-centre by less than this reads as "aimed"
    # --- USB webcam controls. These live on the device, not in the program, so they
    # reset on a replug or a reboot unless we set them at every start.
    #
    # Measured on this camera by sweeping focus and scoring sharpness: the FAR field
    # (top of frame) peaks at 64, the NEAR field (bottom) peaks at 320, and past 192
    # the far field collapses and never recovers. There is no value that is sharp at
    # both, so 128 is the compromise that leaves each at about 45% of its own peak.
    # null hands focus back to the camera's continuous autofocus, which hunts.
    "cam_focus": 128,
    # Auto white balance is the quiet killer of a tuned HSV window: it shifts hue as
    # the scene changes, so the colour you calibrated stops being the colour you see.
    # Lock it, and re-run calibrate.py if the venue's lights are different.
    "cam_lock_wb": 1,
    # --- --object mode: a YOLO detector finds the ball instead of the colour window.
    # A .pt runs on PyTorch, about 3.5 fps at 320 on a Pi 4. Export it once to NCNN
    # for roughly three times that, then point obj_model at the folder it writes:
    #   yolo export model=best.pt format=ncnn imgsz=320   -> best_ncnn_model/
    "obj_model": "best.pt",  # relative paths are tried from the cwd, then next to bot.py
    "obj_class": "ball",     # class name, or list of names, that counts as the ball.
                             # best.pt also knows "car", and a car is a rival robot we
                             # must not chase. null takes ANY class - placeholders only
    "obj_conf": 0.4,         # detections below this confidence are ignored
    "obj_imgsz": 320,        # inference size; the frame is already 320 wide
    "obj_max_age": 0.8,      # s; a detection older than this is not steered on
}
_tf = os.path.join(HERE, "tune.json")
TUNE = {**DEFAULTS, **(json.load(open(_tf)) if os.path.exists(_tf) else {})}


# ---------------------------------------------------------------- pure logic
def clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


def mix(vx, vy, w):
    """X-mecanum mixer. vx=+forward, vy=+left, w=+counter-clockwise -> FL,FR,BL,BR."""
    fl, fr, bl, br = vx - vy - w, vx + vy + w, vx + vy - w, vx - vy + w
    m = max(1.0, abs(fl), abs(fr), abs(bl), abs(br))
    return fl / m, fr / m, bl / m, br / m


def angle_diff(target, current):
    """Signed degrees from current to target, in [-180, 180)."""
    return (target - current + 180) % 360 - 180


def gyro_step(h, rate, dt, sign=1):
    """Fold a turn rate (deg/s, + = counter-clockwise) into a clockwise compass
    heading, so an MPU-6050 reads the same way round as a BNO055."""
    return (h - rate * dt * sign) % 360


def should_escape(still, blind, asking_to_move, stalled_for):
    """Stuck means the WORLD stopped moving while we asked the wheels for movement.
    Do not measure this with the ball's size: pushing the ball down the field holds
    its radius rock steady for seconds, and that is the one moment we must never
    interrupt. Needs a working camera, so a blinded robot never calls itself stuck."""
    return still and not blind and asking_to_move and stalled_for > TUNE["stuck_secs"]


def decide(ball, herr, st, blind=False, goal=None):
    """ball = (dx in -1..1, radius_px) or None.
    herr = degrees we must turn counter-clockwise to face the enemy goal, from the
    gyro (None = no IMU).
    goal = (dx in -1..1, gap_px) for the goal mouth as find_goal() actually saw it
    in THIS frame, or None if it is not in view. It is ground truth with none of
    the gyro's drift, so whenever we can see it, it overrides the compass estimate.
    blind = the camera can't see anything (covered, knocked, or blown out).
    Returns (vx, vy, w).

    No sonar and no kicker on this robot: we push the ball in, and the stuck-detector
    in play() is what digs us out of a wall grind."""
    T = TUNE
    st["n"] = st.get("n", 0) + 1
    aim_gyro = 0.0 if herr is None else clamp(herr * T["turn_gain"] / 90.0)
    aim = aim_gyro if goal is None else clamp(-goal[0] * T["goal_gain"])
    aligned = (abs(goal[0]) < T["shoot_dx"]) if goal is not None else (
        herr is None or abs(herr) < T["shoot_deg"])
    if blind:
        # Can't find the ball, but the compass still knows where the goal is: push
        # that way and sweep, so a dead camera costs us the match instead of the game.
        sweep = 1.0 if (st["n"] // 30) % 2 == 0 else -1.0
        return (T["speed"] * 0.8, sweep * 0.35, aim)
    if ball is None:
        # At point-blank range the plow and the camera's own blind spot swallow the
        # ball. Losing it while it was CLOSE means it's on our nose, not gone - back
        # off now and you shove it away and chase it forever. Keep pushing briefly.
        if (st.get("seen_r", 0) >= T["close_radius"]
                and st["n"] - st.get("seen_n", -999) < T["ball_memory"]):
            return (T["speed"], 0.0, aim)
        # No ball in view: still searching, but if we know which way the goal is
        # (compass, or the goal frame itself), bend the search toward it instead of
        # spinning blind off the last side the ball happened to vanish on.
        return (-0.12, 0.0, clamp(st.get("spin", 1) * T["search_spin"] + aim))
    dx, r = ball
    st["spin"] = 1 if dx < 0 else -1
    st["seen_n"], st["seen_r"] = st["n"], r
    if r >= T["close_radius"]:
        if aligned:
            return (T["speed"], -dx * 0.35, aim)                   # lined up: drive through it
        # Orbit around the ball to face the goal. A small correction, not just the
        # turn direction: a big turn takes a while, and if the ball drifts toward
        # the frame edge mid-orbit and drops out of view, decide() falls back to
        # blind "push straight ahead" - which can shove the ball at exactly the
        # wrong angle. Nudging the strafe by dx keeps it roughly centred while
        # the turn plays out, same idea as the far-ball branch below.
        s = 1.0 if aim > 0 else -1.0
        orbit = clamp(-s * 0.9 - dx * 0.4)
        return (0.15, orbit * T["speed"], aim)                     # orbit around the ball
    return (T["speed"] * (1 - 0.45 * abs(dx)), -dx * T["strafe_gain"], aim * 0.5)


# ---------------------------------------------------------------- hardware
class Gyro:
    """MPU-6050 fallback. Six axes, no magnetometer, so this is dead reckoning: it
    tells you how far you have turned since start, not where north is, and the error
    creeps up over the match. Two things keep it usable:
      - the bias calibration below (skip it and you drift degrees per SECOND)
      - rule 6.3 restarts the match after every goal, so you re-aim and re-zero often
    """
    ADDR, SCALE = 0x68, 131.0   # LSB per deg/s at the default +-250 deg/s range

    def __init__(self, bus=None):
        if bus is None:
            try:
                from smbus2 import SMBus
            except ImportError:
                from smbus import SMBus   # apt's python3-smbus, same two calls we make
            bus = SMBus(1)
            bus.write_byte_data(self.ADDR, 0x6B, 0)   # wake it up
            time.sleep(0.1)
        self.bus, self.bias = bus, 0.0
        self.zero()

    def zero(self):
        """Re-measure the resting bias, robot STILL. Called at every start press, not
        at boot: systemd starts us while the robot is still being carried to the field,
        and a bias measured while moving poisons the heading for the whole match."""
        print("calibrating gyro - keep the robot COMPLETELY still")
        # The button press is a physical shove, and this runs the instant your finger
        # leaves it. 200 reads take ~0.1 s, so without this the robot is still rocking
        # through the whole sample and the wobble averages straight into the bias -
        # which then drifts the heading for the entire segment, silently.
        time.sleep(TUNE["gyro_settle"])
        self.bias = sum(self._raw() for _ in range(200)) / 200.0
        self.h, self.t = 0.0, time.time()

    def _raw(self):
        hi, lo = self.bus.read_i2c_block_data(self.ADDR, 0x47, 2)  # GYRO_ZOUT
        v = (hi << 8) | lo
        return v - 65536 if v > 32767 else v

    def heading(self):
        now = time.time()
        dt = min(now - self.t, 0.1)   # a long stall shouldn't integrate one sample forever
        self.h = gyro_step(self.h, (self._raw() - self.bias) / self.SCALE, dt,
                           TUNE["imu_sign"])
        self.t = now
        return self.h


class Robot:
    """Every device is optional; whatever isn't wired up degrades to None."""

    def __init__(self):
        self.motors = self.button = None
        self.heading_fn = self.gyro = None
        if DRY:
            return
        from gpiozero import Motor, Button
        # enable=None means the board's enable is jumpered to 5 V and no wire reaches
        # the Pi. Say so out loud: if a wire IS there, this is the silent failure where
        # the IN pins read perfect on a meter and the wheels never move.
        self.motors = []
        for name, (f, b, e) in MOTORS.items():
            if e is None:
                print("%s: enable not driven - correct ONLY if ENA/ENB is jumpered to 5 V"
                      % name)
                self.motors.append(Motor(f, b, pwm=True))
            else:
                self.motors.append(Motor(f, b, enable=e, pwm=True))
        self.button = Button(BUTTON_PIN)
        try:
            import board, adafruit_bno055
            bno = adafruit_bno055.BNO055_I2C(board.I2C())
            self.heading_fn = lambda: bno.euler[0]
        except Exception:
            try:
                self.gyro = Gyro()
                self.heading_fn = self.gyro.heading
            except Exception as e:  # no IMU at all -> chase the ball, no goal lock
                print("no IMU (%s); heading lock off" % e)

    def zero_heading(self):
        if self.gyro:
            self.gyro.zero()

    def drive(self, vx, vy, w):
        vy *= TUNE["invert_strafe"]
        w *= TUNE["invert_turn"]
        vals = mix(vx, vy, w)
        if DRY:
            print("vx%+.2f vy%+.2f w%+.2f -> %s" %
                  (vx, vy, w, " ".join("%+.2f" % v for v in vals)))
            return
        for m, v in zip(self.motors, vals):
            m.value = 0.0 if abs(v) < 0.03 else math.copysign(
                max(abs(v), TUNE["min_duty"]), v)

    def stop(self):
        self.drive(0, 0, 0)

    def heading(self):
        """Degrees, counting clockwise, or None if no IMU is fitted."""
        return None if self.heading_fn is None else self.heading_fn()



# ---------------------------------------------------------------- vision
def cam_control(name, value, dev="/dev/video0"):
    """Set one V4L2 control on a USB webcam. -> True if it took.

    v4l2-ctl rather than cv2's CAP_PROP_*, for one practical reason: it works on a
    device another process is already streaming from, which is what lets you tune
    focus while watching calibrate.py's own stream. Missing tool or missing control
    is not an error - plenty of cameras have neither, and the robot still plays."""
    import shutil, subprocess
    if not shutil.which("v4l2-ctl") or not os.path.exists(dev):
        return False
    r = subprocess.run(["v4l2-ctl", "-d", dev, "-c", "%s=%s" % (name, value)],
                       capture_output=True, text=True)
    return r.returncode == 0


def apply_cam_controls():
    """Pin focus and white balance, both of which otherwise drift mid-match."""
    if TUNE.get("cam_lock_wb"):
        cam_control("white_balance_automatic", 0)
    focus = TUNE.get("cam_focus")
    if focus is None:
        cam_control("focus_automatic_continuous", 1)
        return
    # Order matters: focus_absolute reads as inactive and is ignored while the
    # camera's own continuous autofocus still owns the lens.
    if cam_control("focus_automatic_continuous", 0):
        if cam_control("focus_absolute", int(focus)):
            print("camera: focus locked at %d" % int(focus))


def open_camera():
    import cv2
    try:
        from picamera2 import Picamera2
        p = Picamera2()
        p.configure(p.create_preview_configuration(main={"format": "RGB888", "size": (320, 240)}))
        p.start(); time.sleep(1)
        return lambda: p.capture_array()   # picamera2's "RGB888" hands back BGR, which cv2 wants
    except Exception:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        # A webcam is free to refuse the size and hand back its own - this one gives
        # 640x480 for a 320x240 request. That is not just four times the work: every
        # pixel threshold here (close_radius, min_area) was tuned at 320 wide, so at
        # 640 a ball a metre away already reads as "on it" and the robot starts
        # orbiting and charging from across the room. Shrink it back to the scale the
        # numbers mean. Keep the aspect ratio - squashing a 16:9 frame into 4:3 would
        # turn the ball into an ellipse and fail the min_round test.
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got[0] != 320:
            print("camera: asked for 320x240, got %dx%d - resizing each frame to 320 wide"
                  % got)
        apply_cam_controls()

        def grab():
            ok, f = cap.read()
            if not ok or f is None:
                return None
            if f.shape[1] != 320:
                f = cv2.resize(f, (320, f.shape[0] * 320 // f.shape[1]),
                               interpolation=cv2.INTER_AREA)
            return f
        return grab


def is_blind(frame):
    """Anything pressed against a lens defocuses to a flat blur, so a covered
    camera has almost no contrast. Also catches a dead camera or a dark frame."""
    return frame is None or frame.std() < TUNE["blind_std"]


def colour_mask(frame, lo, hi):
    """Blur, keep the pixels inside an HSV range, drop specks.

    One function for the ball, the black frame and both tuners, so the mask a tuner
    shows you is exactly the mask the robot steers on."""
    import cv2, numpy as np
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def frame_mask(frame):
    """Pixels that are the black field frame: walls, goal posts, crossbar."""
    return colour_mask(frame, TUNE["frame_lo"], TUNE["frame_hi"])


def find_ball(frame):
    """-> (dx in -1..1, radius_px) or None."""
    import cv2
    mask = colour_mask(frame, TUNE["hsv_lo"], TUNE["hsv_hi"])
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # A rival can be painted the ball's own green, and it is BIGGER than the ball -
    # so taking the largest green blob drives us straight into it. Only a ball fills
    # its own enclosing circle (a chassis, a sticker, or a ball merged with the robot
    # touching it all sit well under min_round), so screen on roundness FIRST and
    # take the largest survivor. Nothing round in frame -> we search, we don't charge.
    best = None
    for c in cnts:
        a = cv2.contourArea(c)
        if a < TUNE["min_area"]:
            continue
        (x, _y), r = cv2.minEnclosingCircle(c)
        if r < 1 or a / (math.pi * r * r) < TUNE["min_round"]:
            continue
        if best is None or a > best[0]:
            best = (a, x, r)
    if best is None:
        return None
    _a, x, r = best
    return (x / (frame.shape[1] / 2) - 1.0, r)


# ---------------------------------------------------------------- object detection
def pick_box(boxes, width, wanted=None):
    """boxes = [(x1, y1, x2, y2, conf, class_name)] -> (dx in -1..1, radius_px) or None.

    The same contract as find_ball, so decide() cannot tell which one found the ball.
    Radius is half the LONGER side: a ball cut off by the frame edge loses one side of
    its box, not the other, and close_radius must still read true when it's that near.
    The class filter is what keeps a rival robot out; among what passes, the most
    confident box wins, since a real field has only one ball in it."""
    if isinstance(wanted, str):
        wanted = [wanted]
    best = None
    for x1, y1, x2, y2, conf, name in boxes:
        if wanted and name not in wanted:
            continue
        if best is None or conf > best[0]:
            best = (conf, (x1 + x2) / 2.0, max(x2 - x1, y2 - y1) / 2.0)
    if best is None:
        return None
    _c, x, r = best
    return (x / (width / 2.0) - 1.0, r)


def model_path(name):
    """Find the model on disk, or raise. Checked here rather than left to ultralytics,
    which answers a missing name that looks like a stock model by downloading it -
    over the wifi rule 2.2 bans, and as a model that has never seen our ball."""
    for p in (name, os.path.join(HERE, name)):
        if os.path.exists(p):
            return p
    raise FileNotFoundError("no model %r in %s or %s" % (name, os.getcwd(), HERE))


class ObjectDetector:
    """Any model ultralytics can load: .pt, an _ncnn_model folder, .onnx, .tflite."""

    def __init__(self, name):
        from ultralytics import YOLO
        try:
            import torch   # leave one core for the control loop and the camera
            torch.set_num_threads(max(1, (os.cpu_count() or 4) - 1))
        except ImportError:
            pass
        self.path = model_path(name)
        self.model = YOLO(self.path, task="detect")
        # The first inference builds the graph and takes seconds. Pay it now, before
        # the countdown, not in the opening moments of the match.
        import numpy as np
        self.names = self._run(np.zeros((240, 320, 3), np.uint8)).names
        wanted = TUNE["obj_class"]
        missing = [w for w in ([wanted] if isinstance(wanted, str) else wanted or [])
                   if w not in self.names.values()]
        if missing:
            # A misspelt class name filters out every detection, and the robot just
            # spins searching forever with nothing on screen to say why.
            print("object: model has no class %s - it knows: %s"
                  % (missing, ", ".join(self.names.values())))
        print("object: %s, ball = %s" % (self.path, wanted or "ANY class (placeholder)"))

    def _run(self, frame):
        return self.model.predict(frame, imgsz=TUNE["obj_imgsz"], conf=TUNE["obj_conf"],
                                  verbose=False)[0]

    def detect(self, frame):
        """-> (dx in -1..1, radius_px) or None, like find_ball."""
        r = self._run(frame)
        boxes = [(*xyxy, conf, r.names[int(k)]) for xyxy, conf, k in
                 zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(), r.boxes.cls.tolist())]
        return pick_box(boxes, frame.shape[1], TUNE["obj_class"])


class AsyncDetector:
    """Runs the detector on its own thread, so a slow model cannot slow the loop.

    This is not about frame rate. Gyro.heading() caps each step at 0.1 s, so a loop
    that waited ~270 ms on inference would drop most of every turn from the heading
    and the robot would lose the goal. The loop hands in its newest frame and reads
    back the newest answer; the worker only ever runs on the latest frame, never a
    queue of old ones."""

    def __init__(self, detector):
        self.detector = detector
        self.cond = threading.Condition()
        self.frame, self.frame_t = None, 0.0
        self.ball, self.ball_t = None, 0.0
        self.running, self.thread = True, None

    def start(self):
        import atexit
        self.thread = threading.Thread(target=self._work, daemon=True)
        self.thread.start()
        # A daemon thread caught mid-inference when Python exits aborts the whole
        # process from inside torch ("terminate called without an active exception").
        # atexit runs on every way out - match over, --dry, Ctrl-C - so stop it there.
        atexit.register(self.close)
        return self

    def close(self):
        with self.cond:
            self.running = False
            self.cond.notify()
        if self.thread:
            self.thread.join(2.0)   # one inference at most

    def see(self, frame):
        """Queue `frame`, return the freshest detection. Drop-in for find_ball."""
        with self.cond:
            self.frame, self.frame_t = frame, time.time()
            self.cond.notify()
            fresh = time.time() - self.ball_t < TUNE["obj_max_age"]
            return self.ball if fresh else None

    def _work(self):
        while True:
            with self.cond:
                while self.frame is None and self.running:
                    self.cond.wait()
                if not self.running:
                    return
                frame, t = self.frame, self.frame_t
                self.frame = None
            try:
                ball = self.detector.detect(frame)
            except Exception as e:     # one bad frame must not kill the thread for good
                print("object: detect failed (%s)" % e)
                ball = None
            with self.cond:
                # Timestamped when the frame was CAPTURED, not when inference ended:
                # the ball has been moving for the whole time the model was thinking.
                self.ball, self.ball_t = ball, t


def ball_finder():
    """-> (fn(frame) -> ball, label). The detector, or colour if it can't be had."""
    if not OBJECT:
        return find_ball, "colour"
    try:
        det = ObjectDetector(OBJECT_MODEL or TUNE["obj_model"])
    except Exception as e:
        # Loud, but keep playing: a robot chasing by colour beats one that won't start.
        print("!" * 60)
        print("object: cannot load model (%s: %s)" % (type(e).__name__, e))
        print("object: FALLING BACK TO COLOUR TRACKING")
        print("!" * 60)
        return find_ball, "colour"
    return AsyncDetector(det).start().see, "object"


def find_goal(frame):
    """-> (dx in -1..1, gap_px) for the goal mouth, or None if it is not in view.

    The wall and the goal posts are the same black, so colour alone cannot tell
    them apart - the goal is the one place that black has a hole in it. Find the
    row band where the frame is mostly black (wherever the wall sits in THIS
    frame - near or far, it doesn't matter), then take the widest run of
    not-black pixels inside that band. Only a run with black frame on BOTH sides
    of it counts: a gap that runs off an edge of the picture might just be the
    wall running out of frame (we are close, or off to one side), not an open
    goal, so that reads as "not seen" rather than risking a bad guess."""
    import numpy as np
    mask = frame_mask(frame) > 0
    h, w = mask.shape
    wall_rows = np.where(mask.mean(axis=1) > TUNE["goal_wall_frac"])[0]
    if len(wall_rows) == 0:
        return None
    is_black = mask[wall_rows[0]:wall_rows[-1] + 1].mean(axis=0) > 0.5
    best = None
    x = 0
    while x < w:
        if is_black[x]:
            x += 1
            continue
        start = x
        while x < w and not is_black[x]:
            x += 1
        if start > 0 and x < w and x - start >= TUNE["goal_min_gap"]:
            if best is None or x - start > best[1] - best[0]:
                best = (start, x)
    if best is None:
        return None
    start, end = best
    return ((start + end) / 2.0 / (w / 2.0) - 1.0, end - start)


# ---------------------------------------------------------------- streaming
PAGE = b"""<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>ballbot</title>
<style>body{margin:0;background:#111;color:#bbb;font:13px system-ui;text-align:center}
img{width:640px;max-width:100%;image-rendering:pixelated;background:#000}</style>
<p>magenta circle = the ball it found &middot; green arrow = where it is driving
<br><img src="/stream.mjpg">
"""


def lan_ip():
    """The address a laptop on the same network would use to reach this Pi."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))   # a UDP connect sends nothing, it just picks a route
        return s.getsockname()[0]
    except Exception:
        return "<pi-address>"
    finally:
        s.close()


def annotate(frame, ball, drive, status):
    """Draw what the robot decided on top of what it saw.

    The point of watching over the network is to see the DECISION, not the picture -
    a raw feed tells you nothing about why it drove past the ball."""
    import cv2
    f = frame.copy()
    hgt, wid = f.shape[:2]
    if ball is not None:
        dx, r = ball
        cx = int((dx + 1.0) * wid / 2.0)
        # find_ball keeps only dx and the radius and throws the height away, so the
        # circle has to be parked on the middle line - its x and size are measured,
        # its height is not. The full-height line is the part that is actually true,
        # and it is what you compare against the arrow to see if the robot is aiming.
        cv2.line(f, (cx, 0), (cx, hgt), (255, 0, 255), 1)
        cv2.circle(f, (cx, hgt // 2), max(3, int(r)), (255, 0, 255), 2)
    if drive is not None:
        vx, vy, turn = drive
        ox, oy = wid // 2, hgt - 10
        # vy is +left and screen x grows rightward, so the arrow's x is negated.
        cv2.arrowedLine(f, (ox, oy), (ox - int(vy * 55), oy - int(vx * 55)),
                        (0, 255, 0), 2, tipLength=0.3)
        if abs(turn) > 0.05:
            # A spin has no direction to point on screen, so it has to be written out.
            cv2.putText(f, "turn %+.2f" % turn, (ox + 34, oy - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    if status:
        cv2.rectangle(f, (0, 0), (wid, 13 * len(status) + 4), (0, 0, 0), -1)
        for i, line in enumerate(status):
            cv2.putText(f, line, (4, 13 * i + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                        (255, 255, 255), 1)
    return f


class Streamer:
    """MJPEG over HTTP, so you can watch from a laptop while the robot runs itself.

    RULE 2.2 BANS WIFI IN A MATCH. This is a bench and practice tool: --stream turns
    it on, and nothing starts unless you ask.

    The match loop must never wait on a browser, so offer() drops the newest frame
    into a slot and returns; the HTTP threads do all the waiting. With nobody
    connected it does not even encode - no viewer, no cost.
    """

    BOUNDARY = b"ballbotframe"

    def __init__(self, port=8000, quality=55, fps=10, page=PAGE, routes=None,
                 label="stream", fatal=False):
        self.port, self.quality, self.min_dt = port, quality, 1.0 / fps
        self.jpeg, self.seq, self.viewers, self.sent_at = None, 0, 0, 0.0
        self.cond = threading.Condition()
        self.server = None
        # page and routes are what let a second tool reuse this: calibrate.py serves
        # its own controls here and gets the mjpeg plumbing for free. A route is
        # fn(query_dict) -> (content_type, bytes), called on an http thread.
        self.page, self.routes = page, routes or {}
        self.label, self.fatal = label, fatal

    def start(self):
        from http.server import ThreadingHTTPServer
        try:
            self.server = ThreadingHTTPServer(("", self.port), _stream_handler(self))
        except OSError as e:
            # A stale process still holding the port is the usual cause. For bot.py
            # losing the video is no reason to refuse to drive, so it carries on;
            # for a tool whose whole interface IS the page, carrying on is useless.
            msg = "%s: port %d unavailable (%s)" % (self.label, self.port, e)
            if self.fatal:
                raise SystemExit(msg + "\n  free it, or pass a different port")
            print(msg + " - running without it")
            return self
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        print("%s: http://%s:%d/   (rule 2.2 bans wifi in a match - bench only)"
              % (self.label, lan_ip(), self.port))
        return self

    def wants_frame(self):
        """Cheap enough to call every pass of the loop, so the caller can skip
        building status strings for a stream nobody is watching."""
        return (self.server is not None and self.viewers > 0
                and time.time() - self.sent_at >= self.min_dt)

    def offer(self, frame, ball=None, drive=None, status=None):
        if frame is None or not self.wants_frame():
            return
        self.sent_at = time.time()
        try:
            import cv2
            ok, buf = cv2.imencode(".jpg", annotate(frame, ball, drive, status or []),
                                   [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        except Exception as e:
            print("stream: encode failed (%s)" % e)
            return
        if not ok:
            return
        with self.cond:
            self.jpeg, self.seq = buf.tobytes(), self.seq + 1
            self.cond.notify_all()

    def wait_frame(self, seen, timeout=4.0):
        """Block until there is a frame newer than `seen` -> (jpeg, seq), or None."""
        with self.cond:
            if self.seq == seen:
                self.cond.wait(timeout)
            return None if self.seq == seen else (self.jpeg, self.seq)

    def _enter(self):
        with self.cond:
            self.viewers += 1

    def _leave(self):
        with self.cond:
            self.viewers -= 1

    def close(self):
        if self.server:
            self.server.shutdown()


def _stream_handler(streamer):
    from http.server import BaseHTTPRequestHandler

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, *a):
            pass            # the console is for the match, not for access logs

        def do_GET(self):
            from urllib.parse import urlparse, parse_qs
            u = urlparse(self.path)
            if u.path.startswith("/stream"):
                return self.mjpeg()
            route = streamer.routes.get(u.path)
            if route is not None:
                try:
                    ctype, body = route(parse_qs(u.query))
                except Exception as e:
                    # A control that throws must not kill the tuner - report it to
                    # the browser and leave the camera loop running.
                    return self.send_error(500, "%s: %s" % (type(e).__name__, e))
            else:
                ctype, body = "text/html; charset=utf-8", streamer.page
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def mjpeg(self):
            self.send_response(200)
            self.send_header("Cache-Control", "no-store, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=%s"
                             % streamer.BOUNDARY.decode())
            self.end_headers()
            streamer._enter()
            seen = 0
            try:
                while True:
                    fresh = streamer.wait_frame(seen)
                    if fresh is None:
                        # Nothing new for seconds: the loop is between segments or
                        # stopped. Re-send the last frame rather than sitting mute -
                        # it holds the tab open, and the write is also how we find out
                        # the laptop has gone, so we stop encoding for an empty room.
                        if streamer.jpeg is None:
                            continue
                        jpeg = streamer.jpeg
                    else:
                        jpeg, seen = fresh
                    self.wfile.write(b"--" + streamer.BOUNDARY + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass        # tab closed; normal
            finally:
                streamer._leave()

    return Handler


# ---------------------------------------------------------------- main
def play(bot, grab, goal_heading, t_end, stream=None, find=find_ball, label="colour"):
    import numpy as np
    st, stuck_since, prev = {"spin": 1}, time.time(), None
    while time.time() < t_end:
        # A toggle switch left in the ON position reads as "pressed" forever, which
        # would break out on frame one - so --nobutton ignores the stop check too.
        if bot.button and not NOBUTTON and bot.button.is_pressed:   # second press = stop
            break
        frame = grab()
        st["blind"] = st.get("blind", 0) + 1 if is_blind(frame) else 0
        blind = st["blind"] > TUNE["blind_frames"]
        # A dropped frame reads as None. Don't hand that to OpenCV - it throws, and a
        # crash mid-match costs far more than a skipped frame.
        ball = None if (blind or frame is None) else find(frame)
        goal = None if (blind or frame is None) else find_goal(frame)
        h = bot.heading()
        herr = None if (h is None or goal_heading is None) else -angle_diff(goal_heading, h)
        vx, vy, w = decide(ball, herr, st, blind, goal)
        if stream is not None and stream.wants_frame():
            stream.offer(frame, ball, (vx, vy, w), [
                "%4.0fs left   heading %s   goal err %s" % (
                    max(0.0, t_end - time.time()),
                    "--" if h is None else "%.0f" % h,
                    "--" if herr is None else "%+.0f" % herr),
                "ball (%s) %s   %s" % (
                    label, "none" if ball is None else "dx%+.2f r%.0f" % ball,
                    "BLIND" if blind else ""),
                "goal %s" % ("none" if goal is None else "dx%+.2f gap%.0f" % goal),
                "vx%+.2f vy%+.2f w%+.2f" % (vx, vy, w),
            ])

        now = time.time()
        small = None if frame is None else frame[::8, ::8].astype(np.int16)
        still = (small is not None and prev is not None and prev.shape == small.shape
                 and np.abs(small - prev).mean() < TUNE["motion_min"])
        prev = small
        if not should_escape(still, blind, max(abs(vx), abs(vy), abs(w)) > 0.1,
                             now - stuck_since):
            stuck_since = now
        else:                                        # wedged: thrash out and look again
            bot.drive(-0.8, 0.5, 0.6)
            while time.time() - now < 0.6:
                bot.heading()                        # keep the gyro integrating meanwhile
                time.sleep(0.02)
            stuck_since = time.time()
            continue
        bot.drive(vx, vy, w)
        time.sleep(0.02)
    bot.stop()


def selftest():
    """python3 bot.py --check - proves the whole stack works ON THIS BOARD.

    "Illegal instruction" means a prebuilt wheel was compiled for a different CPU
    than yours. pip serves one wheel to every ARM board; apt builds for the exact
    architecture of your OS image. So the fix is always the same: get numpy and
    OpenCV from apt, and check below that they really came from there.
    """
    import platform, numpy, cv2
    arch = platform.machine()
    on_arm = arch.startswith(("arm", "aarch"))
    print("machine     :", arch, " (aarch64 = 64-bit Pi OS, armv7l = 32-bit)")
    for mod in (numpy, cv2):
        apt = "/usr/lib/" in mod.__file__
        note = "apt, good" if apt else (
            "PIP - the usual cause of Illegal instruction on a Pi" if on_arm
            else "pip, fine off-Pi")
        print("%-12s: %-9s %s  <- %s" % (mod.__name__, mod.__version__, mod.__file__, note))
    # numpy usually dies in BLAS rather than on import, so actually do some maths
    print("numpy maths :", numpy.zeros((32, 32)).dot(numpy.ones((32, 32))).sum(), "(want 0.0)")
    bot = Robot()
    print("heading     :", bot.heading())
    frame = open_camera()()
    print("camera      :", "NO FRAME" if frame is None else
          "%s contrast %.1f%s" % (frame.shape, frame.std(),
                                  "  BLIND - lens covered?" if is_blind(frame) else ""))
    if frame is not None:
        print("ball        :", find_ball(frame), " (aim the camera at the ball)")
        if OBJECT:
            try:
                det = ObjectDetector(OBJECT_MODEL or TUNE["obj_model"])
                t = time.time()
                found = det.detect(frame)
                print("object      :", found, " (%.0f ms)" % ((time.time() - t) * 1000))
            except Exception as e:
                print("object      : FAILED - %s: %s" % (type(e).__name__, e))
    bot.stop()


def wheeltest():
    """python3 bot.py --wheels - robot UP ON A BOX, wheels hanging free.

    Watch each move and fix what's wrong before you tune anything else. Every later
    problem looks like bad driving if a wheel is backwards."""
    bot = Robot()
    for i, name in enumerate(("FL", "FR", "BL", "BR")):
        print("wheel %s alone - the top of it should roll FORWARD" % name, flush=True)
        for j, m in enumerate(bot.motors or []):
            m.value = 0.4 if i == j else 0.0
        time.sleep(1.5)
        bot.stop()
        time.sleep(0.7)
    moves = [("FORWARD", 1, 0, 0), ("BACKWARD", -1, 0, 0),
             ("STRAFE LEFT  (sideways, nose stays put)", 0, 1, 0),
             ("STRAFE RIGHT", 0, -1, 0),
             ("SPIN LEFT  (counter-clockwise)", 0, 0, 1), ("SPIN RIGHT", 0, 0, -1)]
    for label, vx, vy, w in moves:
        print(label, flush=True)
        s = TUNE["speed"]
        bot.drive(vx * s, vy * s, w * s)
        time.sleep(2)
        bot.stop()
        time.sleep(1)
    print("\nONE wheel spinning the wrong way -> swap that motor's two wires at the")
    print("driver board. Do NOT fix it in code, or strafing will go diagonal.")
    print("All four right but sideways is backwards -> tune.json  invert_strafe: -1")
    print("All four right but spin is backwards     -> tune.json  invert_turn: -1")


def main():
    if "--check" in sys.argv:
        return selftest()
    if "--wheels" in sys.argv:
        return wheeltest()
    bot = Robot()
    grab = open_camera()
    find, label = ball_finder()   # before the countdown: loading a model takes seconds
    stream = Streamer(STREAM_PORT).start() if STREAM else None
    left = 12 * 60
    try:
        while left > 0:
            if bot.button and not NOBUTTON:
                print("point the robot at the ENEMY goal, then press start")
                bot.button.wait_for_press(); bot.button.wait_for_release()
            else:
                print("point the robot at the ENEMY goal and stand clear"
                      " (no button: Ctrl-C stops it)")
                for n in range(START_COUNTDOWN, 0, -1):
                    print("  %d..." % n, flush=True)
                    time.sleep(1)
            bot.zero_heading()      # robot is placed and still now; boot time was not
            goal_heading = bot.heading()
            print("go. %.0f s left, goal heading = %s" % (left, goal_heading))
            t0 = time.time()
            play(bot, grab, goal_heading, t0 + (5 if DRY else left), stream, find, label)
            left -= time.time() - t0   # 6.3: a goal stops the clock, it doesn't reset it
            print("stopped, %.0f s left" % max(left, 0))
            if DRY:
                return
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop()
        if stream:
            stream.close()


if __name__ == "__main__":
    main()
