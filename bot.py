#!/usr/bin/env python3
"""Ball-touch robot: 100% autonomous, Raspberry Pi 4 + 4x mecanum + top camera.

  python3 bot.py         run for real (press the start button once)
  python3 bot.py --check  prove numpy/cv2/camera/IMU work on this board
  python3 bot.py --wheels bring-up: wheel directions, WHEELS OFF THE GROUND
  python3 bot.py --dry    no motors, prints what it would do (test on a laptop)
  python3 bot.py --nobutton  no start button fitted: count down and go
  python3 calibrate.py   tune the ball colour at the venue -> tune.json

Rules this code is built around:
  2.2  autonomous only, one button press, no remote / wifi
  5.x  touching and pushing are free; TRAPPING the ball forces the wheels to
       stop for <=5 s, so we never trap - the front stays open on purpose
  10.1 pinning another robot on the wall is a foul -> back off when close
  12.3 don't park in front of your own goal -> the retreat is capped
"""
import json, math, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
DRY = "--dry" in sys.argv
# Rule 2.2 wants a real momentary button on the robot and you fail inspection without
# one. This is for bench runs before it's fitted: it counts down instead of waiting.
# A toggle switch on GPIO 4 does NOT need this - flip it on then off and the normal
# press/release path starts you; leaving it on is what this flag is here to survive.
NOBUTTON = "--nobutton" in sys.argv
START_COUNTDOWN = 5   # seconds, long enough to get your hands clear

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
# Only two channels wired? Use two entries and drive() falls back to tank() on its own:
# MOTORS = {"L": (19, 13, 12), "R": (6, 5, 18)}
TANK = len(MOTORS) == 2      # drop to differential drive when only two channels exist
BUTTON_PIN = 4
KICKER_PIN = None      # no kicker fitted - we push the ball in. Set to a GPIO if you add one.
SONAR_PINS = (17, 22)  # HC-SR04 (echo, trigger); set None if you didn't fit one

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
    "close_radius": 34,      # ball radius in px that means "we are on it"
    "speed": 0.75,           # base power 0..1
    "strafe_gain": 1.2,      # how hard we slide sideways onto the ball
    "turn_gain": 1.4,        # how hard we rotate onto the goal heading
    "search_spin": 0.45,
    "shoot_deg": 14,         # heading error we accept before charging the goal
    "wall_cm": 14,
    "invert_strafe": 1,      # flip to -1 if it strafes the wrong way
    "invert_turn": 1,        # flip to -1 if it spins the wrong way
    "min_duty": 0.22,        # below this a motor just buzzes
    "kick_cooldown": 2.2,    # rule 5.4: no re-grab within 2 s of a throw
    "blind_std": 12,         # frame contrast below this = lens covered
    "blind_frames": 15,      # ~0.5 s of that before we believe it
    "imu_sign": 1,           # flip to -1 if the MPU-6050 is mounted upside down
    "gyro_settle": 0.3,      # s to let the button shove die out before sampling bias
    "ball_memory": 25,       # frames to keep pushing after the plow hides the ball
    "motion_min": 2.5,       # frame-to-frame pixel change below this = we aren't moving
    "stuck_secs": 2.5,       # how long that has to hold before we thrash free
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


def tank(vx, w):
    """Differential mixer for two ganged channels -> (left, right).

    There is no vy: two channels cannot strafe, so the caller drops it. Turning is
    the only way this chassis corrects a sideways ball error, which is why decide()'s
    strafe term has to be folded into the turn when TANK is set."""
    l, r = vx - w, vx + w
    m = max(1.0, abs(l), abs(r))
    return l / m, r / m


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


def decide(ball, herr, front_cm, st, blind=False):
    """ball = (dx in -1..1, radius_px) or None.
    herr = degrees we must turn counter-clockwise to face the enemy goal (None = no IMU).
    blind = the camera can't see anything (covered, knocked, or blown out).
    Returns (vx, vy, w, kick)."""
    T = TUNE
    st["n"] = st.get("n", 0) + 1
    aim = 0.0 if herr is None else clamp(herr * T["turn_gain"] / 90.0)
    if blind:
        # Can't find the ball, but the compass still knows where the goal is: push
        # that way and sweep, so a dead camera costs us the match instead of the game.
        # Blind is exactly when we can't see a wall coming, so the sonar rules here.
        if front_cm is not None and front_cm < T["wall_cm"]:
            return (-T["speed"] * 0.5, 0.0, aim, False)
        sweep = 1.0 if (st["n"] // 30) % 2 == 0 else -1.0
        return (T["speed"] * 0.8, sweep * 0.35, aim, True)
    if ball is None:
        # At point-blank range the plow and the camera's own blind spot swallow the
        # ball. Losing it while it was CLOSE means it's on our nose, not gone - back
        # off now and you shove it away and chase it forever. Keep pushing briefly.
        if (st.get("seen_r", 0) >= T["close_radius"]
                and st["n"] - st.get("seen_n", -999) < T["ball_memory"]):
            return (T["speed"], 0.0, aim, True)
        return (-0.12, 0.0, st.get("spin", 1) * T["search_spin"], False)
    dx, r = ball
    st["spin"] = 1 if dx < 0 else -1
    st["seen_n"], st["seen_r"] = st["n"], r
    if front_cm is not None and front_cm < T["wall_cm"] and r < T["close_radius"]:
        return (-T["speed"], 0.0, st.get("spin", 1) * 0.3, False)  # 10.1: peel off the wall
    if r >= T["close_radius"]:
        if herr is None or abs(herr) < T["shoot_deg"]:
            return (T["speed"], -dx * 0.35, aim, True)             # lined up: drive through it
        s = 1.0 if herr > 0 else -1.0
        return (0.15, -s * T["speed"] * 0.9, aim, False)           # orbit around the ball
    return (T["speed"] * (1 - 0.45 * abs(dx)), -dx * T["strafe_gain"], aim * 0.5, False)


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
        self.motors = self.button = self.kicker = self.sonar = None
        self.heading_fn = self.gyro = None
        if DRY:
            return
        from gpiozero import Motor, Button, OutputDevice
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
        if KICKER_PIN is not None:
            self.kicker = OutputDevice(KICKER_PIN)
        if SONAR_PINS is not None:
            from gpiozero import DistanceSensor
            self.sonar = DistanceSensor(echo=SONAR_PINS[0], trigger=SONAR_PINS[1], max_distance=1.0)
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
        if TANK:
            # Two ganged channels cannot strafe. Rather than throw the sideways
            # command away and creep past the ball, spend it as extra turn - the only
            # authority this chassis has over a left/right error.
            w = clamp(w + vy * TUNE["strafe_gain"] * 0.5)
            vals = tank(vx, w)
        else:
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

    def front_cm(self):
        return None if self.sonar is None else self.sonar.distance * 100

    def kick(self):
        if self.kicker:
            self.kicker.on(); time.sleep(0.06); self.kicker.off()


# ---------------------------------------------------------------- vision
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
        return lambda: cap.read()[1]


def is_blind(frame):
    """Anything pressed against a lens defocuses to a flat blur, so a covered
    camera has almost no contrast. Also catches a dead camera or a dark frame."""
    return frame is None or frame.std() < TUNE["blind_std"]


def find_ball(frame):
    """-> (dx in -1..1, radius_px) or None."""
    import cv2, numpy as np
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(TUNE["hsv_lo"]), np.array(TUNE["hsv_hi"]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
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


# ---------------------------------------------------------------- main
def play(bot, grab, goal_heading, t_end):
    import numpy as np
    st, last_kick, stuck_since, prev = {"spin": 1}, 0.0, time.time(), None
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
        ball = None if (blind or frame is None) else find_ball(frame)
        h = bot.heading()
        herr = None if (h is None or goal_heading is None) else -angle_diff(goal_heading, h)
        vx, vy, w, kick = decide(ball, herr, bot.front_cm(), st, blind)

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
        if kick and now - last_kick > TUNE["kick_cooldown"]:
            bot.kick(); last_kick = now
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
    print("heading     :", bot.heading(), " front_cm:", bot.front_cm())
    frame = open_camera()()
    print("camera      :", "NO FRAME" if frame is None else
          "%s contrast %.1f%s" % (frame.shape, frame.std(),
                                  "  BLIND - lens covered?" if is_blind(frame) else ""))
    if frame is not None:
        print("ball        :", find_ball(frame), " (aim the camera at the ball)")
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
    left = 12 * 60
    try:
        while left > 0:
            if bot.button and not NOBUTTON:
                print("point the robot at the ENEMY goal, then press start")
                bot.button.wait_for_press(); bot.button.wait_for_release()
            else:
                print("point the robot at the ENEMY goal and stand clear")
                for n in range(START_COUNTDOWN, 0, -1):
                    print("  %d..." % n, flush=True)
                    time.sleep(1)
            bot.zero_heading()      # robot is placed and still now; boot time was not
            goal_heading = bot.heading()
            print("go. %.0f s left, goal heading = %s" % (left, goal_heading))
            t0 = time.time()
            play(bot, grab, goal_heading, t0 + (5 if DRY else left))
            left -= time.time() - t0   # 6.3: a goal stops the clock, it doesn't reset it
            print("stopped, %.0f s left" % max(left, 0))
            if DRY:
                return
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop()


if __name__ == "__main__":
    main()
