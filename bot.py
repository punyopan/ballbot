#!/usr/bin/env python3
"""Ball-touch robot: 100% autonomous, Raspberry Pi 4 + 4x mecanum + top camera.

  python3 bot.py         run for real (press the start button once)
  python3 bot.py --dry   no motors, prints what it would do (test on a laptop)
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

# --- pins (BCM). Motor = (forward, backward, enable/pwm). Match your driver board.
MOTORS = {"FL": (5, 6, 12), "FR": (13, 19, 18), "BL": (16, 20, 21), "BR": (23, 24, 25)}
BUTTON_PIN = 4
KICKER_PIN = 27        # solenoid MOSFET gate; set None if you didn't fit one
SONAR_PINS = (17, 22)  # HC-SR04 (echo, trigger); set None if you didn't fit one

# --- everything worth changing trackside lives in tune.json
DEFAULTS = {
    "hsv_lo": [5, 120, 110], "hsv_hi": [25, 255, 255],  # orange ball on green floor
    "min_area": 60,          # px, ignore specks
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
    "ball_memory": 25,       # frames to keep pushing after the plow hides the ball
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
            from smbus2 import SMBus
            bus = SMBus(1)
            bus.write_byte_data(self.ADDR, 0x6B, 0)   # wake it up
            time.sleep(0.1)
        self.bus, self.bias = bus, 0.0
        print("calibrating gyro - keep the robot COMPLETELY still")
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
        self.motors = self.button = self.kicker = self.sonar = self.heading_fn = None
        if DRY:
            return
        from gpiozero import Motor, Button, OutputDevice
        self.motors = [Motor(f, b, enable=e, pwm=True) for f, b, e in
                       (MOTORS["FL"], MOTORS["FR"], MOTORS["BL"], MOTORS["BR"])]
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
                self.heading_fn = Gyro().heading
            except Exception as e:  # no IMU at all -> chase the ball, no goal lock
                print("no IMU (%s); heading lock off" % e)

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
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < TUNE["min_area"]:
        return None
    (x, _y), r = cv2.minEnclosingCircle(c)
    return (x / (frame.shape[1] / 2) - 1.0, r)


# ---------------------------------------------------------------- main
def play(bot, grab, goal_heading, t_end):
    st, last_kick, stuck_since, last_r = {"spin": 1}, 0.0, time.time(), 0.0
    while time.time() < t_end:
        if bot.button and bot.button.is_pressed:     # second press = stop
            break
        frame = grab()
        st["blind"] = st.get("blind", 0) + 1 if is_blind(frame) else 0
        blind = st["blind"] > TUNE["blind_frames"]
        ball = None if blind else find_ball(frame)
        h = bot.heading()
        herr = None if (h is None or goal_heading is None) else -angle_diff(goal_heading, h)
        vx, vy, w, kick = decide(ball, herr, bot.front_cm(), st, blind)

        now = time.time()
        r = ball[1] if ball else 0.0
        if ball is None or abs(r - last_r) > 3:
            stuck_since, last_r = now, r
        elif now - stuck_since > 2.5:                # wedged: shove out and look again
            bot.drive(-0.8, 0.5, 0.6)
            while time.time() - now < 0.6:
                bot.heading()                        # keep the gyro integrating while we thrash
                time.sleep(0.02)
            stuck_since = now
            continue
        if kick and now - last_kick > TUNE["kick_cooldown"]:
            bot.kick(); last_kick = now
        bot.drive(vx, vy, w)
        time.sleep(0.02)
    bot.stop()


def main():
    bot = Robot()
    grab = open_camera()
    try:
        while True:
            print("point the robot at the ENEMY goal, then press start")
            if bot.button:
                bot.button.wait_for_press(); bot.button.wait_for_release()
            goal_heading = bot.heading()
            print("go. goal heading =", goal_heading)
            play(bot, grab, goal_heading, time.time() + 12 * 60)
            print("stopped")
            if DRY:
                return
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop()


if __name__ == "__main__":
    main()
