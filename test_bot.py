#!/usr/bin/env python3
"""Self-check for the driving maths and the strategy. Run: python3 test_bot.py"""
import sys
sys.argv.append("--dry")
from bot import mix, angle_diff, decide, gyro_step, should_escape, Gyro, TUNE


def approx(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_mix():
    assert approx(mix(1, 0, 0), (1, 1, 1, 1)), "forward = all wheels forward"
    assert approx(mix(-1, 0, 0), (-1, -1, -1, -1))
    assert approx(mix(0, 1, 0), (-1, 1, 1, -1)), "left strafe = X pattern"
    assert approx(mix(0, 0, 1), (-1, 1, -1, 1)), "CCW = left side back, right side fwd"
    assert max(abs(v) for v in mix(1, 1, 1)) <= 1.0, "must stay inside motor range"
    assert approx(mix(0.5, 0, 0), (0.5, 0.5, 0.5, 0.5)), "small commands are not normalised up"


def test_angle_diff():
    assert angle_diff(10, 350) == 20, "wraps across north"
    assert angle_diff(350, 10) == -20
    assert angle_diff(0, 180) == -180, "exact opposite resolves one way, not both"
    assert angle_diff(90, 90) == 0


def test_decide():
    close, far = TUNE["close_radius"] + 5, TUNE["close_radius"] - 15

    vx, vy, w = decide(None, 0, {"spin": 1})
    assert w > 0 and vx < 0, "no ball: spin and ease back"

    vx, vy, w = decide((0.0, close), 0.0, {})
    assert vx > 0.5, "on the ball and aimed: drive through it"

    vx, vy, w = decide((0.0, close), 60.0, {})
    assert vy < 0 and w > 0, "goal is to our left: slide right around the ball"
    assert decide((0.0, close), -60.0, {})[1] > 0, "mirrored for the other side"

    vx, vy, w = decide((0.8, far), None, {})
    assert vy < 0 and vx > 0, "ball far right: drive and strafe right"

    st = {}
    decide((0.5, far), None, st)
    assert st["spin"] == -1, "remembers which way the ball went for the next search"


def test_ball_memory():
    close, far = TUNE["close_radius"] + 5, TUNE["close_radius"] - 15

    st = {}
    decide((0.0, close), 0.0, st)                # had it right on the nose...
    assert decide(None, 0.0, st)[0] > 0, "...then it vanished under the plow: push on"
    for _ in range(TUNE["ball_memory"] + 1):
        out = decide(None, 0.0, st)
    assert out[0] < 0, "but give up eventually instead of driving blind forever"

    st = {}
    decide((0.0, far), 0.0, st)                  # only ever saw it far away
    assert decide(None, 0.0, st)[0] < 0, "a ball lost at distance is really lost"


def test_blind():
    st = {}
    vx, vy, w = decide(None, 90.0, st, blind=True)
    assert vx > 0 and w > 0, "camera dead: still drive on the compass toward the goal"
    ys = [decide(None, 0.0, st, blind=True)[1] for _ in range(90)]
    assert min(ys) < 0 < max(ys), "and sweep both ways instead of driving one line"
    assert len(set(ys)) == 2, "sweep is a slow flip, not per-frame jitter"


def test_should_escape():
    long_enough = TUNE["stuck_secs"] + 1
    assert should_escape(True, False, True, long_enough), "frozen while driving = wedged"
    assert not should_escape(False, False, True, long_enough), "the view is still changing"
    assert not should_escape(True, False, True, TUNE["stuck_secs"] - 0.1), "too soon"
    assert not should_escape(True, False, False, long_enough), "we asked it to sit still"
    # the regression that matters: a good push holds the ball's size steady for
    # seconds, and the old radius-based check reversed away from the ball right then
    assert not should_escape(True, True, True, long_enough), "a blind camera can't judge motion"


def test_gyro():
    # turning counter-clockwise must make the clockwise compass heading go DOWN
    assert gyro_step(10.0, 5.0, 1.0) == 5.0
    assert gyro_step(2.0, 5.0, 1.0) == 357.0, "wraps below zero"
    assert gyro_step(10.0, 5.0, 1.0, sign=-1) == 15.0, "imu_sign flips a bad mounting"
    assert gyro_step(90.0, 0.0, 10.0) == 90.0, "sitting still never drifts the maths"

    import time as _t

    class FakeBus:  # a gyro that reads a steady +2 deg/s of real rotation off zero bias
        born, first_read = _t.time(), None

        def read_i2c_block_data(self, addr, reg, n):
            if self.first_read is None:
                FakeBus.first_read = _t.time() - self.born
            return [1, 6]  # 262 raw = 2.0 deg/s at 131 LSB per deg/s

    g = Gyro(bus=FakeBus())
    assert abs(g.bias - 262.0) < 1e-6, "bias calibration soaks up a constant offset"
    # The settle has to come BEFORE the first read, or it is decoration: the whole
    # point is that no sample is taken while the button shove is still ringing.
    assert g.bus.first_read >= TUNE["gyro_settle"],         "sampled %.2fs after zero() began, settle is %.2fs" % (
            g.bus.first_read, TUNE["gyro_settle"])
    g.t -= 1.0
    assert abs(g.heading() - 0.0) < 1e-6, "so the same reading now means 'not turning'"




def test_find_ball_ignores_a_same_coloured_rival():
    """The venue is white tile and the ball is green, so a rival in the ball's own
    green is the case to survive. It is always BIGGER than the ball, so picking the
    largest blob of ball colour drives us into it. Only the round one counts.
    (Colours here are the medians measured off the real ball photos.)"""
    import cv2, numpy as np
    from bot import find_ball
    BALL = (18, 200, 139)                       # BGR of HSV(40,232,200), the measured ball

    def frame_with(*shapes):
        f = np.full((240, 320, 3), 205, np.uint8)         # bare white tile
        cv2.line(f, (60, 0), (60, 240), (120, 120, 120), 3)   # a grout line to ignore
        for s in shapes:
            s(f)
        return f

    ball_left = lambda f: cv2.circle(f, (80, 120), 20, BALL, -1)
    ball_right = lambda f: cv2.circle(f, (240, 120), 20, BALL, -1)
    rival = lambda f: cv2.rectangle(f, (180, 60), (300, 180), BALL, -1)   # far bigger

    dx, r = find_ball(frame_with(ball_left))
    assert dx < -0.4 and 15 < r < 25, "a lone ball is still found: %s" % ((dx, r),)

    assert find_ball(frame_with(rival)) is None, "a same-coloured chassis is not a ball"

    dx, _r = find_ball(frame_with(ball_left, rival))
    assert dx < -0.4, "picked the bigger rival over the ball, dx=%.2f" % dx

    # Two balls, unequal: still the largest ROUND one, so distance logic is intact.
    big = lambda f: cv2.circle(f, (240, 120), 34, BALL, -1)
    dx, r = find_ball(frame_with(ball_left, big))
    assert dx > 0.4 and r > 30, "should prefer the nearer/bigger ball: %s" % ((dx, r),)


def test_pick_box():
    """--object mode's box -> (dx, radius), the same shape decide() gets from colour."""
    from bot import pick_box
    ball = (140, 100, 180, 140, 0.6, "ball")             # centred at x=160, 40 px wide
    rival = (0, 0, 120, 120, 0.9, "robot")

    dx, r = pick_box([ball], 320)
    assert abs(dx) < 1e-6 and r == 20, "centre of a 320-wide frame is dx 0: %s" % ((dx, r),)
    assert pick_box([], 320) is None
    assert pick_box([ball, rival], 320, "ball")[0] == 0, "class filter keeps the rival out"
    assert pick_box([rival], 320, ["ball"]) is None, "a list of classes works too"
    assert pick_box([ball, rival], 320)[1] == 60, "no filter: most confident box wins"
    # a ball cut off at the bottom edge: box is short, but still as wide as the ball
    assert pick_box([(140, 200, 180, 240, 0.8, "ball"), ], 320)[1] == 20
    assert pick_box([(140, 220, 180, 240, 0.8, "ball")], 320)[1] == 20, "longer side, not shorter"


def test_async_detector_goes_stale():
    """A slow model must not freeze the loop, and an old answer must not steer."""
    import time as _t
    from bot import AsyncDetector

    class Slow:
        def detect(self, frame):
            _t.sleep(0.05)
            return (0.5, 10)

    a = AsyncDetector(Slow()).start()
    t = _t.time()
    assert a.see("frame") is None, "nothing detected yet"
    assert _t.time() - t < 0.02, "see() must not wait on the model"
    _t.sleep(0.15)
    assert a.see("frame") == (0.5, 10), "picks up the worker's answer"
    a.ball_t -= TUNE["obj_max_age"] + 1
    assert a.see("frame") is None, "a detection older than obj_max_age is dropped"


for fn in (test_mix, test_angle_diff, test_decide, test_ball_memory, test_blind, test_should_escape, test_gyro,
           test_find_ball_ignores_a_same_coloured_rival, test_pick_box,
           test_async_detector_goes_stale):
    fn()
    print("ok", fn.__name__)
print("all good")
