#!/usr/bin/env python3
"""Self-check for the driving maths and the strategy. Run: python3 test_bot.py"""
import sys
sys.argv.append("--dry")
from bot import mix, angle_diff, decide, TUNE


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

    vx, vy, w, kick = decide(None, 0, None, {"spin": 1})
    assert not kick and w > 0 and vx < 0, "no ball: spin and ease back"

    vx, vy, w, kick = decide((0.0, close), 0.0, None, {})
    assert kick and vx > 0.5, "on the ball and aimed: charge and kick"

    vx, vy, w, kick = decide((0.0, close), 60.0, None, {})
    assert not kick and vy < 0 and w > 0, "goal is to our left: slide right around the ball"
    assert decide((0.0, close), -60.0, None, {})[1] > 0, "mirrored for the other side"

    vx, vy, w, kick = decide((0.8, far), None, None, {})
    assert vy < 0 and vx > 0 and not kick, "ball far right: drive and strafe right"

    vx, vy, w, kick = decide((0.0, far), 0.0, TUNE["wall_cm"] - 1, {"spin": 1})
    assert vx < 0, "wall ahead without the ball: back off (rule 10.1)"

    st = {}
    decide((0.5, far), None, None, st)
    assert st["spin"] == -1, "remembers which way the ball went for the next search"


def test_blind():
    st = {}
    vx, vy, w, kick = decide(None, 90.0, None, st, blind=True)
    assert vx > 0 and w > 0, "camera dead: still drive on the compass toward the goal"
    ys = [decide(None, 0.0, None, st, blind=True)[1] for _ in range(90)]
    assert min(ys) < 0 < max(ys), "and sweep both ways instead of driving one line"
    assert len(set(ys)) == 2, "sweep is a slow flip, not per-frame jitter"


for fn in (test_mix, test_angle_diff, test_decide, test_blind):
    fn()
    print("ok", fn.__name__)
print("all good")
