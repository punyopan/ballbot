#!/usr/bin/env python3
"""Ball colour tuner. Run it on the real field, under the real lights.

CLICK THE BALL to eyedrop its colour - each click widens the range to include
what you clicked, so click the lit side, the shadowed side, and the edge. Drag
the sliders to fine-tune, then press 's'.

  click   sample under the cursor and widen the range to fit it
  c       clear the samples and start again
  s       write tune.json, which bot.py reads at startup
  q       quit without saving
"""
import json, os, cv2
import numpy as np
import bot

W = "tune (click ball, s=save, c=clear, q=quit)"
PATCH = 11          # px box sampled per click - big enough to beat sensor noise
H_PAD = 8           # hue drifts with the venue's white balance, so pad it
S_PAD, V_PAD = 30, 40

LAST = {"frame": None}
SAMPLE = {"lo": None, "hi": None}


def apply_sample():
    """Push the accumulated sample onto the sliders, which is what TUNE reads."""
    lo, hi = SAMPLE["lo"], SAMPLE["hi"]
    # A ball only ever reads MORE saturated and brighter than the sample - a
    # highlight, a closer approach - so open those ceilings right up and let the
    # floors do the work. Hue is the one that has to be bounded on both sides.
    for name, v in (("H lo", lo[0] - H_PAD), ("H hi", hi[0] + H_PAD),
                    ("S lo", lo[1] - S_PAD), ("S hi", 255),
                    ("V lo", lo[2] - V_PAD), ("V hi", 255)):
        top = 179 if name.startswith("H") else 255
        cv2.setTrackbarPos(name, W, int(min(top, max(0, v))))


def eyedrop(event, x, y, flags, _param):
    if event != cv2.EVENT_LBUTTONDOWN or LAST["frame"] is None:
        return
    frame = LAST["frame"]
    h, w = frame.shape[:2]
    r = PATCH // 2
    patch = frame[max(0, y - r):min(h, y + r + 1), max(0, x - r):min(w, x + r + 1)]
    if patch.size == 0:
        return
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    # Percentiles, not min/max: one specular highlight or a pixel of background
    # caught at the edge of the box would otherwise blow the range open on a
    # single click, and you would never notice until the robot chased the floor.
    lo = np.percentile(hsv, 10, axis=0)
    hi = np.percentile(hsv, 90, axis=0)
    if SAMPLE["lo"] is None:
        SAMPLE["lo"], SAMPLE["hi"] = lo, hi
    else:
        SAMPLE["lo"] = np.minimum(SAMPLE["lo"], lo)
        SAMPLE["hi"] = np.maximum(SAMPLE["hi"], hi)
    apply_sample()
    print("sampled H%d-%d S%d-%d V%d-%d" % (
        SAMPLE["lo"][0], SAMPLE["hi"][0], SAMPLE["lo"][1],
        SAMPLE["hi"][1], SAMPLE["lo"][2], SAMPLE["hi"][2]))


cv2.namedWindow(W)
cv2.setMouseCallback(W, eyedrop)
for i, name in enumerate(("H lo", "S lo", "V lo")):
    top = 179 if i == 0 else 255        # OpenCV hue is 0-179, not 0-255
    cv2.createTrackbar(name, W, min(top, bot.TUNE["hsv_lo"][i]), top, lambda _: None)
for i, name in enumerate(("H hi", "S hi", "V hi")):
    top = 179 if i == 0 else 255
    cv2.createTrackbar(name, W, min(top, bot.TUNE["hsv_hi"][i]), top, lambda _: None)
# Roundness is what keeps us off a rival wearing the ball's colour. Point the camera
# at the RIVAL and raise this until the circle stops locking on to it.
cv2.createTrackbar("round%", W, int(bot.TUNE["min_round"] * 100), 100, lambda _: None)

grab = bot.open_camera()
while True:
    frame = grab()
    if frame is None:
        continue
    LAST["frame"] = frame.copy()        # the callback samples raw pixels, not overlay
    bot.TUNE["hsv_lo"] = [cv2.getTrackbarPos(n, W) for n in ("H lo", "S lo", "V lo")]
    bot.TUNE["hsv_hi"] = [cv2.getTrackbarPos(n, W) for n in ("H hi", "S hi", "V hi")]
    bot.TUNE["min_round"] = cv2.getTrackbarPos("round%", W) / 100.0
    found = bot.find_ball(frame)
    if found:
        dx, r = found
        cx = int((dx + 1) * frame.shape[1] / 2)
        cv2.circle(frame, (cx, frame.shape[0] // 2), int(r), (255, 0, 255), 2)
        cv2.putText(frame, "dx %+.2f  r %.0f" % (dx, r), (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    else:
        cv2.putText(frame, "no ball - click it to sample", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.imshow(W, frame)
    k = cv2.waitKey(30) & 0xFF
    if k == ord("s"):
        path = os.path.join(bot.HERE, "tune.json")
        json.dump(bot.TUNE, open(path, "w"), indent=2)
        print("saved", path, bot.TUNE["hsv_lo"], bot.TUNE["hsv_hi"])
    elif k == ord("c"):
        SAMPLE["lo"] = SAMPLE["hi"] = None
        print("samples cleared - click the ball again")
    elif k == ord("q"):
        break
cv2.destroyAllWindows()
