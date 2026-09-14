#!/usr/bin/env python3
"""Ball colour tuner. Run it on the real field, under the real lights.

Drag the sliders until only the ball is white in the mask window, press 's'.
Writes tune.json, which bot.py reads at startup. 'q' quits without saving.
"""
import json, os, cv2
import bot

W = "tune (s=save, q=quit)"
cv2.namedWindow(W)
for i, name in enumerate(("H lo", "S lo", "V lo")):
    cv2.createTrackbar(name, W, bot.TUNE["hsv_lo"][i], 255, lambda _: None)
for i, name in enumerate(("H hi", "S hi", "V hi")):
    cv2.createTrackbar(name, W, bot.TUNE["hsv_hi"][i], 255, lambda _: None)
# Roundness is what keeps us off a rival wearing the ball's colour. Point the camera
# at the RIVAL and raise this until the circle stops locking on to it.
cv2.createTrackbar("round%", W, int(bot.TUNE["min_round"] * 100), 100, lambda _: None)

grab = bot.open_camera()
while True:
    frame = grab()
    if frame is None:
        continue
    bot.TUNE["hsv_lo"] = [cv2.getTrackbarPos(n, W) for n in ("H lo", "S lo", "V lo")]
    bot.TUNE["hsv_hi"] = [cv2.getTrackbarPos(n, W) for n in ("H hi", "S hi", "V hi")]
    bot.TUNE["min_round"] = cv2.getTrackbarPos("round%", W) / 100.0
    cv2.imshow(W, bot.draw_ball(frame, bot.find_ball(frame)))
    k = cv2.waitKey(30) & 0xFF
    if k == ord("s"):
        path = os.path.join(bot.HERE, "tune.json")
        json.dump(bot.TUNE, open(path, "w"), indent=2)
        print("saved", path, bot.TUNE["hsv_lo"], bot.TUNE["hsv_hi"])
    elif k == ord("q"):
        break
cv2.destroyAllWindows()
