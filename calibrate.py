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

grab = bot.open_camera()
while True:
    frame = grab()
    if frame is None:
        continue
    bot.TUNE["hsv_lo"] = [cv2.getTrackbarPos(n, W) for n in ("H lo", "S lo", "V lo")]
    bot.TUNE["hsv_hi"] = [cv2.getTrackbarPos(n, W) for n in ("H hi", "S hi", "V hi")]
    found = bot.find_ball(frame)
    if found:
        dx, r = found
        cx = int((dx + 1) * frame.shape[1] / 2)
        cv2.circle(frame, (cx, frame.shape[0] // 2), int(r), (255, 0, 255), 2)
        cv2.putText(frame, "dx %+.2f  r %.0f" % (dx, r), (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    cv2.imshow(W, frame)
    k = cv2.waitKey(30) & 0xFF
    if k == ord("s"):
        path = os.path.join(bot.HERE, "tune.json")
        json.dump(bot.TUNE, open(path, "w"), indent=2)
        print("saved", path, bot.TUNE["hsv_lo"], bot.TUNE["hsv_hi"])
    elif k == ord("q"):
        break
cv2.destroyAllWindows()
