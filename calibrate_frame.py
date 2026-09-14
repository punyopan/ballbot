#!/usr/bin/env python3
"""Black frame tuner: the walls, goal posts and crossbar. Run it on the real field.

Drag the sliders until the frame is solid white in the mask window (walls, goal
posts, crossbar) while the floor, the ball and the shadows under robots stay
black, press 's'. Writes tune.json, which bot.py reads at startup. 'q' quits
without saving.

This is the first half of goal detection: colour alone can't tell a wall from a
goal post, they're the same black - find_goal() (in bot.py) finds the goal by
looking for the one place that black has a hole in it, and it needs a clean
mask of the frame to work from.
"""
import json, os, cv2
import bot

W = "frame tune (s=save, q=quit)"
cv2.namedWindow(W)
for i, name in enumerate(("H lo", "S lo", "V lo")):
    cv2.createTrackbar(name, W, bot.TUNE["frame_lo"][i], 255, lambda _: None)
for i, name in enumerate(("H hi", "S hi", "V hi")):
    cv2.createTrackbar(name, W, bot.TUNE["frame_hi"][i], 255, lambda _: None)

grab = bot.open_camera()
while True:
    frame = grab()
    if frame is None:
        continue
    bot.TUNE["frame_lo"] = [cv2.getTrackbarPos(n, W) for n in ("H lo", "S lo", "V lo")]
    bot.TUNE["frame_hi"] = [cv2.getTrackbarPos(n, W) for n in ("H hi", "S hi", "V hi")]
    cv2.imshow(W, bot.frame_mask(frame))
    k = cv2.waitKey(30) & 0xFF
    if k == ord("s"):
        path = os.path.join(bot.HERE, "tune.json")
        json.dump(bot.TUNE, open(path, "w"), indent=2)
        print("saved", path, bot.TUNE["frame_lo"], bot.TUNE["frame_hi"])
    elif k == ord("q"):
        break
cv2.destroyAllWindows()
