#!/usr/bin/env python3
"""Black frame tuner: the walls, goal posts and crossbar. Run it on the real field.

  python3 calibrate_frame.py          web tuner if there is no display: http://<pi>:8002/
  python3 calibrate_frame.py --web 9000    pick the port
  python3 calibrate_frame.py --gui    force the local cv2 window (needs a screen on the Pi)

CLICK THE BLACK FRAME - a wall, a goal post, the crossbar, the lit side and the side
in shadow - and each click raises the brightness ceiling to include what you clicked.
Then read the mask: the frame should be solid white, while the white floor, the ball
and the shadows under the robots stay black. Save writes frame_lo and frame_hi into
tune.json, next to the ball's numbers.

Nothing drives on this mask yet. It is the first half of goal detection: the walls
and the goal frame are the same black, so colour alone cannot tell them apart - the
goal is the one place the black frame has an opening in it, and finding that opening
needs a clean mask of the frame first.

This is calibrate.py's engine with a different target, so the page, the sliders, the
click-to-sample and the saving all behave the same way.
"""
import cv2
import bot
import calibrate

V_PAD = 25   # the wall in shadow reads darker than the lit patch you clicked - pad for it


def widen_frame(lo, hi):
    """Black is the dark end of the scale, so the floor of the range is always 0 and
    only the brightness ceiling comes from the sample. Hue and saturation stay wide
    open: with that little light the camera's hue and saturation readings are mostly
    noise, and narrowing them to one click would drop the next patch of the same
    wall. If a dark coloured object gets in, lower S hi by hand."""
    return {"H lo": 0, "H hi": 179, "S lo": 0, "S hi": 255,
            "V lo": 0, "V hi": hi[2] + V_PAD}


def overlay_frame(frame):
    """Box the largest black region, and report how much of the picture is black."""
    mask = bot.frame_mask(frame)
    cover = 100.0 * cv2.countNonZero(mask) / mask.size
    line = "black: %.0f%% of frame" % cover
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnts:
        x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        line += "   largest region %dx%d px" % (w, h)
    return [line]


FRAME = dict(
    title="frame", lo="frame_lo", hi="frame_hi", port=8002, extra=[],
    widen=widen_frame, mask=bot.frame_mask, overlay=overlay_frame,
    hint="<b>click the black wall, goal posts and crossbar</b> to sample them. Tune "
         "until the frame is solid white in the mask, while the white floor, the ball "
         "and the shadows under robots stay black.",
)


if __name__ == "__main__":
    calibrate.main(FRAME)
