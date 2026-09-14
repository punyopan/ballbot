#!/usr/bin/env python3
"""Ball colour tuner. Run it on the real field, under the real lights.

  python3 calibrate.py           web tuner if there is no display, else the cv2 window
  python3 calibrate.py --web     force the web tuner: http://<pi>:8001/
  python3 calibrate.py --web 9000    pick the port
  python3 calibrate.py --gui     force the local cv2 window (needs a screen on the Pi)

  python3 calibrate_frame.py     the same tuner, pointed at the black walls and goal frame

CLICK THE BALL to eyedrop its colour - each click widens the range to include what
you clicked, so click the lit side, the shadowed side, and the edge. Then nudge the
sliders and save.

The web tuner is for tuning over ssh with no screen on the robot. It shows the camera
and the mask side by side, because the mask is the thing you are actually tuning: you
want the ball solid white and everything else black. Rule 2.2 bans wifi during a
match - this is a bench and practice tool, like bot.py --stream.

  cv2 window:  click = sample, c = clear samples, s = save, q = quit
  web page:    click the image = sample, and there are buttons for the rest
"""
import json, os, sys, threading, cv2
import numpy as np
import bot

PATCH = 11          # px box sampled per click - big enough to beat sensor noise
H_PAD = 8           # hue drifts with the venue's white balance, so pad it
S_PAD, V_PAD = 30, 40

LAST = {"frame": None}
SAMPLE = {"lo": None, "hi": None}
# The web controls run on http threads while the camera loop reads the same values,
# so every read and write of TUNE below goes through this.
LOCK = threading.Lock()

# What is being tuned: the ball here, the black frame in calibrate_frame.py. Everything
# below reads it, so both tuners share one engine instead of two copies drifting apart.
TARGET = {}
# (name, tune key, index or None, slider maximum, scale), built from TARGET. One list,
# so the cv2 trackbars and the web sliders cannot disagree.
CONTROLS = []


def hsv_controls(lo, hi):
    return [("H lo", lo, 0, 179, 1), ("H hi", hi, 0, 179, 1),   # OpenCV hue is 0-179
            ("S lo", lo, 1, 255, 1), ("S hi", hi, 1, 255, 1),
            ("V lo", lo, 2, 255, 1), ("V hi", hi, 2, 255, 1)]


def configure(target):
    TARGET.clear()
    TARGET.update(target)
    CONTROLS[:] = hsv_controls(target["lo"], target["hi"]) + list(target.get("extra", []))
    SAMPLE["lo"] = SAMPLE["hi"] = None


def control(name):
    for c in CONTROLS:
        if c[0] == name:
            return c
    raise KeyError(name)


def get_control(name):
    _name, key, idx, _top, scale = control(name)
    v = bot.TUNE[key]
    v = v[idx] if idx is not None else v
    return int(round(v * scale))


def set_control(name, value):
    _name, key, idx, top, scale = control(name)
    value = max(0, min(int(top), int(float(value))))
    if idx is None:
        bot.TUNE[key] = value / scale if scale != 1 else value
    else:
        v = list(bot.TUNE[key])
        v[idx] = value
        bot.TUNE[key] = v


def state():
    with LOCK:
        return {"controls": [{"name": c[0], "value": get_control(c[0]), "max": c[3]}
                             for c in CONTROLS],
                "lo": list(bot.TUNE[TARGET["lo"]]), "hi": list(bot.TUNE[TARGET["hi"]]),
                "sampled": SAMPLE["lo"] is not None}


def apply_sample():
    """Push the accumulated sample onto the controls, which is what TUNE reads.
    How a sample becomes a range is the target's call - see widen_ball."""
    for name, v in TARGET["widen"](SAMPLE["lo"], SAMPLE["hi"]).items():
        set_control(name, v)


def sample_at(x, y):
    """Eyedrop the frame at a pixel and widen the range to include it."""
    frame = LAST["frame"]
    if frame is None:
        return
    h, w = frame.shape[:2]
    x, y = int(x), int(y)
    if not (0 <= x < w and 0 <= y < h):
        return
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


def clear_samples():
    SAMPLE["lo"] = SAMPLE["hi"] = None
    print("samples cleared - click again")


def save_tune():
    path = os.path.join(bot.HERE, "tune.json")
    with LOCK:
        json.dump(bot.TUNE, open(path, "w"), indent=2)
        lo, hi = list(bot.TUNE[TARGET["lo"]]), list(bot.TUNE[TARGET["hi"]])
    print("saved", path, TARGET["lo"], lo, TARGET["hi"], hi)
    return path


# ---------------------------------------------------------------- the ball target
def widen_ball(lo, hi):
    """A ball only ever reads MORE saturated and brighter than the sample - a
    highlight, a closer approach - so open those ceilings right up and let the
    floors do the work. Hue is the one that has to be bounded on both sides."""
    return {"H lo": lo[0] - H_PAD, "H hi": hi[0] + H_PAD, "S lo": lo[1] - S_PAD,
            "S hi": 255, "V lo": lo[2] - V_PAD, "V hi": 255}


def mask_ball(frame):
    return bot.colour_mask(frame, bot.TUNE["hsv_lo"], bot.TUNE["hsv_hi"])


def draw_found(frame, found):
    if found:
        dx, r = found
        cx = int((dx + 1) * frame.shape[1] / 2)
        cv2.circle(frame, (cx, frame.shape[0] // 2), int(r), (255, 0, 255), 2)
        cv2.putText(frame, "dx %+.2f  r %.0f" % (dx, r), (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    else:
        cv2.putText(frame, "no ball - click it to sample", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return frame


def overlay_ball(frame):
    """Draw the detection onto the frame, and return the status line for the page."""
    found = bot.find_ball(frame)
    draw_found(frame, found)
    return ["ball: %s" % ("none" if not found else "dx%+.2f r%.0f" % found)]


BALL = dict(
    title="ball", lo="hsv_lo", hi="hsv_hi", port=8001,
    # Roundness is what keeps us off a rival wearing the ball's colour. Point the
    # camera at the RIVAL and raise this until the circle stops locking on to it.
    extra=[("round%", "min_round", None, 100, 100), ("min_area", "min_area", None, 2000, 1)],
    widen=widen_ball, mask=mask_ball, overlay=overlay_ball,
    hint="<b>click either one on the ball</b> to sample it. Tune until the ball is "
         "solid white in the mask and the floor is black.",
)


# ---------------------------------------------------------------- web tuner
PAGE = b"""<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>ballbot tune @@TITLE@@</title>
<style>
body{margin:0;background:#111;color:#ddd;font:13px system-ui}
.wrap{display:flex;flex-wrap:wrap;gap:16px;padding:16px;align-items:flex-start}
#v{width:760px;max-width:100%;image-rendering:pixelated;background:#000;cursor:crosshair}
.panel{min-width:260px;flex:1}
.row{display:grid;grid-template-columns:64px 1fr 44px;gap:8px;align-items:center;margin:6px 0}
label{color:#999}
input[type=range]{width:100%}
b{color:#fff;font-weight:600;text-align:right}
button{font:13px system-ui;padding:7px 13px;margin:4px 6px 0 0;border:0;border-radius:5px;
  background:#2b6;color:#042;font-weight:600;cursor:pointer}
button.alt{background:#444;color:#ddd}
#msg{color:#8d8;min-height:18px;margin-top:8px}
p{color:#888}
</style>
<div class=wrap>
  <div>
    <img id=v src="/stream.mjpg">
    <p>left = camera, right = mask &middot; @@HINT@@</p>
  </div>
  <div class=panel>
    <div id=rows></div>
    <button onclick=save()>Save to tune.json</button>
    <button class=alt onclick=go('/clear')>Clear samples</button>
    <div id=msg></div>
  </div>
</div>
<script>
let built = false;
async function go(u){ const r = await fetch(u); return r.json(); }
function render(s){
  if(!built){
    document.getElementById('rows').innerHTML = s.controls.map(c =>
      `<div class=row><label>${c.name}</label>
       <input type=range min=0 max=${c.max} value=${c.value} id="s_${c.name}"
              oninput="set('${c.name}',this.value)">
       <b id="n_${c.name}">${c.value}</b></div>`).join('');
    built = true;
  }
  for(const c of s.controls){
    document.getElementById('s_'+c.name).value = c.value;
    document.getElementById('n_'+c.name).textContent = c.value;
  }
}
async function set(name,value){
  document.getElementById('n_'+name).textContent = value;
  render(await go('/set?name='+encodeURIComponent(name)+'&value='+value));
}
async function save(){
  const r = await go('/save');
  document.getElementById('msg').textContent = 'saved ' + r.path;
}
document.getElementById('v').addEventListener('click', async e => {
  const img = e.target, box = img.getBoundingClientRect();
  // The stream is camera and mask side by side, so the natural width is two frames.
  // Either half maps to the same pixel: fold the right half back onto the left.
  const scale = img.naturalWidth / box.width;
  let x = (e.clientX - box.left) * scale, y = (e.clientY - box.top) * scale;
  const half = img.naturalWidth / 2;
  if (x >= half) x -= half;
  render(await go('/sample?x='+Math.round(x)+'&y='+Math.round(y)));
});
setInterval(async () => { if(built) render(await go('/state')); }, 2000);
go('/state').then(render);
</script>
"""


def json_reply(payload):
    return "application/json", json.dumps(payload).encode()


def routes():
    def r_state(_q):
        return json_reply(state())

    def r_set(q):
        name, value = q.get("name", [""])[0], q.get("value", ["0"])[0]
        with LOCK:
            if any(c[0] == name for c in CONTROLS):
                set_control(name, value)
        return json_reply(state())

    def r_sample(q):
        with LOCK:
            sample_at(q.get("x", [0])[0], q.get("y", [0])[0])
        return json_reply(state())

    def r_clear(_q):
        with LOCK:
            clear_samples()
        return json_reply(state())

    def r_save(_q):
        return json_reply({"path": save_tune()})

    return {"/state": r_state, "/set": r_set, "/sample": r_sample,
            "/clear": r_clear, "/save": r_save}


def run_web(port):
    page = (PAGE.replace(b"@@TITLE@@", TARGET["title"].encode())
                .replace(b"@@HINT@@", TARGET["hint"].encode()))
    # fatal=True: for bot.py a dead stream just costs you the video, but here the
    # page IS the interface, so a taken port means there is nothing to fall back to.
    stream = bot.Streamer(port, quality=70, fps=12, page=page, routes=routes(),
                          label="tune " + TARGET["title"], fatal=True).start()
    print("click the %s on either image, then Save. Ctrl-C to quit." % TARGET["title"])
    grab = bot.open_camera()
    try:
        while True:
            frame = grab()
            if frame is None:
                continue
            LAST["frame"] = frame.copy()   # sampling reads raw pixels, not the overlay
            with LOCK:
                mask = TARGET["mask"](frame)        # from the clean frame, before drawing
                status = TARGET["overlay"](frame)
                lo, hi = list(bot.TUNE[TARGET["lo"]]), list(bot.TUNE[TARGET["hi"]])
            side = cv2.hconcat([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
            stream.offer(side, status=[
                "H %d-%d   S %d-%d   V %d-%d" % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]),
            ] + status)
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        stream.close()


# ---------------------------------------------------------------- cv2 tuner
def run_gui():
    win = "tune %s (click it, s=save, c=clear, q=quit)" % TARGET["title"]

    def on_click(event, x, y, *_):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        sample_at(x, y)
        # The loop below reads every trackbar back into TUNE each frame, so a sample
        # that is not pushed onto the trackbars too is overwritten one frame later.
        for c in CONTROLS:
            cv2.setTrackbarPos(c[0], win, get_control(c[0]))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_click)
    for name, _key, _idx, top, _scale in CONTROLS:
        cv2.createTrackbar(name, win, min(top, get_control(name)), top, lambda _: None)
    grab = bot.open_camera()
    while True:
        frame = grab()
        if frame is None:
            continue
        LAST["frame"] = frame.copy()
        for c in CONTROLS:
            set_control(c[0], cv2.getTrackbarPos(c[0], win))
        TARGET["overlay"](frame)
        cv2.imshow(win, frame)
        k = cv2.waitKey(30) & 0xFF
        if k == ord("s"):
            save_tune()
        elif k == ord("c"):
            clear_samples()
        elif k == ord("q"):
            break
    cv2.destroyAllWindows()


def main(target=None):
    configure(target or BALL)
    web = "--web" in sys.argv
    gui = "--gui" in sys.argv
    if not web and not gui:
        # Over ssh there is no screen to put a window on, and cv2 fails with a
        # message about the Qt platform plugin that says nothing about the cause.
        web = not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if web:
            print("no display detected - starting the web tuner (--gui to force a window)")
    if web:
        return run_web(int(bot.argval("--web", TARGET["port"])))
    return run_gui()


if __name__ == "__main__":
    main()
