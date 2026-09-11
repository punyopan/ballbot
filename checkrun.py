#!/usr/bin/env python3
"""Why won't the robot run?  ->  python3 checkrun.py

Walks the stack in the order things actually break and prints a FIX line for every
failure. Nothing here drives a motor, so it is safe with the wheels on the ground.

The check that catches most "it doesn't run" reports is CONFLICT: ballbot.service
already holds GPIO 4 and the twelve motor pins, so a second bot.py started by hand
dies on a busy pin before it reaches the start prompt.
"""
import os
import subprocess
import sys
import time

sys.argv.append("--dry")          # keep bot.py from claiming hardware on import
import bot                        # noqa: E402

HERE = bot.HERE
OK, BAD, WARN = [], [], []


def say(state, label, detail, fix=None):
    """state: 'ok' | 'bad' | 'warn'."""
    mark = {"ok": "  OK  ", "bad": " FAIL ", "warn": " WARN "}[state]
    print("[%s] %-14s %s" % (mark, label, detail))
    if fix:
        print("%18sFIX: %s" % ("", fix))
    {"ok": OK, "bad": BAD, "warn": WARN}[state].append(label)


def head(title):
    print("\n--- %s %s" % (title, "-" * max(0, 56 - len(title))))


# ------------------------------------------------------------------ 1. conflict
def check_conflict():
    head("1. is something else already holding the GPIOs")
    try:
        active = subprocess.run(["systemctl", "is-active", "ballbot"],
                                capture_output=True, text=True).stdout.strip()
    except FileNotFoundError:
        active = "unknown"
    others = []
    for line in subprocess.run(["ps", "-eo", "pid,args"], capture_output=True,
                               text=True).stdout.splitlines():
        if "bot.py" in line and "checkrun" not in line and line.split()[0].isdigit():
            if int(line.split()[0]) != os.getpid():
                others.append(line.strip())
    if active == "active" or others:
        say("bad", "conflict",
            "ballbot.service is %s; %d other bot.py process(es)" % (active, len(others)),
            "sudo systemctl stop ballbot   (then re-run this, and run bot.py by hand)")
        for o in others:
            print("%18s%s" % ("", o))
    else:
        say("ok", "conflict", "no other bot.py holding the pins")


# ------------------------------------------------------------------ 2. python
def check_python():
    head("2. interpreter and the two libraries that break on a Pi")
    say("ok", "python", sys.executable)
    if "handenv" not in sys.executable:
        say("warn", "env", "not running from handenv",
            "conda activate handenv   (ballbot.service uses handenv's python)")
    import platform
    arch = platform.machine()
    on_arm = arch.startswith(("arm", "aarch"))
    say("ok", "machine", "%s  (aarch64 = 64-bit Pi OS)" % arch)
    for name in ("numpy", "cv2"):
        try:
            mod = __import__(name)
        except Exception as e:
            say("bad", name, "import failed: %s" % e,
                "sudo apt install python3-numpy python3-opencv")
            continue
        # pip serves one wheel to every ARM board; apt builds for this exact CPU.
        # A pip wheel is the usual cause of "Illegal instruction" on a Pi.
        from_apt = "/usr/lib/" in (mod.__file__ or "")
        note = "apt, good" if from_apt else (
            "PIP - usual cause of Illegal instruction on a Pi" if on_arm else "pip, fine off-Pi")
        say("ok" if (from_apt or not on_arm) else "warn", name,
            "%s  (%s)" % (mod.__version__, note),
            None if (from_apt or not on_arm) else "sudo apt install python3-%s" %
            ("numpy" if name == "numpy" else "opencv"))
    try:
        import numpy
        # numpy usually dies inside BLAS rather than on import, so do real maths.
        numpy.zeros((32, 32)).dot(numpy.ones((32, 32))).sum()
        say("ok", "numpy maths", "matrix multiply survived")
    except Exception as e:
        say("bad", "numpy maths", str(e), "sudo apt install python3-numpy")


# ------------------------------------------------------------------ 3. gpio
def check_gpio():
    head("3. GPIO")
    try:
        import gpiozero
    except Exception as e:
        say("bad", "gpiozero", "import failed: %s" % e, "pip install gpiozero lgpio")
        return
    say("ok", "gpiozero", getattr(gpiozero, "__version__", "installed"))
    try:
        from gpiozero import Device
        Device.ensure_pin_factory()
        say("ok", "pin factory", type(Device.pin_factory).__name__)
    except Exception as e:
        say("bad", "pin factory", str(e),
            "pip install lgpio   (and check you're in the gpio group: groups)")


# ------------------------------------------------------------------ 4. button
def check_button():
    head("4. start button on GPIO %d" % bot.BUTTON_PIN)
    raw = subprocess.run(["pinctrl", "get", str(bot.BUTTON_PIN)],
                         capture_output=True, text=True).stdout.strip()
    if raw:
        say("ok", "pin state", raw)
    try:
        from gpiozero import Button
        b = Button(bot.BUTTON_PIN)
    except Exception as e:
        say("bad", "button", "cannot claim GPIO %d: %s" % (bot.BUTTON_PIN, e),
            "something else holds it - see the conflict check above")
        return
    try:
        # Held down at rest means it is wired to 3.3 V instead of GND, and bot.py
        # would blow straight through wait_for_press and then stop on frame one.
        if b.is_pressed:
            say("bad", "button", "reads PRESSED while untouched",
                "wire the other side to GND, not 3.3 V - or use: bot.py --nobutton")
        else:
            print("       press the button now (3 s)...", flush=True)
            got = b.wait_for_press(timeout=3)
            if got:
                say("ok", "button", "press detected - wiring is good")
            else:
                say("warn", "button", "no press seen (you may simply not have pressed)",
                    "if nothing is wired to GPIO %d: python3 bot.py --nobutton" % bot.BUTTON_PIN)
    finally:
        b.close()


# ------------------------------------------------------------------ 5. i2c
def check_motor_pins():
    head("4b. motor pins")
    say("ok", "channels", "%d (%s)%s" % (len(bot.MOTORS), ", ".join(bot.MOTORS),
                                         "  TANK/differential" if bot.TANK else "  mecanum"))
    # The failure that looks exactly like a dead robot while every IN pin measures
    # right: enable gates the H-bridge, so an undriven enable means no current at all.
    undriven = [n for n, (_f, _b, e) in bot.MOTORS.items() if e is None]
    if undriven:
        say("warn", "enable", "not driven for: %s" % ", ".join(undriven),
            "correct ONLY if ENA/ENB is jumpered to 5 V. If a wire runs to the Pi, "
            "put that GPIO in MOTORS or the wheels stay dead")
    else:
        say("ok", "enable", "every channel drives its own enable pin")
    seen = {}
    for name, pins in bot.MOTORS.items():
        for p in pins:
            if p is None:
                continue
            if p in seen:
                say("bad", "pin clash", "GPIO %d used by %s and %s" % (p, seen[p], name),
                    "two devices cannot share a pin - renumber one in MOTORS")
            seen[p] = name
    for other, pin in (("button", bot.BUTTON_PIN), ("kicker", bot.KICKER_PIN)):
        if pin is not None and pin in seen:
            say("bad", "pin clash", "GPIO %d is both %s and %s" % (pin, seen[pin], other),
                "renumber one of them")
    if not BAD:
        say("ok", "no clashes", "%d pins, all distinct" % len(seen))


def check_i2c():
    head("5. I2C (IMU, and the LCD if fitted)")
    if not os.path.exists("/dev/i2c-1"):
        say("bad", "/dev/i2c-1", "missing", "sudo raspi-config -> Interface Options -> I2C -> enable")
        return
    try:
        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus
        found = []
        with SMBus(1) as busobj:
            for addr in range(0x03, 0x78):
                try:
                    busobj.read_byte(addr)
                    found.append(addr)
                except OSError:
                    pass
    except Exception as e:
        say("bad", "i2c scan", str(e), "sudo apt install python3-smbus i2c-tools")
        return
    known = {0x68: "MPU-6050 gyro", 0x69: "MPU-6050 (alt)", 0x28: "BNO055",
             0x29: "BNO055 (alt)", 0x27: "LCD", 0x3F: "LCD (alt)"}
    say("ok", "i2c scan", ", ".join("0x%02X %s" % (a, known.get(a, "?")) for a in found)
        or "nothing found")
    if not any(a in found for a in (0x68, 0x69, 0x28, 0x29)):
        say("warn", "IMU", "no gyro on the bus",
            "without a heading the robot cannot tell your goal from theirs (rule 6.1)")


# ------------------------------------------------------------------ 6. camera
def check_camera():
    head("6. camera and ball detection")
    try:
        grab = bot.open_camera()
    except Exception as e:
        say("bad", "camera", "open failed: %s" % e, "check the ribbon/USB, then: bot.py --check")
        return
    frame = None
    for _ in range(10):          # first frames off a USB webcam are often empty
        frame = grab()
        if frame is not None:
            break
        time.sleep(0.1)
    if frame is None:
        say("bad", "camera", "NO FRAME",
            "USB webcam: check `ls /dev/video*`. Pi cam: check the ribbon direction.")
        return
    say("ok", "camera", "%s  contrast %.1f" % (frame.shape, frame.std()))
    if bot.is_blind(frame):
        say("bad", "blind", "contrast below %s - lens covered, dark room, or dead sensor"
            % bot.TUNE["blind_std"], "uncover the lens / turn the lights on")
        return
    found = bot.find_ball(frame)
    if found:
        say("ok", "find_ball", "dx %+.2f  radius %.0f px" % found)
    else:
        say("warn", "find_ball", "no ball in view",
            "aim the camera at the ball; if it still misses: python3 calibrate.py")


# ------------------------------------------------------------------ 7. tuning
def check_tune():
    head("7. tuning")
    path = os.path.join(HERE, "tune.json")
    if os.path.exists(path):
        say("ok", "tune.json", path)
    else:
        say("warn", "tune.json", "missing - running on built-in DEFAULTS",
            "python3 calibrate.py on the field, under the real lights, then press 's'")
    if bot.SONAR_PINS is None:
        say("warn", "sonar", "not fitted (optional)")
    else:
        say("ok", "sonar", "pins %s" % (bot.SONAR_PINS,))


def main():
    print("checkrun - why won't the robot run?")
    for fn in (check_conflict, check_python, check_gpio, check_button, check_motor_pins,
               check_i2c, check_camera, check_tune):
        try:
            fn()
        except Exception as e:
            say("bad", fn.__name__, "check itself crashed: %r" % e)
    head("summary")
    print("%d ok, %d warnings, %d failures" % (len(OK), len(WARN), len(BAD)))
    if BAD:
        print("\nfix these first: " + ", ".join(BAD))
        return 1
    print("\nNothing blocking. If the robot still sits still, it IS running and just")
    print("hasn't been started: it waits at the prompt until the button (or the")
    print("--nobutton countdown) fires. Watch it decide with:  python3 bot.py --dry")
    return 0


if __name__ == "__main__":
    sys.exit(main())
