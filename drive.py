#!/usr/bin/env python3
"""Drive the robot by hand - no camera, no ball, no start button.

  python3 drive.py forward              2 s forward
  python3 drive.py left --seconds 3     strafe left for 3 s
  python3 drive.py forward left spin    a sequence, in order
  python3 drive.py circle --speed 0.5   drive around in an arc
  python3 drive.py --list               show every move
  python3 drive.py forward --dry        print the wheel values, move nothing

Stops on Ctrl-C, and always stops the motors on the way out. Speed defaults to a
gentle 0.4 - full TUNE["speed"] is 0.75 and will get away from you indoors.
"""
import argparse
import math
import subprocess
import sys
import time

import bot

# name -> (vx, vy, w, what you should see)
# vx = +forward, vy = +left, w = +counter-clockwise. Same convention as bot.mix.
MOVES = {
    "forward": (1, 0, 0, "drives straight forward"),
    "back": (-1, 0, 0, "drives straight backward"),
    "left": (0, 1, 0, "slides LEFT, nose stays pointing the same way"),
    "right": (0, -1, 0, "slides RIGHT, nose stays pointing the same way"),
    "spin": (0, 0, 1, "rotates counter-clockwise on the spot"),
    "spinright": (0, 0, -1, "rotates clockwise on the spot"),
    "circle": (1, 0, 0.6, "arcs around counter-clockwise while moving forward"),
    "diag": (1, 1, 0, "slides forward-left at 45 deg, nose straight"),
    "stop": (0, 0, 0, "all motors off"),
}


def positive(value):
    n = float(value)
    if not math.isfinite(n) or n <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return n


def warn_if_service_running():
    """ballbot.service holds all twelve motor pins. A second process asking for them
    dies on a busy pin, so say so plainly instead of letting gpiozero throw."""
    try:
        state = subprocess.run(["systemctl", "is-active", "ballbot"],
                               capture_output=True, text=True).stdout.strip()
    except FileNotFoundError:
        return
    if state == "active":
        print("ballbot.service is running and already holds the motor pins.")
        print("  sudo systemctl stop ballbot")
        sys.exit(1)


def run_move(robot, name, speed, seconds):
    vx, vy, w, what = MOVES[name]
    print("%-10s %s" % (name, what), flush=True)
    robot.drive(vx * speed, vy * speed, w * speed)
    time.sleep(seconds)
    robot.stop()
    time.sleep(0.4)          # let it settle before the next move


def probe_pins(seconds=4.0):
    """Hold one IN pin high at a time, steady DC, no PWM and no Motor object.

    This splits "the Pi isn't sending anything" from "the driver board isn't passing
    it on". Probe with a multimeter, black on any Pi GND pin:
      Pi pin -> ~3.3 V while named, 0 V otherwise  = the Pi is fine, fault is the board
      L298N OUT pair -> ~0 V while its IN is high  = no motor supply, or no shared GND
    """
    from gpiozero import DigitalOutputDevice
    print("Probe each pin against GND. Ctrl-C to stop.\n")
    for side, (fwd, back, en) in bot.MOTORS.items():
        # An IN pin alone does nothing while ENA/ENB is low, so hold enable high for
        # the whole channel - otherwise this probe reads dead on working hardware.
        gate = DigitalOutputDevice(en) if en is not None else None
        if gate:
            gate.on()
            print("  %s enable  -> GPIO %-2d held high" % (side, en))
        try:
            for label, pin in (("forward", fwd), ("backward", back)):
                print("  %s %-8s -> GPIO %-2d high for %.0f s" % (side, label, pin, seconds),
                      flush=True)
                d = DigitalOutputDevice(pin)
                try:
                    d.on()
                    time.sleep(seconds)
                finally:
                    d.off()
                    d.close()
        finally:
            if gate:
                gate.off()
                gate.close()
    print("\nEvery pin driven. If each measured ~3.3 V at the Pi but no motor turned,")
    print("the Pi is not the problem - check the motor supply on +12V, and that each")
    print("L298N GND is tied to a Pi GND.")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("moves", nargs="*", help="one or more of: " + ", ".join(MOVES))
    parser.add_argument("--speed", type=positive, default=0.4, help="0..1 (default 0.4)")
    parser.add_argument("--seconds", type=positive, default=2.0, help="per move (default 2)")
    parser.add_argument("--now", action="store_true", help="skip the 3 s countdown")
    parser.add_argument("--dry", action="store_true", help="print wheel values, move nothing")
    parser.add_argument("--list", action="store_true", help="show every move and exit")
    parser.add_argument("--pins", action="store_true",
                        help="hold each motor pin high in turn, to probe with a multimeter")
    args = parser.parse_args(argv)

    if args.pins:
        warn_if_service_running()
        try:
            probe_pins()
        except KeyboardInterrupt:
            print("\nstopped by Ctrl-C")
        return 0
    if args.list or not args.moves:
        print("moves:")
        for name, (vx, vy, w, what) in MOVES.items():
            print("  %-10s vx%+.1f vy%+.1f w%+.1f   %s" % (name, vx, vy, w, what))
        return 0
    unknown = [m for m in args.moves if m not in MOVES]
    if unknown:
        parser.error("unknown move(s): %s\nvalid: %s" % (", ".join(unknown), ", ".join(MOVES)))
    if args.speed > 1:
        parser.error("--speed must be at most 1")

    bot.DRY = args.dry
    if not args.dry:
        warn_if_service_running()
    robot = bot.Robot()
    try:
        robot.stop()
        if not args.dry and not args.now:
            print("%s at speed %.2f, %.1f s each." %
                  (" -> ".join(args.moves), args.speed, args.seconds))
            print("Clear the area (or put it on a box). Ctrl-C stops.", flush=True)
            for n in (3, 2, 1):
                print("  %d..." % n, flush=True)
                time.sleep(1)
        for name in args.moves:
            run_move(robot, name, args.speed, args.seconds)
        print("done")
    except KeyboardInterrupt:
        print("\nstopped by Ctrl-C")
    finally:
        robot.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
