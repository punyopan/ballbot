#!/usr/bin/env python3
"""Short movement check. Run with --dry on a laptop; no camera needed."""
import argparse
import math
import time

import bot

MOVES = (
    ("FORWARD", 1, 0, 0),
    ("BACKWARD", -1, 0, 0),
    ("STRAFE LEFT", 0, 1, 0),
    ("STRAFE RIGHT", 0, -1, 0),
    ("TURN LEFT", 0, 0, 1),
    ("TURN RIGHT", 0, 0, -1),
)


def positive_number(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def wait_or_stop(robot, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if robot.button and robot.button.is_pressed:
            return False
        time.sleep(min(0.02, max(0, deadline - time.monotonic())))
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry", action="store_true", help="print commands without hardware")
    parser.add_argument("--speed", type=positive_number, default=0.4, help="power up to 1 (default: 0.4)")
    parser.add_argument("--seconds", type=positive_number, default=1.0, help="seconds per move (default: 1)")
    args = parser.parse_args(argv)
    if args.speed > 1:
        parser.error("--speed must be at most 1")
    bot.DRY = args.dry
    robot = bot.Robot()
    try:
        robot.stop()
        if args.dry:
            print("DRY RUN: no hardware is tested.", flush=True)
        else:
            print("Raise the robot so all wheels are off the ground.", flush=True)
            print("Starting in 3 seconds; press START or Ctrl-C to stop.", flush=True)
            if not wait_or_stop(robot, 3):
                print("Stopped by start button.")
                return
        for label, vx, vy, turn in MOVES:
            if robot.button and robot.button.is_pressed:
                print("Stopped by start button.")
                return
            print(label, flush=True)
            robot.drive(vx * args.speed, vy * args.speed, turn * args.speed)
            if not wait_or_stop(robot, args.seconds):
                print("Stopped by start button.")
                return
            robot.stop()
            if not wait_or_stop(robot, 0.5):
                print("Stopped by start button.")
                return
        print("Sequence complete. " + (
            "Commands work; run on the Pi to check physical movement."
            if args.dry else "Check that the wheels matched each printed direction."
        ))
    except KeyboardInterrupt:
        print("\nStopped by Ctrl-C.")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
