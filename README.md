# หุ่นยนต์แตะบอล — build + code plan

Base kit (fixed by rule 4.1): Raspberry Pi 4, 4× mecanum, 4 motors, top camera,
motor driver board, 18650 battery box. Everything below is legal modification.

## Files

| file | what it is |
|---|---|
| `bot.py` | the whole robot: vision → strategy → mecanum drive |
| `calibrate.py` | slider tool to lock the ball colour on the real field → `tune.json` |
| `test_bot.py` | self-check for the driving maths and the strategy, runs on a laptop |
| `tune.json` | written by calibrate; every number you'd want to change at the venue |
| `ballbot.service` | starts `bot.py` at power-on, because rule 9.1.3 bans laptops at the field |
| `HARDWARE.md` | pinout, power, wiring, shopping list |

## Install (Raspberry Pi OS Bookworm)

Everything comes from apt. Don't use pip here: `picamera2` is built against the
system `libcamera` and a pip copy won't find it, and Bookworm blocks system-wide pip
anyway (`externally-managed-environment`).

```bash
sudo apt update && sudo apt install -y python3-opencv python3-picamera2 python3-gpiozero python3-smbus i2c-tools
```

Then turn on the interfaces — `sudo raspi-config` → Interface Options → **I2C** on,
and **Camera** on if your image still lists it. Reboot, then check the IMU answers:

```bash
i2cdetect -y 1
```

`68` in the grid is the MPU-6050, `28` or `29` is a BNO055. An empty grid means
wiring, not software — check 3.3 V, GND, SDA on GPIO 2, SCL on GPIO 3.

Only if you buy a BNO055 later (pip is unavoidable for this one):

```bash
sudo pip3 install --break-system-packages adafruit-circuitpython-bno055
```

Nothing above is needed to run the tests — those are pure Python:

```bash
python3 test_bot.py     # maths sanity, no hardware needed
```
```bash
python3 bot.py --dry    # prints motor commands instead of driving
```

## The one strategic decision: **do not build a gripper**

Rule 5.2/5.3 says the moment the ball can't escape on its own you are "คีบ":
wheels must stop, 5 second limit, no help from your ally, and if you move you
get warned and hand the ball over. A gripper turns your fastest robot into a
statue. Rule 5.1 says touching, pushing and kicking with an **open** front is
totally free — keep driving.

So: open front, ball always able to roll out. All the code assumes this.

## What to add (in order of points per baht)

1. **BNO055 IMU (~350฿)** — the single biggest upgrade. Mecanum wheels drift; without
   a compass the robot has no idea which goal is which. At start you aim the robot
   at the enemy goal and press the button — that heading is remembered for 12 minutes.
   `bot.py` already reads it and orbits the ball until the nose points at the goal.
   No IMU → the code still works, it just chases the ball blindly.
   **Already own an MPU-6050?** It works — `bot.py` falls back to it automatically
   (`pip3 install smbus2`). It has no magnetometer, so it only knows how far you have
   turned since start, and that estimate creeps. Two things make it good enough:
   the startup bias calibration (**keep the robot dead still** while it prints
   "calibrating gyro" — skip this and it drifts degrees per *second*), and rule 6.3,
   which restarts the match after every goal so you re-aim and re-zero constantly.
   If it turns away from the goal instead of toward it, set `imu_sign: -1`.
   Buy the BNO055 when you can: it survives a long goalless stretch, the MPU-6050 is
   the one that quietly goes wrong around minute five.
2. **Curved front plow, ~70 mm deep, open top** — the whole 70 mm forward allowance
   in rule 4.3. A shallow V centres the ball while you drive so you can push straight.
   Aluminium or 3 mm PVC sheet. Not a cage (rule 4.4).
3. **HC-SR04 or VL53L0X on the front** — stops you shoving a robot into the wall,
   which is a foul under 10.1. `bot.py` backs off automatically.
4. **Side wings, 40 mm each, angled outward and open at the front** — legal under
   4.2/5.5, they funnel the ball into the plow. Keep them fixed and open; a wing that
   closes counts as gripping.
5. **12 V solenoid kicker (optional)** — one GPIO through a MOSFET and a flyback
   diode. `KICKER_PIN` in `bot.py`; the fire only happens when the ball is close
   AND the nose is on the goal, with a 2.2 s cooldown for rule 5.4.
6. **Rubber tread or O-rings on the rollers** — the green field is slippery and
   mecanum loses grip first.
7. Kill switch (required, rule 4.5), battery in its box (required), team colour
   markers on ≥2 sides (rule 2.3), cable tie everything (9.4: only one 30 s repair
   per match).

Budget check: 320 × 280 × 230 mm and 2.50 kg measured **with wings and kicker fully
extended**. Plow 70 mm + base leaves you very little length — measure before you glue.

## About "battle bots" / interrupting the other robots

Legal, within limits, and the rules quietly reward it:

- Bumping and body-blocking while chasing the ball is normal play — no rule against contact.
- **Pinning** an opponent against the wall and not backing off is a foul (10.1).
  The sonar back-off is exactly what keeps you legal here.
- Two robots parking in front of your own goal gets you split up (12.3).
- Sharp parts, anything that flips or damages, glue, flame → instant removal (4.4, 10.1).

So don't build a weapon — build a robot that gets to the ball first and is heavy
and low enough to shoulder people off it. With an ally on your side, the good play
is one robot on the ball and one sitting between the ball and your own goal.

## If someone tries to blind the camera

Not explicitly banned by 4.4 (which lists sharp parts, liquid, glue, flame, smoke),
but 1.3 and 10.3 give the referee the final call and this would almost certainly get
stopped — and a cloth launcher spends their 2.5 kg and 320 mm on something that
scores zero goals. Still, a blinded camera also happens from glare, a knock, or
another robot parking in your face, so it's handled:

- `is_blind()` — anything pressed on a lens defocuses to a flat blur, so the frame
  loses almost all contrast. Below `blind_std` for `blind_frames` in a row = blind.
- Blind mode ignores the camera and drives on the **compass** toward the enemy goal,
  sweeping side to side and kicking. A dead camera costs you the ball, not the match.
  This only works if you fitted the IMU — one more reason it's item 1 on the list.
- Physically: recess the lens in a short hood or tube, and tilt it forward ~30° so
  a falling cloth slides off instead of lying flat on it. Mount it low behind the
  plow, not on top of a mast where anything can land on it.

## Competition day checklist

Things that lose matches for reasons that have nothing to do with your code:

- [ ] **Pi powered separately from the motors.** Four motors stalling against a wall
      drag the 18650 pack down, the Pi browns out and reboots, and your robot is dead
      for the rest of the match. Give the Pi its own power bank, or its own buck
      converter off the pack. This is the single most common Raspberry Pi robot failure.
- [ ] **`sudo systemctl enable --now ballbot`** — rule 9.1.3 means you cannot open a
      laptop at the field. It has to boot into the program already running.
- [ ] **Spare SD card, flashed and tested.** Corruption from a hard power-cut is the
      second most common failure. Ten minutes to make, saves the whole tournament.
- [ ] Spare charged 18650 pack. You play 4 matches (rule 8.1).
- [ ] Kill switch fitted and easy to reach (4.5), battery in its box, no bare cells.
- [ ] Team colour markers on **two** sides (2.3) — easy to forget, checked at inspection.
- [ ] Weigh it fully assembled: 2.50 kg with battery, wings out, plow on (4.3).
- [ ] Walk the 10-item inspection sheet (ภาคผนวก ก) yourself before you hand it over.
- [ ] Use your 30 s field test (12.1) to run `calibrate.py` under the venue's lights,
      not to drive around. Lighting is what breaks colour tracking, and it's the only
      time you'll see the real field.

## Tuning at the venue

1. `python3 calibrate.py` — sliders until only the ball is white, press `s`.
2. Drive on the field. If it strafes the wrong way set `invert_strafe: -1` in
   `tune.json`; if it spins the wrong way set `invert_turn: -1`.
3. Hold the ball where you want the robot to commit and read the printed `r` —
   that number is your `close_radius`.
4. Motors buzzing but not turning at low power → raise `min_duty`.
