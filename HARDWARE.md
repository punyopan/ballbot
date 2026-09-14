# Hardware — wiring, parts, and the things that fry a Raspberry Pi

The pin numbers here match the top of `bot.py` exactly. Change one, change both.

## Pinout (BCM numbering)

**Two L298N boards, one motor per wheel — four independent channels.**

*Board 1 — front:*

| GPIO | L298N pin | Goes to | Notes |
|---|---|---|---|
| 5, 6 | IN1, IN2 | Motor **FL** | |
| 12 | **ENA** | gates OUT1/OUT2 = FL | hardware-PWM capable |
| 13, 19 | IN3, IN4 | Motor **FR** | |
| 18 | **ENB** | gates OUT3/OUT4 = FR | hardware-PWM capable |

*Board 2 — rear:*

| GPIO | L298N pin | Goes to | Notes |
|---|---|---|---|
| 16, 20 | IN1, IN2 | Motor **BL** | |
| 21 | **ENA** | gates OUT1/OUT2 = BL | |
| 23, 24 | IN3, IN4 | Motor **BR** | |
| 25 | **ENB** | gates OUT3/OUT4 = BR | |

**Enable needs no jumper cap.** A cap just ties enable to 5 V; a GPIO does the same
job and adds speed control. If caps are fitted, pull them — they short enable to 5 V
and fight the pin driving it. Short of wire, a dupont lead from the enable header to
the board's own 5 V pin substitutes for a missing cap (full speed, no PWM on that
channel).

*Everything else:*

| GPIO | Goes to | Notes |
|---|---|---|
| 4 | Start button | other side to GND, `gpiozero` pulls it up internally |
| 2, 3 | I2C SDA, SCL | IMU. Don't reassign these, they're the hardware I2C |

No kicker and no sonar are fitted, and `bot.py` carries no code for either — we push
the ball in, and the stuck-detector in `play()` is what digs us out of a wall grind.
Nothing collides, and I2C is left clear. There are 12 pins on motors alone, so if you
add anything later, take it from 7, 8, 9, 10, 11, 17, 22, 26, 27.

## Power — the thing that actually kills these robots

Run **two rails from one battery pack**:

```
18650 pack ──┬── motor driver Vmotor   (sags hard when motors stall)
             │
             └── buck converter 5 V 3 A ── Pi 5 V   (stays clean)

             all grounds tied together at ONE point
```

Four mecanum motors stalling against a wall pull the pack voltage down for a moment.
If the Pi is on that same sagging rail it browns out, reboots, and your robot is a
brick for the rest of the match — clock still running, and rule 9.4 gives you exactly
one 30-second repair. A separate buck converter (or a plain USB power bank) for the
Pi costs about 100 ฿ and removes the failure entirely.

Do tie the grounds together. A motor driver and a Pi with separate grounds gives you
random, unrepeatable misbehaviour that looks exactly like a software bug.

## Wiring each part

Wire one thing, then run its check below before wiring the next. Debugging four
mistakes at once is what turns an afternoon into a week.

**Motor driver.** Two dual-channel boards for four motors. L298N is the one everyone
has, but it burns ~2 V and gets hot; TB6612FNG is cheaper, cooler, and more efficient
if you're still buying. Either way the wiring is the same three pins per motor.
Split them **driver A = FL + FR**, **driver B = BL + BR**, matching `MOTORS` in `bot.py`.

Both drivers take **battery voltage directly** — they do *not* run through the buck
converter. The buck exists to keep motor sag off the Pi; putting the motors behind it
undoes that and overloads it besides.

Three things that will bite you with two drivers:

- **Take the ENA/ENB jumpers off.** They ship jumpered to 5 V, meaning "always full
  speed". The code PWMs the enable pin (the third pin in each `MOTORS` entry), so with
  the jumpers on, every speed value in `tune.json` does nothing. This is the most
  common reason mecanum code appears not to work.
- **Both drivers' GND must reach the Pi's GND**, not just the battery negative. The
  direction and PWM signals are measured against that reference, and without it the
  motors misbehave in a way that looks exactly like a software bug.
- **Leave each driver's 5 V pin unconnected.** With its jumper on that pin is an
  *output* from a small onboard regulator — nowhere near enough for a Pi 4, and it
  would fight the buck converter.

*Check it* — robot up on a box, wheels hanging free:

```bash
python3 bot.py --wheels
```

It spins each wheel alone first, then does the six whole-robot moves, announcing each
one. If a single wheel spins backwards, swap **that motor's two wires at the driver** —
don't fix it in code, or strafing goes diagonal later. Once all four are right, if the
robot as a whole strafes or spins the wrong way, that's `invert_strafe` / `invert_turn`
in `tune.json`. A wheel that buzzes without turning wants a higher `min_duty`.

**MPU-6050 / GY-521.** VCC → 3.3 V, GND → GND, SDA → GPIO 2, SCL → GPIO 3. Mount it
as far from the motors and their wires as you can: it's measuring rotation, and motor
current makes magnetic noise. Enable I2C with `sudo raspi-config` → Interface Options.

*Check it* — `sudo apt install i2c-tools`, then:

```bash
i2cdetect -y 1
```

`68` in the grid is the MPU-6050. An empty grid is always wiring, never software.
`python3 bot.py --check` then prints a live heading; turn the robot by hand and it
should change, and settle when you stop. If it counts the wrong way, `imu_sign: -1`.

The code reads it through `smbus` (apt's `python3-smbus`) or `smbus2` if you happen to
have it — no pip needed either way.

**BNO055, if you upgrade.** Same four wires. It clock-stretches and the Pi's I2C
hardware can't cope, so add this to `/boot/firmware/config.txt` and reboot:

```
dtparam=i2c_arm_baudrate=10000
```

**HC-SR04 — read this or you damage the Pi.** Its echo pin outputs **5 V** and Pi GPIO
takes a maximum of **3.3 V**. Put a divider on the echo line:

```
ECHO ──[ 1 kΩ ]──┬── GPIO 17
                 │
              [ 2 kΩ ]
                 │
                GND
```

Trigger (GPIO 22) is an output, so it needs nothing. If you're buying fresh, get a
**VL53L0X** instead — it's I2C, natively 3.3 V, no divider, more accurate, and shares
the two pins the IMU already uses.

> **Not currently fitted.** `bot.py` has no sonar code at all — there is no
> `SONAR_PINS` and no `front_cm()`. Adding one back means restoring both, and note
> that gpiozero's `DistanceSensor.distance` **blocks forever** when nothing echoes,
> so it has to be read on a background thread or it freezes the match loop.

**Solenoid kicker — skip this for your first competition.** `bot.py` has no kicker
code and the robot works exactly as it is. A plow at full throttle
already puts the ball in the goal; a kicker spends weight, length, current and one more
brownout risk to do the same job. Build it later for the design prize (12.4).

If you do build one, a 12 V solenoid pulls several amps, far more than a GPIO can do.
You need:

- a logic-level MOSFET (IRLZ44N — the "LZ" matters, a plain IRF44N won't switch fully at 3.3 V)
- GPIO 27 → 220 Ω → gate, and a 10 kΩ from gate to GND so it stays off during boot
- a flyback diode (1N5408) **across the solenoid coil**, band toward +12 V — without
  it the collapsing coil spikes hundreds of volts back into your MOSFET and Pi
- a big capacitor (1000 µF, 25 V) across the 12 V rail to absorb the current slam —
  it's an electrolytic, so mind the polarity stripe
- **a 5 A inline fuse on the +12 V solenoid branch.** A MOSFET that fails *shorted*
  leaves the coil permanently energised, and a coil rated for 60 ms pulses will cook.
  This is the fire protection, not a nicety.
- **18–20 AWG wire for the solenoid leg.** It carries several amps; thin jumper wire
  heats up and drops enough voltage to weaken the kick.
- **Perfboard and soldered joints, or screw terminals. Do NOT breadboard this.**
  Breadboard contacts are rated for a few hundred milliamps — at solenoid current they
  heat, arc, and melt the board.

The solenoid taps the **raw battery** like the motor drivers, not the buck converter.

```
battery + ──[5 A fuse]──┬────────────┬──────────┐
                        │            │          │
                    [1000 µF]     [coil]     [diode]  band ^ to +12 V
                        │            │          │
                        │            +----------+
                        │            │
                        │          drain
   GPIO 27 ──[220 ohm]────────────- gate   IRLZ44N
                        │       │  source
                     [10 kohm]--+    │
                        │            │
battery - ──────────────+------------+---- common with Pi GND
```

Pick a **12 V push-pull solenoid rated for intermittent duty**, not continuous. Check
its stroke (~10 mm is typical) and remember the whole assembly lives inside the 70 mm
forward allowance and the 2.50 kg limit. On a 2S pack (7.4 V) a 12 V solenoid kicks
weakly and no wiring fixes that — you want 3S.

`bot.py` fires it for 60 ms with a 2.2 s cooldown, which respects rule 5.4.

**Start button.** Any momentary push button, one side GPIO 4, other side GND. Mount it
where you can reach it without leaning over the field, and where you won't hit it by
accident — a second press stops the robot.

*Check it* — run `python3 bot.py`. It waits at "point the robot at the ENEMY goal,
then press start" and only moves once you press. If it starts on its own, the button
is wired to 3.3 V instead of GND.

**Camera.** *Check it* — `python3 bot.py --check` prints the frame size and a contrast
number. `NO FRAME` means the camera isn't detected at all; `BLIND` means it sees
something but with almost no contrast, which is a lens cap, a dark room, or a ribbon
cable in the wrong way round.

## Smoother motors (optional)

`gpiozero` defaults to software PWM, which jitters a little and makes the four wheels
slightly inconsistent. For hardware-timed PWM on every pin:

```bash
sudo apt install pigpio && sudo systemctl enable --now pigpiod
```

Then add `Environment=GPIOZERO_PIN_FACTORY=pigpio` to `ballbot.service`. Only do this
**after** the robot already drives — if the daemon isn't running, `bot.py` won't start.

## What you actually need

More hardware is not better hardware. You have 2.50 kg and a 320 × 280 × 230 mm box,
and every gadget spends both. A light robot that reaches the ball first beats a heavy
one with a kicker on it.

**Required — you fail inspection without these**
kill switch (4.5), battery in its box (4.5), cables tied down (4.5), colour markers on
two sides (2.3), all four mecanum wheels (4.4), a start button on the robot (2.2).

**Worth it — these decide whether you win**
- *IMU* — you already own the MPU-6050. Without a heading the robot cannot tell your
  goal from theirs and will score own goals (6.1).
- *Plow* — the cheapest performance on the list. Sheet plastic and an afternoon.
- *Buck converter for the Pi* — stops the brownout that ends matches.
- *Grip on the rollers* — mecanum loses traction first on a slick field.

**Optional — skip these with no regret**
- *Distance sensor.* The stuck-detector in `play()` already digs you out of most wall
  grinds. It earns its place in one specific case: if the camera gets blinded, the
  sonar is the only thing that knows a wall is there. Skip it if you're short on time.
- *Wings.* Marginal, and they eat width you may want for the plow.
- *BNO055.* Only once the MPU-6050's drift actually costs you a match.

**Leave it off for your first competition**
- *Solenoid kicker.* It adds weight, current spikes, wiring, and a new failure mode,
  to do something a plow and full throttle already do. `bot.py` carries no kicker
  code at all. Build it for the design prize (12.4) after you can score.

## Shopping list

You already have the base kit (rule 4.1): Pi 4, 4 × mecanum, 4 motors, driver board,
camera, 18650 box.

| Part | Why | Rough ฿ |
|---|---|---|
| Buck converter 5 V 3 A | separate Pi rail — **buy this first** | 60–120 |
| Spare microSD, flashed | corruption from hard power-cuts is failure #2 | 150–250 |
| Spare 18650 pack, charged | you play 4 matches (rule 8.1) | — |
| MPU-6050 | heading — you already own one | ~60 |
| BNO055 | heading that doesn't drift; the real upgrade | 300–500 |
| VL53L0X or HC-SR04 | wall back-off, keeps you legal under 10.1 | 40–150 |
| Rocker kill switch | **required** by 4.5 | ~20 |
| IRLZ44N + 1N5408 + 1000 µF + 5 A fuse | kicker driver, if you build one | ~100 |
| 3 mm PVC or aluminium sheet | plow and wings | ~100 |
| Rubber O-rings for rollers | the green field is slippery | ~50 |

Prices vary a lot; treat them as order-of-magnitude.

## Mechanical

**Camera.** Low, behind the plow, tilted forward ~30° — not up on a mast where a
knock, a cloth, or a ceiling light can take it out. Recess the lens in a short hood.
Aim it so the ball is still in frame at about 15 cm; you'll find `close_radius` from
what it reads there.

**Plow.** A shallow curve, 70 mm forward max (rule 4.3), open at the top. It centres
the ball while you drive. It must not become a pocket the ball cannot roll out of —
that's gripping under 5.2 and it stops your wheels.

**Wings.** 40 mm each side, angled outward, fixed and open at the front (4.2, 5.5).
A wing that closes counts as gripping.

**Weight.** 2.50 kg with battery, plow, wings and kicker all fitted, measured in
competition pose. Weigh it early — the plow and a solenoid eat the margin fast, and
inspection (ภาคผนวก ก) checks it before your first match.

**Cables.** Tie everything down. Rule 4.5 wants no cables dragging, and rule 9.4 gives
you one 30-second repair per match — a yanked motor wire spends it.
