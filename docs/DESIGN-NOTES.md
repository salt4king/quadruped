# Design notes

Written while the control model was being built, before any hardware
existed. Kept because the reasoning is still the reasoning; the paths
have been updated, and anything it says about the hardware not existing
has been overtaken by `cad/`.

---

A balancing, walking quadruped robot. The control core is here and working in
simulation; the hardware does not exist yet. This file is the plan for getting
from one to the other.

```
python3 reference/tests/test_leg.py      # the IK, verified to machine precision
python3 reference/tests/test_robot.py    # the body-pose solver
python3 reference/tests/test_gait.py     # walking, turning, stability
python3 reference/sizing.py              # what servos and battery the geometry needs
python3 reference/export_motion.py       # regenerate the 3D viewer's motion data
```

No dependencies. Stdlib only, which means it runs on a Raspberry Pi with
nothing to install.

---

## The leg: three servos, two axes

Each leg has three joints, twelve on the robot:

| joint | rotates about | moves the foot |
|---|---|---|
| hip roll | body **X** (forward) | sideways — this is the balance axis |
| hip pitch | body **Y** (sideways) | fore/aft |
| knee | body **Y** — *the same axis* | in/out along the leg |

Hip pitch and knee share an axis, so they form a flat two-link arm in one
plane; only the hip roll is perpendicular to it. That is a mechanical fact
worth designing around — two servos mount parallel on the leg itself, one
mounts crosswise at the body — and it is why the IK has a closed form at all:
solve the sideways problem first, then a textbook planar arm. No iteration, no
Jacobian, no convergence failures.

## The one idea worth understanding first

A standing robot's feet are on the ground, and the ground does not move. So
you never command a joint angle. You say **where the body should be**, and the
inverse kinematics works out all twelve angles to hold the feet where they
already are.

Every behaviour is then the same solver with different inputs:

| you want | you say |
|---|---|
| balance | hold the body level while the ground tilts |
| crouch | lower the body, feet unchanged |
| lean | shift the body over the feet that are down |
| walk | move the body forward; periodically pick up a foot and put it down ahead |

That is why `robot.py` is short. It is one coordinate change —
`p_body = R^T (p_world - t)` — wrapped in a loop over four legs.

---

## What is built

| file | what it does |
|---|---|
| `quadruped/vec.py` | 3D vectors and rotations |
| `quadruped/leg.py` | one leg: closed-form IK and FK, joint limits, reach envelope |
| `quadruped/robot.py` | four legs, body pose, support polygon, stability margin |
| `quadruped/gait.py` | crawl and trot gaits, footstep planning, the body lean |
| `sizing.py` | servo torque, power and control rates from the geometry |
| `export_motion.py` | runs the controller and writes motion data for the viewer |

The IK is exact — verified to 1.4×10⁻¹⁶ m over ~38,000 poses, including a
sweep whose targets are invented independently of the forward kinematics, so
a shared misunderstanding between the two could not hide in it.

### Four things the simulation taught that are not obvious

**A quadruped standing square has exactly zero stability margin the instant
it lifts a foot.** The remaining three feet make a triangle whose edge passes
straight through the centre of mass. Dead on the tipping point. So a robot
cannot simply lift a foot and step — it has to shift its weight over the
support triangle first. This is the fact that shapes the entire gait.

**The lift order matters more than any tuning parameter.** Alternating
diagonally (RL, FR, RR, FL) looks best — it never lifts neighbouring legs in
a row, so each support triangle is as fat as possible. Measured, it tips:
worst margin −0.2 mm at its best tuning, across a sweep of 240 parameter
combinations. Stepping down one side and then the other (RL, FL, RR, FR)
gives +19.1 mm with everything else identical. The reason is that the body
has to *get* to each triangle, and consecutive diagonal lifts put consecutive
targets on opposite corners, so it crosses its own centre twice per cycle and
arrives late every time.

**The swing path has to land with zero horizontal speed.** A foot still
moving sideways when it touches down scuffs, and since it is now carrying
weight the scuff shoves the whole robot. A cycloid is exactly the curve whose
horizontal velocity vanishes at both ends. Measured: 5 mm/s at touchdown
against 771 mm/s at mid-swing.

**There is no such thing as "the robot's top speed" per axis.** Forward,
sideways and turning all spend the same leg's workspace, so each one alone
manages a lot (0.184 m/s, 0.098 m/s, 47 °/s) while a modest combination of
all three is impossible. The controller scales the whole command vector when
that happens, so the robot keeps the commanded *direction* and gives up only
speed.

---

## The plan

### Stage 1 — Decide the geometry, then draw it

The numbers in `RobotConfig` are the ones to commit to, because everything
downstream depends on them: body 200 × 110 mm, thigh 110 mm, shank 130 mm,
hip offset 35 mm, about 1.6 kg.

Change them there first and re-run `sizing.py` and the tests. The tests will
tell you if a change breaks the workspace — `test_robot.py` checks the body
can still move 3 cm down and 2 cm up from standing, which is the travel it
needs to absorb a stumble.

Only then draw it. Free options: Onshape (browser, free for public documents),
or FreeCAD. Print in PETG rather than PLA — PLA creeps under sustained load
and the legs are under load whenever the robot is switched on.

**Decide the knee layout before you draw, because it changes the parts count.**
`RobotConfig.knee_layout` takes:

| layout | front | rear | consequence |
|---|---|---|---|
| `back` *(default)* | back | back | what Spot runs; knee trails, foot leads. **One** bracket design |
| `forward` | forward | forward | the mirror; same saving, but the knee leads |
| `mammal` | back (elbow) | forward (stifle) | how a dog is built; front and rear become **different** parts |

All three reach identical foot positions and measure the same stability margin
to the millimetre, so this is not a performance choice — it shows up in what
the leg *collides with*, not how well it walks.

`back` is the default for the reason Spot uses it: the knee trails the hip, so
the foot is the part reaching out ahead. Nothing sticks forward into a step, a
table leg or a carpet edge, and the shank swings forward under the body to
plant the foot — which is the same direction it then has to push. `mammal`
looks the most animal-like but costs you a second bracket design.

### Stage 2 — Buy the parts

Run `sizing.py` for the derivation. The short version:

| part | spec | why |
|---|---|---|
| 12 × servo | ≥ 20 kg·cm, metal gear | the knee needs 10 kg·cm; buying at the requirement means buying a servo that sags |
| leg brackets | one design, L/R mirrored | all four knees bend the same way, so the legs are interchangeable |
| 3 servos per leg | 12 total | hip roll (about body X), hip pitch and knee (both about body Y) |
| controller | Raspberry Pi 5 | enough compute for vision later |
| servo driver | PCA9685 (16-ch PWM, I²C) | or skip it entirely with serial bus servos |
| IMU | **BNO085**, not MPU6050 | see below |
| battery | 2S LiPo, 20 A+ continuous | 12 servos peak around 10 A |
| BEC / regulator | 5 V 3 A, separate from servo rail | a servo inrush browning out the Pi looks exactly like a software bug |
| 4 × foot switch | microswitch or FSR | tells you the foot actually landed |
| camera | Pi Camera Module 3 | stage 6 |

**On the IMU — get the BNO085.** An MPU6050 gives you raw accelerometer and
gyro, and you write the sensor fusion yourself: a complementary or Kalman
filter, then fight gyro drift and accelerometer noise forever. The BNO085
does fusion on-chip and hands you an orientation directly. It costs about $15
more and removes what is otherwise the single most frustrating part of this
project.

**On foot sensors — not IMUs.** An IMU measures acceleration and rotation,
not load. On a foot it screams at every impact, drifts constantly, and still
cannot tell you *this foot is carrying 40% of the weight*, which is the thing
balance needs. A $1 microswitch answers "am I touching" better than a $10 IMU
does. An FSR (~$8) or a load cell with an HX711 (~$10) gives you force if you
want it later.

### Stage 3 — Make it stand

Not glamorous, and it is where most of the time goes. The task is calibration:
mapping each servo's raw PWM value to the joint angle your code means.

Write a small script that drives one joint at a time to a known angle, and
adjust the per-joint offset until the leg physically matches what the FK
predicts. Twelve joints, twelve offsets, one sign per joint. Store them in a
config file, not in the code.

Get this wrong and nothing downstream works, in ways that look like gait bugs.

### Stage 4 — Balance (the first real milestone)

Everything is in place: the body-pose solver already holds the feet planted
while the body moves. What is missing is `balance.py`, and it is small:

```
read the IMU  ->  body roll and pitch
error = (0, 0) - (roll, pitch)          # you want level
correction = Kp * error + Kd * d(error)/dt
command a BodyPose with that correction, feet unchanged
```

A PD controller. Start with Kd = 0, raise Kp until the robot oscillates, back
off to about half, then add Kd to damp what is left. Test it by tilting a
board under the robot — it should stay level while the board does not.

This is also the stage where you find out whether your servos are fast enough.
If the robot wobbles at a frequency you cannot tune out, the servos are the
limit, not the gains.

### Stage 5 — Walk, then turn

`gait.py` already does both, and turning came free: a foot planted on the
ground moves at `v = v_body + ω × r`, so feeding a yaw rate through the same
equation makes the outside legs take longer strides automatically. There is no
turning code to write. Measured: turning left gives 155 mm outside strides
against 70 mm inside.

Start with the crawl, slowly, tethered, over carpet. Expect the first real
walk to look worse than the simulation — the simulation has perfect servos,
no backlash, and a perfectly flat floor.

When the crawl is solid, try the trot. It will fall over, because a trot
stands on two feet and two feet support nothing statically. That is not a bug
to fix; it is why the trot needs stage 4 running underneath it.

### Stage 6 — Vision

Now the dependencies arrive: OpenCV and numpy. Keep them out of the control
core — vision runs in its own process at ~10 Hz and hands the controller a
`Velocity`, which is the same three numbers a joystick would produce. The
robot cannot tell the difference, and that is the point.

A sensible order:
1. Stream frames, measure the real frame rate before building on it.
2. Find something specific — a coloured ball, an AprilTag. AprilTags give you
   full 6-DOF pose from one marker, which is far more than colour blobs.
3. Turn a detection into a `Velocity`. Follow the tag: turn toward it, walk
   until it fills a target fraction of the frame.
4. Obstacles: depth from a stereo camera, or the cheap trick of watching how
   fast features grow between frames.

### Stage 7 — Actual autonomy

Everything before this is deterministic control, and should stay that way.
Learning belongs on top, choosing *where to go*, not *how to move the joints*.

Reasonable next steps, in increasing order of ambition:
- A behaviour tree over the primitives you have. Not machine learning, and it
  handles most of what people mean by "give it AI".
- A small vision model on-device for object recognition.
- Reinforcement learning for gait. Real, and the way the state of the art
  works — but train in a physics simulator (PyBullet, Isaac), not on hardware,
  and expect the sim-to-real gap to be the hard part.

---

## Where the maths is written down

Each module's docstring derives its own maths rather than pointing elsewhere:

- `leg.py` — the IK derivation, step by step, including the branch point where
  two valid solutions exist and why we take the one we take
- `robot.py` — the body-frame coordinate change, and why it makes balancing
  and walking the same operation
- `gait.py` — the lift order, the cycloid swing path, the body lean, and the
  velocity clamp

The comments include the mistakes. Three separate bugs in this code looked
like mild tuning problems and were structural: a body lean folded back into
the integrated pose (cost a third of the commanded speed), footsteps planned
from the leaned body instead of the nominal path (ran away at 12.5 mm/s), and
a reach check that validated touchdown but not lift-off (stalls a servo, but
only at speed). Each is written up where it happened, because the failure mode
is the useful part.
