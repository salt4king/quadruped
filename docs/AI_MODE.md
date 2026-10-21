# AI mode — future work

Not built. This is the shape the rest of the design leaves room for, so
that adding it later is not a rebuild.

## Where it sits

AI goes on the **host**, above the walking controller, and talks to it
in the same language a gamepad does: a velocity and a body pose. It
never produces joint angles.

```
  perception ──▶ decide ──▶ (vx, vy, omega, pose) ──▶ Teensy ──▶ legs
```

That boundary is the thing worth protecting. Keep it and you can
replace the whole upper half — classical CV, a planner, a learned
policy — without touching firmware that is already walking. Break it
and every experiment risks the robot's ability to stand up.

## What the hardware already allows

| | |
|---|---|
| spare on the Teensy | 600 MHz M7, the gait loop uses a fraction |
| link | USB serial to the host, already carrying commands |
| sensing today | IMU only (BNO055) |
| power | 7.5 A walking from a ~10 A budget |
| mounting | roof clips off; the spine has board pads |

The roof is the mounting surface. It comes off without tools, so a
camera mast or a LiDAR puck bolts to a *second* roof and the original
goes back on when you want the robot small again.

## Three rungs, cheapest first

**1 — reactive, no camera.** The IMU is already there. Terrain
estimation from how the body actually pitched versus how it was
commanded to; a stumble reflex that shortens the next stride. Runs on
the host in Python at 50 Hz, needs nothing new.

**2 — see the ground.** A camera on the front bar looking down and
ahead. Classical CV goes a long way here: plane fitting for a step
edge, optical flow for odometry. A Pi 5 is enough. This is where the
two-tier split starts paying for itself, because a 30 Hz perception
loop and a 200 Hz servo loop stop fighting.

**3 — learned locomotion.** A policy mapping IMU and joint state to
foot targets, trained in sim and transferred. Needs the joint-state
telemetry to be honest — servo *commands* are not servo *positions*,
and a DS3225 will not tell you where it actually is. Plan on closing
that loop before trusting a policy: either servos with feedback, or
accept that the model's state is open-loop.

## Things that will bite

**No joint feedback.** Hobby servos take a position and report
nothing. Anything model-based is running blind on whether the leg got
there. This is the single biggest constraint on rung 3.

**Weight.** The robot is 2.06 kg with a peak of 15.3 kg·cm against a
25 kg·cm stall — about 1.6×. A Pi, a camera and a mast is maybe 250 g,
which eats a chunk of that. Re-run `tests/test_torque.py` with the new
mass before flying a payload; it is one constant in
`reference/robot.py`.

**Power.** 7.5 A walking. A Jetson under load is another 2–3 A and it
wants clean 5 V, not the servo rail — twelve servos starting together
drag that rail down hard enough to brown out an SBC. Separate regulator.

**Latency budget.** A 30 Hz perception loop is 33 ms, plus inference,
plus the link. The gait is already committed several steps ahead. Treat
perception as *steering*, not *stepping*: it changes where the robot is
going, not where the next foot lands, until the foothold planner exists.

## Simulation first

The kinematics and gait in `reference/` run without hardware. That is
the place to try a policy, because the mechanical envelope is already
encoded: joint limits, the ±35° roll, the reach the gait will not
exceed. A policy trained against those limits produces commands the
real robot can actually execute, which is most of the sim-to-real
problem for a machine this size.
