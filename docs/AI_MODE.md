# AI mode — later

Not built. This is the room the design leaves for it.

## Where it goes

AI sits on the **host**, above the walking controller, and talks to it
the way a gamepad does — a velocity and a body pose. It never produces
joint angles.

```
  perception ──▶ decide ──▶ (vx, vy, omega, pose) ──▶ Teensy ──▶ legs
```

Keep that boundary and you can swap the whole upper half without
touching firmware that already walks.

## Three steps, cheapest first

**1 — reactive, no camera.** The IMU is already there. Estimate terrain
from how the body actually pitched versus how it was told to; shorten
the next stride after a stumble. Python on the host at 50 Hz, nothing
new to buy.

**2 — see the ground.** A camera on the front bar looking down and
ahead. Classical CV goes a long way: plane fitting for a step edge,
optical flow for odometry. A Pi 5 is enough.

**3 — learned locomotion.** A policy from IMU and joint state to foot
targets, trained in sim. Train it against `reference/`, where the joint
limits and the ±35° roll are already encoded — that is most of the
sim-to-real problem at this size.

## What will bite

**No joint feedback.** Hobby servos take a position and report nothing,
so anything model-based is blind to whether the leg got there. Biggest
constraint on step 3.

**Weight.** 15.3 kg·cm peak against a 25 kg·cm stall. A Pi, camera and
mast is ~250 g and eats into that — re-run the torque check with the new
mass before flying a payload.

**Power.** 7.5 A walking already. Give an SBC its own regulator; twelve
servos starting together drag the servo rail down hard enough to brown
one out.

**Latency.** 30 Hz perception is 33 ms, plus inference, plus the link,
and the gait is committed several steps ahead. Treat perception as
*steering*, not *stepping*.

## Mounting

The roof comes off without tools and the spine has board pads. Bolt a
camera mast to a *second* roof and swap it in — the original goes back
on when you want the robot small again.
