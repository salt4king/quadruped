# Control architecture

Nothing in this repo implements the controller. This is the plan the
mechanics were designed against, so the firmware has something to aim at.

## Two tiers, and why

```
  Steam Deck / laptop / Pi            Teensy 4.1
  ────────────────────────            ──────────────────────
  web UI, gamepad                     IK, three joints a leg
  route planning                      gait scheduler
  (later) perception, policy          servo PWM, 12 channels
  Python                              C++
          │                                   ▲
          └──── velocity + body pose ─────────┘
                ~20-50 Hz, WiFi or USB
```

The split is not about horsepower. It is about **deadlines**. The servo
loop has to produce a new set of twelve angles every 5 ms and never be
late; Linux will not promise that, and a vision model sharing the CPU
makes it worse. An MCU does nothing else and so is never late.

The link carries **intent, not angles**: a velocity command
(`vx, vy, omega`) and a body pose. Roughly 24 bytes. If the link drops
for a second the robot keeps walking on its last command and then stops
on a watchdog — which is only safe because the gait lives on the MCU
side of the line.

## What goes where

| | Teensy | host |
|---|---|---|
| leg IK | ✅ | reference copy |
| gait scheduler | ✅ | |
| servo PWM | ✅ | |
| balance / IMU fusion | ✅ | |
| web server, UI | | ✅ |
| gamepad | | ✅ |
| path planning | | ✅ |
| perception, learned policy | | ✅ (see `AI_MODE.md`) |

The Teensy has 35 hardware PWM pins, so all twelve servos hang off it
directly. No PCA9685 needed — and its native PWM is finer than 12 bits
over I²C anyway. Run the servos at 333 Hz; the DS3225MG's datasheet
gives 50–330 Hz, and at 50 Hz the quantisation is visible in the gait.

## Why not put everything on the host

You can. A Pi 5 driving a PCA9685 will walk, and plenty of hobby
quadrupeds do exactly that. Two things you give up:

- **jitter** — a late frame is a twitch in every leg at once
- **failure mode** — if the host wedges, the servos hold their last
  command. With the gait on an MCU, a lost link is a clean stop.

The honest counter-argument is that it is one board instead of two and
one language instead of two. If you want walking sooner, start there and
move the loop down later; the wire protocol is the same either way.

## Remote control

The host runs the web server. The browser never talks to the Teensy.

```
browser ──HTTP/WebSocket──▶ host ──USB serial──▶ Teensy
```

A Steam Deck works well for this: it is a normal Linux box with a
gamepad already attached, so the same process can serve the UI and read
the sticks. Nothing about the design depends on it.

Suggested surface — this is a sketch, not an API:

```
GET  /                 the control page
WS   /telemetry        joint angles, IMU, battery, at ~20 Hz
POST /cmd              {vx, vy, omega, height, roll, pitch}
POST /stop             cut to a stand, then idle
```

**Put a deadman on it.** The Teensy should stop if it has not heard
from the host in ~200 ms, and the host should stop if the browser goes
quiet. A quadruped that keeps walking because a WebSocket closed is a
quadruped that walks off a table.

## Can the Teensy run Python?

Technically yes — MicroPython has an `mimxrt` port that lists Teensy
4.0 and 4.1. In practice, don't: you lose Teensyduino's libraries, and
an interpreter running the IK twelve times at 200 Hz is spending its
budget in the wrong place. C++ is the supported path.

## Can a Python AI use the C++ IK?

It does not need to, and that is the point of the split. The host sends
*where to go*; the Teensy owns *how to move the joints*. The Python
side never computes a joint angle.

When the host does need the kinematics — checking a foothold is
reachable before committing to it, say — there are three options, in
increasing order of effort:

1. **Use the Python reference in `reference/`.** It is the same maths
   the C++ will implement, already written and tested. Two
   implementations of one model is a real risk of drift, so keep
   `reference/params.py` as the single source of the link lengths and
   generate both from it.
2. **Ask the Teensy.** Add a query to the protocol: "is this foot
   position reachable?" One implementation, one round trip.
3. **Bind the C++.** `pybind11` over the same source the firmware
   compiles. One implementation, no round trip, a build step.

Start with (1), move to (2) if they disagree.
