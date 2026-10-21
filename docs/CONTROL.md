# Control plan

No controller is written here. This is what the mechanics were designed
around, so the firmware has something to aim at.

## Two tiers

```
  Steam Deck / laptop / Pi          Teensy 4.1
  ────────────────────────          ──────────────────
  web UI, gamepad                   leg IK
  (later) perception                gait scheduler
  Python                            servo PWM, 12 ch
          │                                  ▲
          └──── velocity + body pose ────────┘
                ~20-50 Hz, WiFi or USB
```

The split is about **deadlines**, not horsepower. Twelve fresh angles
every 5 ms, never late — Linux will not promise that, an MCU will.

The link carries **intent, not angles**: `vx, vy, omega` plus a body
pose, about 24 bytes. Lose the link and the robot keeps walking on its
last command, then stops on a watchdog. That is only safe because the
gait lives on the MCU.

All twelve servos hang off the Teensy's hardware PWM pins — no PCA9685.
Run them at 333 Hz; at 50 Hz the steps are visible in the gait.

## Remote control

The browser never talks to the Teensy.

```
browser ──HTTP/WebSocket──▶ host ──USB serial──▶ Teensy
```

A Steam Deck suits this: normal Linux, gamepad already attached, so one
process can serve the page and read the sticks. Nothing depends on it.

Rough surface:

```
GET  /            the control page
WS   /telemetry   joint angles, IMU, battery, ~20 Hz
POST /cmd         {vx, vy, omega, height, roll, pitch}
POST /stop        stand, then idle
```

**Put a deadman on it.** Teensy stops if the host goes quiet for ~200
ms; the host stops if the browser does.

## Two questions

**Can the Teensy run Python?** Technically yes, MicroPython has a
Teensy 4.1 port. Don't — you lose the Teensyduino libraries and spend
the loop budget on an interpreter. C++ is the supported path.

**Can a Python AI use the C++ IK?** It never needs to. The host sends
*where to go*, the Teensy owns *how to move*. If the host does need the
kinematics — checking a foothold is reachable — use the Python in
`reference/`, or add a "is this reachable?" query to the protocol.

## If you'd rather use one board

A Pi driving a PCA9685 will walk. You give up jitter margin, and the
failure mode: if the host wedges, the servos hold their last command.
Start there if you want walking sooner — the wire protocol is the same
either way.
