# Host

Empty. The other half of [`docs/CONTROL.md`](../docs/CONTROL.md): a web UI
and gamepad on a Steam Deck, laptop or Pi, sending velocity and body pose
over USB serial at 20–50 Hz, and — later — the perception in
[`docs/AI_MODE.md`](../docs/AI_MODE.md).

Nothing here constrains the language; Python is the obvious one, and
`reference/` already has the kinematics in it if the host ever needs to ask
whether a foothold is reachable.

What belongs here:

```
the serial link to the Teensy    speaks the same 24 bytes as protocol.cpp
a web server                     serves the page, holds a WebSocket open
the control page                 sticks, sliders, telemetry
a gamepad reader                 optional; the Deck's sticks are a joystick
```

The browser never talks to the Teensy directly. And give this side its own
deadman: if the browser goes quiet, stop sending, and let the Teensy's
timeout do the rest.
