# Firmware

Empty scaffolding for the Teensy 4.1. None of it is written yet —
[`docs/CONTROL.md`](../docs/CONTROL.md) is the spec.

```
platformio.ini   build config, Teensy 4.1
include/config.h geometry, joint limits, rates — filled in, from the model
include/*.h      one header per piece, each saying what belongs in it
src/main.cpp     setup() and loop(), with the loop order as comments
src/*.cpp        empty
```

```bash
cd firmware
pio run          # builds as-is, so you can check your toolchain first
pio run -t upload
```

## Where to start

1. **`servos.cpp` alone.** Pin map, PWM at 333 Hz, and move one joint to a
   known angle. Nothing else works until this does, and it's where the
   calibration lives — measure the pulse width at zero per servo rather than
   trusting the datasheet.
2. **`kinematics.cpp`.** Port `reference/quadruped/leg.py`; it has tests you
   can check your numbers against. Stand the robot on a commanded body pose
   before going near a gait.
3. **`protocol.cpp`,** with the deadman from the start, not added later.
4. **`gait.cpp`** last.

`config.h` already holds the link lengths and joint limits the CAD was built
from. Change one there and in `reference/` together, or the model and the
machine stop agreeing.
