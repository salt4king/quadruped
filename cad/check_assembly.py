"""Can this robot actually be put together, and wired?

THE QUESTION check_fit.py DOES NOT ASK. It proves no two parts share
material in the finished pose. That is necessary and it is not enough: a
pocket the exact shape of a servo, with a 3 mm wall on all six faces,
passes an interference check perfectly and cannot be assembled, because
the servo has no way to reach the hole. All three housings on this robot
were like that until this file existed.

Two things are checked per servo, and both are the same kind of question:
is there EMPTY SPACE where something has to move or sit.

  slide    the volume the servo sweeps through on its way into the
           bracket, from outside to seated. Any bracket material in that
           volume is material you would have to cut away with the servo
           already in your hand.

  wire     the cable leaves the far end of the body. A bracket closed
           across that corner traps it.

  driver   a path straight up each screw axis. A mount you cannot reach
           with a driver is a mount you cannot do up.

  pad      the opposite question -- material AROUND each insert pilot.
           Four holes in thin air hold nothing, and the slide channel is
           cut right beside them, so this is the one that catches an
           over-greedy clearance cut.

Run:  python3 cad/check_assembly.py
"""

from __future__ import annotations

import struct
import subprocess
import sys
import tempfile
from pathlib import Path

CAD = Path(__file__).resolve().parent

# part file, the part's PART/variant flags, and the servo to test in it
CASES = [
    ("knee servo in the thigh", "leg.scad", ['PART="thigh"'], "knee"),
    ("pitch servo in the shoulder", "shoulder.scad", ["ARM=1"], "pitch"),
    ("roll servo in the chassis", "chassis.scad", ["SIDE=1"], "roll"),
]

# How much of the swept volume we tolerate inside the bracket. Not zero:
# the sweep is cut with CLEAR all round and the pocket is cut to the same
# clearance, so the two agree to within meshing error.
BUDGET = 0.02      # cm^3

# Four M3 heat-set pads, 10 mm across with a 4 mm pilot, 8 mm deep:
#   4 * pi/4 * (10^2 - 4^2) * 8  =  2.11 cm^3 if every pad is complete.
# Allow for a pad clipped by a nearby feature, but not for a missing one.
PAD_WANT = 1.60    # cm^3


def volume(path: Path) -> float:
    if not path.exists():
        return 0.0
    d = path.read_bytes()
    if len(d) < 84:
        return 0.0
    n = struct.unpack("<I", d[80:84])[0]
    v = 0.0
    for i in range(n):
        o = 84 + i * 50 + 12
        a, b, c = [struct.unpack("<fff", d[o + k * 12:o + k * 12 + 12]) for k in range(3)]
        v += (a[0] * (b[1] * c[2] - c[1] * b[2])
              - a[1] * (b[0] * c[2] - c[0] * b[2])
              + a[2] * (b[0] * c[1] - c[0] * b[1])) / 6.0
    return abs(v) / 1000.0


def measure(name: str, src: str, flags: list[str], servo: str, what: str,
            tmp: Path) -> float:
    """Volume of bracket material inside the servo's slide channel / wire slot."""
    out = tmp / f"{name}_{what}.stl".replace(" ", "_")
    args = ["openscad", "-o", str(out), "--export-format=binstl",
            "-D", f'ASSEMBLY_TEST="{what}"', "-D", f'ASSEMBLY_SERVO="{servo}"']
    for f in flags:
        args += ["-D", f]
    r = subprocess.run(args + [str(CAD / "assembly_test.scad")],
                       capture_output=True, text=True)
    if r.returncode != 0 and "top level object is empty" not in r.stderr.lower():
        raise SystemExit(f"openscad failed on {name}/{what}:\n{r.stderr[:600]}")
    return volume(out)


def main() -> None:
    tmp = Path(tempfile.mkdtemp())
    print("\nCan it be assembled, wired and screwed down?  (cm3)\n")
    print(f"{'':<34}{'------ must be empty ------':>42}{'must be solid':>16}")
    print(f"{'servo':<34}{'slide in':>10}{'cable':>10}{'driver':>10}{'filler':>12}"
          f"{'screw pads':>16}   verdict")
    print("-" * 98)
    bad = []
    for name, src, flags, servo in CASES:
        f = flags + [f'SRC="{src}"']
        slide = measure(name, src, f, servo, "slide", tmp)
        wire = measure(name, src, f, servo, "wire", tmp)
        driver = measure(name, src, f, servo, "driver", tmp)
        pad = measure(name, src, f, servo, "pad", tmp)
        cap = measure(name, src, f, servo, "cap", tmp)
        probs = []
        if slide > BUDGET:
            probs.append(f"{slide:.2f} cm3 blocking the servo")
        if wire > BUDGET:
            probs.append(f"{wire:.2f} cm3 closing the cable exit")
        if driver > BUDGET:
            probs.append(f"{driver:.2f} cm3 in the driver's way")
        if pad < PAD_WANT:
            probs.append(f"only {pad:.2f} cm3 of screw pad (want {PAD_WANT})")
        if cap > BUDGET:
            probs.append(f"{cap:.2f} cm3 where the retainer cap has to sit")
        if probs:
            bad.append((name, probs))
        print(f"{name:<34}{slide:>10.3f}{wire:>10.3f}{driver:>10.3f}{cap:>12.3f}"
              f"{pad:>16.3f}   {'ok' if not probs else 'FAIL'}")
    print("-" * 98)
    if bad:
        print(f"\n{len(bad)} servo mount(s) not buildable:")
        for n, probs in bad:
            for pr in probs:
                print(f"  {n}: {pr}")
        sys.exit(1)
    print("\nEvery servo slides in, takes its filler and screws down on four")
    print("M3 inserts, and its lead has somewhere to go.")


if __name__ == "__main__":
    main()
