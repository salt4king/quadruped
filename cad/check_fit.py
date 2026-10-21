"""Pairwise interference check across the assembly, through the sweep.

Overlapping solids render without complaint, so visual inspection cannot
find them. This intersects each pair and measures the volume of whatever
survives. Anything above a rounding threshold is material two parts both
occupy.

AND IT CHECKS ALL FOUR LEGS. It used to check only the front-left one,
"because all the assembly mirrors are the identity there" -- which is true,
and is exactly why that proved nothing about the other three. The shoulder
mirrors fore/aft and the leg links do not, so the rear legs are a genuinely
different assembly. Three real clashes were hiding there, every one of them
some version of "this part mirrors and the part it mates with does not".

AND IT SWEEPS, because the knee is a linkage now. A mechanism that is clear
in the standing pose tells you almost nothing: the crank, the rod and the
rocker move relative to the thigh, and the thigh moves relative to the
shoulder. Checking one pose would miss exactly the clash a linkage is prone
to. So every pair is checked at the corners and middle of the joint ranges
the gait actually commands.

Run:  python3 cad/check_fit.py          (sweep, slow)
      python3 cad/check_fit.py --fast   (standing pose, all four legs)
"""

import struct
import subprocess
import sys
import tempfile
from pathlib import Path

CAD = Path(__file__).resolve().parent

# Pairs that could plausibly touch. Parts at opposite ends of the robot are
# not worth the CGAL time.
PAIRS = [
    ("thigh", "shoulder"), ("thigh", "pitchservo"), ("thigh", "kneeservo"),
    ("thigh", "chassis"), ("shank", "thigh"), ("shank", "kneeservo"),
    ("shank", "shoulder"), ("shoulder", "chassis"), ("shoulder", "rollservo"),
    ("shoulder", "pitchservo"), ("chassis", "rollservo"),
    ("chassis", "pitchservo"), ("chassis", "kneeservo"),
    # the linkage, against everything it moves past
    ("crank", "thigh"), ("crank", "shank"), ("crank", "pitchservo"),
    ("crank", "shoulder"), ("crank", "chassis"),
    ("pushrod", "thigh"), ("pushrod", "shank"), ("pushrod", "kneeservo"),
    ("pushrod", "shoulder"), ("pushrod", "pitchservo"), ("pushrod", "chassis"),
    ("crank", "pushrod"), ("shank", "pushrod"), ("shank", "crank"),
    # the fillers, which are parts like any other and were not being
    # checked at all -- the roll one was sitting inside the shoulder arm
    ("rollfill", "shoulder"), ("rollfill", "chassis"), ("rollfill", "thigh"),
    ("pitchfill", "shoulder"), ("pitchfill", "thigh"), ("pitchfill", "chassis"),
    # The roof sits over the spine. Nothing should ever reach it --
    # which is exactly the sort of belief worth checking, since the
    # thigh swings a long way inboard at full hip extension.
    ("roof", "thigh"), ("roof", "shank"), ("roof", "shoulder"),
    ("roof", "crank"), ("roof", "pushrod"), ("roof", "pitchservo"),
    ("roof", "rollservo"), ("roof", "rollfill"),
    ("kneefill", "thigh"), ("kneefill", "shank"), ("kneefill", "crank"),
    ("kneefill", "pushrod"), ("kneefill", "shoulder"),
]

# A pivot joins two parts, so their pin bosses touch by design. The pair is
# still checked -- it just has a budget rather than zero.
TOUCHING = {("crank", "pushrod"), ("shank", "pushrod")}

# Every pocket in this design is cut with the servo solid plus CLEAR, so
# even a part and the servo it houses intersect to nothing. Keeping the
# set here (empty) documents that "all zeros" is the intended answer and
# not something that was never checked.
EXPECTED_FIT: set[tuple[str, str]] = set()

PARTS = ["chassis", "roof", "shoulder", "rollservo", "pitchservo",
         "kneeservo", "thigh", "shank", "crank", "pushrod",
         "rollfill", "pitchfill", "kneefill"]

# Joint angles to test, in degrees. The ends and the middle of what the
# gait actually commands -- measured, not assumed, by sweeping every gait
# and the whole balance envelope.
KNEE_SWEEP = [15.0, 50.0, 85.0, 120.0, 150.0]
HIP_SWEEP = [-88.0, -47.0, 17.0]
# ROLL WAS NEVER SWEPT. It sat pinned at zero through every check in
# this file, which is why rolling a leg inboard could put the shoulder
# arm into the roll servo and nothing noticed.
ROLL_SWEEP = [-35.0, 0.0, 35.0]
LEGS = [("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1)]


def volume(path: Path) -> float:
    d = path.read_bytes()
    if len(d) < 84:
        return 0.0
    n = struct.unpack("<I", d[80:84])[0]
    if n == 0:
        return 0.0
    v = 0.0
    for i in range(n):
        o = 84 + i * 50 + 12
        a, b, c = [struct.unpack("<fff", d[o + k * 12:o + k * 12 + 12]) for k in range(3)]
        v += (a[0] * (b[1] * c[2] - c[1] * b[2])
              - a[1] * (b[0] * c[2] - c[0] * b[2])
              + a[2] * (b[0] * c[1] - c[0] * b[1])) / 6.0
    return abs(v) / 1000.0        # mm^3 -> cm^3


def render(tmp: Path, name: str, a: str, b: str = "",
           pose: tuple[float, float] | None = None,
           leg: tuple[int, int] = (1, 1)) -> Path:
    """Render one part, or the intersection of two. Raises if OpenSCAD fails."""
    out = tmp / f"{name}.stl"
    # Clear it first. An empty intersection -- the result we want -- makes
    # OpenSCAD exit without writing anything, so a leftover file from the
    # PREVIOUS call gets measured instead. Callers reuse one name per
    # pair across the whole sweep, so a clear pose would read back as
    # whatever the last fouled pose shared. The per-pair maximum came out
    # right anyway (every stale number is a real number from some other
    # pose of the same pair), but the "at" column pointed at the wrong
    # pose, which is the column you debug from.
    if out.exists():
        out.unlink()
    args = ["openscad", "-o", str(out), "--export-format=binstl",
            "-D", f'A="{a}"', "-D", f"SX={leg[0]}", "-D", f"SY={leg[1]}"]
    if pose is not None:
        args += ["-D", f"HIP_Q={pose[0]}", "-D", f"KNEE_Q={pose[1]}",
                 "-D", f"ROLL_Q={pose[2]}"]
    args += ["-D", f'B="{b}"'] if b else ["-D", 'MODE="solo"']
    # CGAL segfaults now and then on these booleans -- a different pair
    # each run, and it renders fine on a retry. Retry a bounded number of
    # times rather than aborting a 20 minute sweep, but never swallow it:
    # a crash that keeps happening still has to be reported, because an
    # unrendered pair reads as 0.000 and would pass.
    for attempt in range(3):
        r = subprocess.run(args + [str(CAD / "interference.scad")],
                           capture_output=True, text=True)
        if r.returncode >= 0:
            break
        if out.exists():
            out.unlink()
    # An empty intersection is the result we WANT, and OpenSCAD reports it
    # as a failure ("top level object is empty", exit 1). Every other
    # non-zero exit is a real problem and must not be read as 0.000 cm3.
    if r.returncode != 0:
        if "top level object is empty" in r.stderr.lower():
            return out                       # may not exist; volume() -> 0.0
        raise SystemExit(
            f"openscad failed on {name} (exit {r.returncode}, 3 attempts).\n"
            f"Without this the check reports 0.000 for everything and calls "
            f"it a pass.\n{r.stderr.strip()[:800]}")
    return out


def main() -> None:
    tmp = Path(tempfile.mkdtemp())

    # An empty intersection and a failed render look identical downstream:
    # both give 0.000 cm3. So prove the renderer works and every part has
    # volume BEFORE trusting a column of zeros.
    print("\nSanity -- each part renders to a solid\n")
    for name in PARTS:
        v = volume(render(tmp, f"solo_{name}", name))
        print(f"  {name:<12}{v:>9.2f} cm3")
        if v < 0.1:
            raise SystemExit(
                f"{name} rendered empty. Every pair involving it would then "
                f"read 0.000 and pass for the wrong reason.")

    fast = "--fast" in sys.argv
    poses = ([(None, "standing")] if fast else
             [((h, k, r), f"hip {h:+.0f} knee {k:+.0f} roll {r:+.0f}")
              for h in HIP_SWEEP for k in KNEE_SWEEP for r in ROLL_SWEEP])
    cases = [(lname, (sx, sy), pose, f"{lname} {plabel}")
             for lname, sx, sy in LEGS for pose, plabel in poses]

    bad = []
    print(f"\nPairwise interference  ({len(LEGS)} legs x "
          f"{len(poses)} pose{'' if len(poses)==1 else 's'} = {len(cases)})\n")
    print(f"{'pair':<24}{'worst cm3':>11}   {'at':<26}verdict")
    print("-" * 76)
    for a, b in PAIRS:
        worst, where = 0.0, ""
        for lname, leg, pose, label in cases:
            out = render(tmp, f"{a}_{b}", a, b, pose, leg)
            v = volume(out) if out.exists() else 0.0
            if v > worst:
                worst, where = v, label
        budget = 0.40 if (a, b) in TOUCHING or (b, a) in TOUCHING else 0.005
        if worst < budget:
            verdict = "clear" if worst < 0.005 else "pinned (expected)"
        else:
            verdict = "INTERFERENCE"; bad.append((a, b, worst, where))
        print(f"{a + ' / ' + b:<24}{worst:>11.3f}   {where:<26}{verdict}")

    print("-" * 76)
    if bad:
        print(f"\n{len(bad)} unexpected interference(s):")
        for a, b, v, where in bad:
            print(f"  {a} and {b} share {v:.3f} cm3 at {where}")
        sys.exit(1)
    print("\nNo unexpected interference, on any leg, anywhere in the sweep.")


if __name__ == "__main__":
    main()
