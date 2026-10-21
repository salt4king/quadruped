"""How much room is there, in millimetres?

check_fit.py answers "do these two parts share material". That is a
yes/no, and by the time it says yes the part is already wrong. This says
how close they come, which is the number you can act on before it does.

It matters most at the LINKAGE PIVOTS. Each one is a shoulder bolt: a
plain 4 mm shank, longer than the rod is thick, so the rod turns on it
while the thread pulls into the arm. That only works if there is a real
gap between the rod's face and the arm's face. Zero overlap is not
enough -- two parts touching over their whole face bind exactly as hard
as a screw done up tight.

Run:  .venv/bin/python cad/check_clearance.py
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from clearance import load_stl, min_distance, transform      # noqa: E402

CAD = Path(__file__).resolve().parent
P = {}
for name, val in re.findall(r"^(\w+)\s*=\s*([-\d.]+)\s*;",
                            (CAD / "params.scad").read_text(), re.M):
    P[name] = float(val)


def roty(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    m[0, 0], m[0, 2] = c, s
    m[2, 0], m[2, 2] = -s, c
    return m


def move(t) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = t
    return m


def along(a_deg: float, L: float) -> np.ndarray:
    a = math.radians(a_deg)
    return np.array([L * math.sin(a), 0.0, -L * math.cos(a)])


def placements(knee: float) -> dict[str, np.ndarray]:
    """Each part's transform in the THIGH's own frame, at this knee angle.

    Mirrors the assembly exactly -- see robot.scad's leg_links().
    """
    crank_q = knee + P["LINK_PHI"]
    tip = np.array([0.0, 0.0, -P["CRANK_C"]]) + along(crank_q, P["LINK_A"])
    return {
        "thigh": np.eye(4),
        "crank": move([0, 0, -P["CRANK_C"]]) @ roty(-crank_q),
        "pushrod": move(tip),
        "shank": move([0, 0, -P["THIGH"]]) @ roty(-knee),
    }


# pair, and what the gap has to be for the design to work
PAIRS = [
    ("pushrod", "crank", 0.30, "shoulder-bolt pivot: the rod must turn"),
    ("pushrod", "shank", 0.30, "shoulder-bolt pivot: the rod must turn"),
    ("pushrod", "thigh", 0.50, "rod sweeps past the thigh"),
    ("crank", "shank", 0.50, "crank disc vs rocker disc"),
    # These two BUTT UP by design: the shank's collar ends on the face of
    # the thigh's knee boss, and the crank's hub ends on the face of its
    # relief. Face contact at a band boundary is what a located joint
    # looks like, and it reads as zero here. What has to be clear on
    # those is the RADIAL gap, which comes from the bore being CLEAR
    # larger than the collar -- check_walls.py covers that.
    ("crank", "thigh", 0.00, "hub in its relief: butts face-to-face"),
    ("shank", "thigh", 0.00, "collar in its bore: butts face-to-face"),
]

KNEE = [15.0, 40.0, 65.0, 85.0, 110.0, 135.0, 150.0]


def main() -> None:
    mesh = {n: load_stl(CAD / "stl" / f"{n}.stl")
            for n in ("thigh", "shank", "crank", "pushrod")}

    print("\nClearance through the knee's travel  (mm, smallest over the sweep)\n")
    print(f"{'pair':<20}{'min gap':>9}{'at knee':>9}{'want':>7}   what it is")
    print("-" * 86)
    bad = []
    for a, b, want, note in PAIRS:
        worst, at = 1e9, 0.0
        for k in KNEE:
            pl = placements(k)
            d = min_distance(transform(mesh[a], pl[a]),
                             transform(mesh[b], pl[b]), cutoff=20.0)
            if d < worst:
                worst, at = d, k
        ok = worst >= want - 1e-9
        if not ok:
            bad.append((a, b, worst, want, note))
        print(f"{a + ' / ' + b:<20}{worst:>9.2f}{at:>9.0f}{want:>7.2f}   "
              f"{note}{'' if ok else '   <-- TIGHT'}")
    print("-" * 86)
    if bad:
        print()
        for a, b, w, want, note in bad:
            print(f"  {a}/{b}: {w:.2f} mm, wanted {want:.2f} -- {note}")
        sys.exit(1)
    print("\nEvery pair keeps its gap across the whole knee travel.")


if __name__ == "__main__":
    main()
