"""Will the screws the build list asks for actually go in?

cad/bom.py works the lengths out from the hip stack. This renders each
screw as a SOLID in the part it goes into and asks the geometry. The
two are allowed to disagree, and once they did: the roll horn was
specified as M3x45 for a hole whose window closes at 40.5 mm, because
the length was being rounded UP to the next stock size. 4.5 mm of that
screw would have been inside the servo's output boss -- it would have
bottomed out and felt tight while the shoulder was still loose.

WHAT THE WINDOW IS. A servo horn is 2 mm thick and it is the only
thread in the joint; the servo's own output boss is immediately behind
it. So for a screw entering at the part's face:

    L >  (face to the horn's near side)   or it never bites
    L <= (face to the horn's far side)    or it bottoms on the servo

Two millimetres wide, and stock lengths come in 4 and 5 mm steps -- so
a counterbore at the entry face is often what moves the window onto a
size you can buy.

THREE THINGS GET CHECKED, per joint:

  shank    the screw's body against the part. Anything here means the
           clearance hole is blocked or missing.
  head     the head against the part, which catches a head sunk into
           material that was never relieved for it.
  bore     the counterbore, which must EXIST -- this one is inverted,
           it has to come back solid or the head has nowhere to sit.

Run:  python3 cad/check_screws.py
"""

from __future__ import annotations

import re
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

CAD = Path(__file__).resolve().parent


def scan(name: str) -> dict[str, float]:
    src = (CAD / name).read_text()
    return {k: float(v) for k, v in
            re.findall(r"^(\w+)\s*=\s*([-\d.]+)\s*;", src, re.M)}


P = scan("params.scad")
SHOULDER = scan("shoulder.scad")

HORN_TH = P["SERVO_HORN_TH"]
STD = [6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50]

# joint, part's entry face, the horn's seat plane, counterbore
JOINTS = [
    ("roll", P["SWEEP_CLR"] + SHOULDER["ARM_W"] + 0.5, 0.0, 0.0,
     "into the shoulder's arm"),
    ("pitch", P["THIGH_IN"], P["PITCH_FACE"], 1.5, "into the thigh"),
    ("knee", P["LINK_IN"], P["KNEE_FACE"], 1.1, "into the crank"),
]


def volume(path: Path) -> float:
    if not path.exists():
        return 0.0
    d = path.read_bytes()
    n = struct.unpack("<I", d[80:84])[0]
    v = 0.0
    for i in range(n):
        o = 84 + i * 50 + 12
        a, b, c = [struct.unpack("<fff", d[o + k * 12:o + k * 12 + 12])
                   for k in range(3)]
        v += (a[0] * (b[1] * c[2] - c[1] * b[2])
              - a[1] * (b[0] * c[2] - c[0] * b[2])
              + a[2] * (b[0] * c[1] - c[0] * b[1])) / 6.0
    return abs(v) / 1000.0


def render(tmp: Path, joint: str, test: str, seat: float,
           length: int, bore: float) -> float:
    out = tmp / f"{joint}_{test}.stl"
    if out.exists():
        out.unlink()
    r = subprocess.run(
        ["openscad", "-o", str(out), "--export-format=binstl",
         "-D", f'JOINT="{joint}"', "-D", f'TEST="{test}"',
         "-D", f"SEAT={seat}", "-D", f"LEN={length}", "-D", f"BORE={bore}",
         str(CAD / "screw_test.scad")],
        capture_output=True, text=True)
    if r.returncode != 0 and "top level object is empty" not in r.stderr.lower():
        raise SystemExit(f"openscad failed on {joint}/{test}:\n{r.stderr[-400:]}")
    return volume(out)


def main() -> None:
    print("\nHorn screws: does the length in the build list actually fit?\n")
    print(f"{'joint':<10}{'window mm':>14}{'spec':>7}{'bite':>7}"
          f"{'shank':>8}{'head':>8}{'seat':>8}   verdict")
    print("-" * 76)
    bad = []
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        for joint, seat, face, bore, note in JOINTS:
            span = abs(face - seat) - bore
            lo, hi = span - HORN_TH, span
            fits = [n for n in STD if lo < n <= hi]
            if not fits:
                print(f"{joint:<10}{f'{lo:.1f}-{hi:.1f}':>14}"
                      f"{'NONE':>7}   no stock length lands in the horn")
                bad.append((joint, "no stock length fits the window"))
                continue
            L = max(fits)
            bite = L - lo

            shank = render(tmp, joint, "shank", seat, L, bore)
            head = render(tmp, joint, "head", seat, L, bore)
            # inverted: the counterbore must be material-free, i.e. cut
            seat_ok = (render(tmp, joint, "bore", seat, L, bore) < 0.001
                       if bore > 0 else True)

            ok = shank < 0.001 and head < 0.001 and seat_ok
            if not ok:
                bad.append((joint, f"shank {shank:.3f}, head {head:.3f}, "
                                   f"counterbore {'cut' if seat_ok else 'MISSING'}"))
            print(f"{joint:<10}{f'{lo:.1f}-{hi:.1f}':>14}{f'M3x{L}':>7}"
                  f"{bite:>7.1f}{shank:>8.3f}{head:>8.3f}"
                  f"{'ok' if seat_ok else 'MISS':>8}   "
                  f"{'ok' if ok else 'PROBLEM'}  {note}")
    print("-" * 76)
    if bad:
        print()
        for joint, why in bad:
            print(f"  {joint}: {why}")
        sys.exit(1)
    print("\nEvery horn screw clears its hole, seats on its face, and lands")
    print(f"inside the horn's {HORN_TH:.1f} mm without reaching the servo.")


if __name__ == "__main__":
    main()
