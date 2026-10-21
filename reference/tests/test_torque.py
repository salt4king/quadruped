"""Will the servos cope with walking?

sizing.py already answers the standing question: park the robot in a grid
of poses, put a fixed share of its weight on one foot, and read off the
worst joint torque. That is the right first check and it is not this one.

WHAT IT MISSES. Standing, the load share is an assumption -- 25% on four
feet, 50% on three. Walking, the share is a RESULT: it depends on where
the feet actually are at that instant and where the body has leaned to,
and it changes every tick. The foot diagonally opposite a lifted one does
not carry half; it carries whatever the moment balance says it carries,
which at some phases is more.

So this runs the real gait and solves the load at every step.

  stance feet     from the gait schedule, not assumed
  load share      moment balance about the centre of mass: sum of the
                  vertical foot forces is the weight, and their moments
                  about the CoM cancel. Three feet makes that exactly
                  determinate; four is under-determined and takes the
                  minimum-norm solution; a trot's two feet is
                  over-determined, and the residual moment is reported
                  because that is the part the robot has to catch
                  dynamically rather than hold
  torque          tau = J^T . F, the same Jacobian sizing.py uses
  dynamics        a touchdown factor, but only for the moment a foot is
                  actually landing. Applying it to every foot at every
                  instant -- which is what the first version did --
                  inflates the sustained figure by 40% and makes the
                  servo look marginal when it is the impact that is
                  marginal, which is a different problem with a
                  different fix

WHAT THE RATING MEANS. A DS3225 is "25 kg.cm" at stall, at 6.8 V, on a
fresh servo. That is the number it cannot exceed, not the number it can
hold.

Two different failures, so two different numbers:

  stripping    a torque spike past stall. Compare the PEAK, which is a
               landing, against the 25.

  cooking      heating, which goes as current squared and so as torque
               squared. The number that matters is the RMS over a whole
               cycle, not the peak -- a joint at 11 kg.cm for a fifth of
               the cycle and 2 for the rest is not a joint at 11. The
               rule of thumb "continuous = stall/3" assumes a constant
               load and is the wrong comparison for a leg, which spends
               much of its time unloaded in swing.

Run:  python3 tests/test_torque.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.robot import Quadruped, LEG_NAMES
from quadruped.gait import GaitGenerator, GaitParams, Velocity, CRAWL, TROT
from quadruped.leg import Unreachable
from sizing import joint_torques

G = 9.81
DT = 1.0 / 200.0

STALL = 25.0          # kg.cm, DS3225 at 6.8 V
CONTINUOUS = STALL / 3.0
TOUCHDOWN = 1.4       # a foot landing carries more than a foot resting
IMPACT_WINDOW = 0.10  # s; how long that lasts after a foot lands

failures: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  pass  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        failures.append(name)


def share(feet, stance, com) -> tuple[dict[str, float], float]:
    """Vertical force on each stance foot, and the moment it cannot balance.

    Solves  sum f = W,  sum f*(x-cx) = 0,  sum f*(y-cy) = 0  by hand
    rather than with a solver: with at most four unknowns the normal
    equations are small, and keeping it explicit makes the physics
    visible. Negative solutions are clamped -- a foot can push, not pull
    -- and the weight is renormalised over what is left.
    """
    names = sorted(stance)
    if not names:
        return {}, 0.0
    w = 1.0
    rows = [[1.0] * len(names),
            [feet[n].x - com[0] for n in names],
            [feet[n].y - com[1] for n in names]]
    rhs = [w, 0.0, 0.0]

    # least-norm / least-squares via the normal equations of A A^T
    m = len(rows)
    ata = [[sum(rows[i][k] * rows[j][k] for k in range(len(names)))
            for j in range(m)] for i in range(m)]
    lam = _solve(ata, rhs)
    if lam is None:
        f = {n: w / len(names) for n in names}
        return f, 0.0
    f = {names[k]: sum(lam[i] * rows[i][k] for i in range(m))
         for k in range(len(names))}

    if any(v < 0 for v in f.values()):
        pos = {n: max(v, 0.0) for n, v in f.items()}
        tot = sum(pos.values()) or 1.0
        f = {n: v * w / tot for n, v in pos.items()}

    resid = math.hypot(sum(f[n] * (feet[n].x - com[0]) for n in names),
                       sum(f[n] * (feet[n].y - com[1]) for n in names))
    return f, resid


def _solve(a, b):
    """Tiny Gaussian elimination. None if singular."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        p = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[p][col]) < 1e-12:
            return None
        m[col], m[p] = m[p], m[col]
        for r in range(n):
            if r == col:
                continue
            fct = m[r][col] / m[col][col]
            for cc in range(col, n + 1):
                m[r][cc] -= fct * m[col][cc]
    return [m[i][n] / m[i][i] for i in range(n)]


CMDS = [
    ("walk 0.06 m/s", Velocity(vx=0.06)),
    ("walk 0.10 m/s", Velocity(vx=0.10)),
    ("brisk 0.15 m/s", Velocity(vx=0.15)),
    ("reverse 0.10 m/s", Velocity(vx=-0.10)),
    ("strafe 0.05 m/s", Velocity(vy=0.05)),
    ("turn 30 deg/s", Velocity(omega=math.radians(30))),
    ("walk + turn", Velocity(vx=0.10, vy=0.03, omega=math.radians(25))),
]


def run() -> None:
    robot = Quadruped()
    weight = robot.config.mass * G
    print(f"\nWalking torque  --  {robot.config.mass:.2f} kg, {weight:.1f} N\n")
    print(f"{'gait':<7}{'command':<17}{'sustained: roll':>16}{'hip':>7}{'knee':>7}"
          f"{'  peak':>8}{'  w/impact':>11}{'   rms':>8}")
    print("-" * 80)

    overall = [0.0, 0.0, 0.0]
    overall_rms = [0.0, 0.0, 0.0]
    overall_impact = 0.0
    worst_case = ""
    for sched, gname in ((CRAWL, "crawl"), (TROT, "trot")):
        for label, cmd in CMDS:
            r = Quadruped()
            gen = GaitGenerator(r, sched, GaitParams())
            peak = [0.0, 0.0, 0.0]
            sq = [0.0, 0.0, 0.0]         # running sum of torque^2, for RMS
            nsamp = 0
            peak_impact = 0.0
            landed: dict[str, float] = {}
            was = set()
            for _ in range(int(14 / DT)):
                st = gen.step(DT, cmd)
                for n in st.stance - was:
                    landed[n] = st.time
                was = set(st.stance)
                try:
                    angles = r.solve(st.pose, st.feet)
                except Unreachable:
                    continue
                com = (st.pose.position.x, st.pose.position.y)
                frac, _ = share(st.feet, st.stance, com)
                nsamp += 1
                # Every leg contributes to the thermal picture, loaded or
                # not: a swing leg is simply at zero, and that zero is
                # what brings the RMS down.
                for n in LEG_NAMES:
                    tq = joint_torques(r.legs[n], angles[n],
                                       (0.0, 0.0, weight * frac.get(n, 0.0)))
                    for j in range(3):
                        sq[j] += tq[j] ** 2
                for n, fr in frac.items():
                    fresh = st.time - landed.get(n, -9) < IMPACT_WINDOW
                    t = joint_torques(r.legs[n], angles[n],
                                      (0.0, 0.0, weight * fr))
                    mx = max(abs(v) for v in t) * 100 / G
                    if fresh:
                        peak_impact = max(peak_impact, mx * TOUCHDOWN)
                    else:
                        for j in range(3):
                            peak[j] = max(peak[j], abs(t[j]))
            kg = [p * 100 / G for p in peak]
            rms = [math.sqrt(v / max(nsamp * len(LEG_NAMES), 1)) * 100 / G
                   for v in sq]
            w = max(kg)
            overall_rms = [max(overall_rms[j], rms[j]) for j in range(3)]
            peak_impact = max(peak_impact, w)
            if w > max(overall):
                worst_case = f"{gname}, {label}"
            overall = [max(overall[j], kg[j]) for j in range(3)]
            overall_impact = max(overall_impact, peak_impact)
            # Flag on what actually threatens the servo: a landing spike
            # near stall. The sustained column being above "stall/3" is
            # not the right alarm -- that rule is for a constant load.
            flag = "" if peak_impact < STALL * 0.7 else "  <-- near stall"
            print(f"{gname:<7}{label:<17}{kg[0]:>16.1f}{kg[1]:>7.1f}{kg[2]:>7.1f}"
                  f"{w:>8.1f}{peak_impact:>11.1f}{max(rms):>8.1f}{flag}")

    print("-" * 80)
    print(f"{'worst':<24}{overall[0]:>16.1f}{overall[1]:>7.1f}{overall[2]:>7.1f}"
          f"{max(overall):>8.1f}{overall_impact:>11.1f}"
          f"{max(overall_rms):>8.1f}  kg.cm")
    print(f"\nWorst sustained: {worst_case}")
    print(f"DS3225: {STALL:.0f} kg.cm stall, ~{CONTINUOUS:.1f} kg.cm continuous "
          f"(stall/3, the usual thermal rule)")
    print(f"Landing adds {TOUCHDOWN:.1f}x for {IMPACT_WINDOW*1000:.0f} ms a step")
    print(f"\nLanding peak vs stall:        {STALL/overall_impact:.2f}x")
    print(f"RMS vs continuous duty:       {CONTINUOUS/max(overall_rms):.2f}x")

    check("the landing peak stays under the stall rating",
          overall_impact < STALL,
          f"{overall_impact:.1f} of {STALL:.0f} kg.cm")
    check("RMS torque stays under continuous duty -- it will not cook",
          max(overall_rms) < CONTINUOUS,
          f"{max(overall_rms):.1f} kg.cm RMS against {CONTINUOUS:.1f} continuous")
    check("landing peak keeps a 1.5x margin on stall",
          overall_impact * 1.5 < STALL,
          f"{overall_impact*1.5:.1f} vs {STALL:.0f}")


run()
print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("Walking torque is inside what the servos can hold.")
