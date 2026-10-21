"""Proof that the joint -> servo mapping is exact and fits real servos.

The knee runs through a parallelogram linkage, so this file has real work to
do: it has to show the mapping is exact everywhere (a general four-bar is
not), that the linkage never approaches a dead point, and that the feet land
in the same places they would without it.

Run:  python3 tests/test_transmission.py
"""

import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.robot import Quadruped, BodyPose, LEG_NAMES
from quadruped.gait import GaitGenerator, GaitParams, Velocity, CRAWL, TROT
from quadruped.leg import JointAngles, Unreachable
from quadruped.transmission import (to_servos, to_joints, TRAVEL,
                                    crank_angle, dead_point_margin,
                                    inertia_saving, LINK_A, LINK_D,
                                    LINK_PHI, KNEE_RANGE)
from quadruped.vec import Vec3

failures: list[str] = []
DT = 1.0 / 200.0


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  pass  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        failures.append(name)


print("\nThe mapping is an exact bijection")
rng = random.Random(11)
worst = 0.0
for _ in range(20000):
    q = JointAngles(rng.uniform(-0.8, 0.8), rng.uniform(-1.6, 0.2), rng.uniform(0.2, 2.6))
    back = to_joints(to_servos(q))
    worst = max(worst, max(abs(a - b) for a, b in zip(q.as_tuple(), back.as_tuple())))
check("joint -> servo -> joint round-trips exactly", worst < 1e-12,
      f"worst {worst:.3e} rad")
print(f"        worst error: {worst:.2e} rad")

# THE PROPERTY THAT MATTERS. A parallelogram's ratio is 1.0 at every
# position; a general four-bar's is not, and that is exactly how the second
# attempt failed -- it looked fine at the standing pose and collapsed to
# 0.12 at the end of the travel. So sample the whole range and differentiate
# numerically rather than trusting the algebra.
worst_ratio, at = None, None
N = 2000
lo, hi = KNEE_RANGE
h = (hi - lo) / N
for i in range(1, N):
    k = lo + i * h
    r = (crank_angle(k + h) - crank_angle(k - h)) / (2 * h)
    if worst_ratio is None or abs(r - 1.0) > abs(worst_ratio - 1.0):
        worst_ratio, at = r, k
check("the linkage ratio is 1.0 across the WHOLE knee range",
      abs(worst_ratio - 1.0) < 1e-9,
      f"worst {worst_ratio:.6f} at knee {math.degrees(at):.1f} deg")
print(f"        worst d(servo)/d(knee) over {math.degrees(hi-lo):.0f} deg: "
      f"{worst_ratio:.9f}")

# It must also NOT be the identity, or there is no linkage and the sketch
# was ignored again.
q = JointAngles(0.0, math.radians(-46.9), math.radians(85.0))
check("the linkage is a real mechanism, not the identity",
      abs(to_servos(q).servo2 - q.knee) > math.radians(5.0),
      f"servo2 {math.degrees(to_servos(q).servo2):+.1f} vs knee "
      f"{math.degrees(q.knee):+.1f} deg")

# A parallelogram has a crossed twin, reachable only through a dead point
# where the crank lines up with CK. The knee's travel has to clear both.
margins = [math.degrees(dead_point_margin(lo + i * (hi - lo) / 200))
           for i in range(201)]
check("the crank never approaches a dead point",
      min(margins) > math.radians(10.0) * 180 / math.pi,
      f"closest approach {min(margins):.1f} deg")
print(f"        dead-point margin: {min(margins):.1f} deg at the worst end "
      f"(crank sweeps {math.degrees(crank_angle(lo)):+.1f} to "
      f"{math.degrees(crank_angle(hi)):+.1f} from the thigh)")

# The rod force follows straight from the crank length, and it is the number
# that decides whether an M3 bolt and a printed rod are enough.
robot_m = Quadruped()
tau_knee = 1.088                       # N.m, worst case, from sizing.py
force = tau_knee / LINK_A
check("rod force stays inside what an M3 joint and a printed rod take",
      force < 120.0, f"{force:.0f} N")
print(f"        rod carries {force:.0f} N at {tau_knee:.2f} N.m of knee torque "
      f"(crank {LINK_A*1000:.0f} mm)")

direct, linked = inertia_saving(0.060, robot_m.config.thigh)
check("moving the servo up the thigh actually buys inertia",
      linked < direct * 0.5,
      f"{linked*1e4:.2f}e-4 vs {direct*1e4:.2f}e-4 kg.m^2")
print(f"        knee servo inertia about the hip: {direct*1e4:.2f}e-4 -> "
      f"{linked*1e4:.2f}e-4 kg.m^2  ({linked/direct:.2f}x)")


print("\nServo travel stays inside a 180 degree servo")
lo = {k: 999.0 for k in ("roll", "servo1", "servo2")}
hi = {k: -999.0 for k in ("roll", "servo1", "servo2")}


def record(angles) -> None:
    for n in LEG_NAMES:
        s = to_servos(angles[n])
        for k, v in zip(("roll", "servo1", "servo2"), s.as_tuple()):
            lo[k] = min(lo[k], v); hi[k] = max(hi[k], v)


cmds = [Velocity(vx=0.06), Velocity(vx=0.10), Velocity(vx=-0.05),
        Velocity(vy=0.05), Velocity(vy=-0.05),
        Velocity(omega=math.radians(30)), Velocity(omega=math.radians(-30)),
        Velocity(vx=0.06, omega=math.radians(20)),
        Velocity(vx=0.10, vy=0.03, omega=math.radians(25))]
for sched in (CRAWL, TROT):
    for cmd in cmds:
        robot = Quadruped(); g = GaitGenerator(robot, sched, GaitParams())
        for _ in range(int(10 / DT)):
            st = g.step(DT, cmd)
            try:
                record(robot.solve(st.pose, st.feet))
            except Unreachable:
                pass

robot = Quadruped(); pose0, feet = robot.stand()
for r in (-20, -10, 0, 10, 20):
    for p in (-20, -10, 0, 10, 20):
        for dz in (-0.05, -0.025, 0, 0.02):
            try:
                record(robot.solve(BodyPose(
                    position=Vec3(0, 0, pose0.position.z + dz),
                    roll=math.radians(r), pitch=math.radians(p)), feet))
            except Unreachable:
                pass

for k in ("roll", "servo1", "servo2"):
    span = math.degrees(hi[k] - lo[k])
    check(f"{k} travel fits a 180 deg servo", span < 180,
          f"needs {span:.1f} deg")
    print(f"        {k:<7} {math.degrees(lo[k]):+7.1f} to {math.degrees(hi[k]):+7.1f}"
          f"  = {span:5.1f} deg  ({180-span:.0f} spare)")

# TRAVEL is the envelope a servo gets SPECIFIED against, so the test is
# containment, not equality: whatever this sweep demands must sit inside
# what was declared. Equality would be the wrong assertion and a brittle
# one -- a sweep that happens to probe slightly less of the envelope would
# fail a spec that is perfectly correct.
for k in ("roll", "servo1", "servo2"):
    d_lo, d_hi = TRAVEL[k]
    check(f"TRAVEL[{k}] contains what the gait demands",
          d_lo <= lo[k] + 1e-9 and hi[k] - 1e-9 <= d_hi,
          f"declared {math.degrees(d_lo):+.1f}..{math.degrees(d_hi):+.1f}, "
          f"demanded {math.degrees(lo[k]):+.1f}..{math.degrees(hi[k]):+.1f}")

# ...and it must not be wildly oversized either, or it stops being useful
# as a purchasing spec.
for k in ("roll", "servo1", "servo2"):
    d_lo, d_hi = TRAVEL[k]
    slack = math.degrees((d_hi - d_lo) - (hi[k] - lo[k]))
    check(f"TRAVEL[{k}] is not wildly oversized", slack < 35.0,
          f"{slack:.1f} deg of slack")


print("\nThe controller is unchanged by the mechanism")
# The IK solves joint angles; the transmission is a layer on the output. The
# feet must land in exactly the same places either way, or the linkage has
# quietly changed the robot's kinematics.
robot = Quadruped()
pose, feet = robot.stand()
angles = robot.solve(pose, feet)
back = robot.feet_from_angles(pose, {n: to_joints(to_servos(angles[n])) for n in LEG_NAMES})
drift = max(feet[n].distance_to(back[n]) for n in LEG_NAMES)
check("feet land identically after a round trip through the transmission",
      drift < 1e-12, f"drift {drift:.3e} m")

print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All transmission tests passed.")
