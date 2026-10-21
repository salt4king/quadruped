"""Proof that the leg IK is correct.

The central claim is that `solve_ik` is a true inverse of `solve_fk`. The way
to test that without re-deriving the same math (and repeating any mistake in
it) is a round trip: pick a foot position, solve for joint angles, push those
angles back through forward kinematics, and check the foot landed where it
was asked to. If the IK had a sign error or a mixed-up link length, the foot
would come back somewhere else.

Run:  python3 tests/test_leg.py
"""

import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.leg import Leg, LegSide, JointAngles, JointLimits, Unreachable
from quadruped.vec import Vec3

TOL = 1e-9
failures: list[str] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    if condition:
        print(f"  pass  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        failures.append(name)


def make_leg(side: LegSide = LegSide.LEFT) -> Leg:
    """A leg with deliberately unequal link lengths.

    Equal thigh and shank would hide a whole class of bug -- swapping l1 and
    l2 changes nothing when they are the same number. Unequal lengths make
    that mistake visible.
    """
    return Leg(
        name=f"test_{side.name.lower()}",
        origin=Vec3(0.10, side.value * 0.05, 0.0),
        side=side,
        l0=0.035,
        l1=0.110,
        l2=0.130,
    )


# --------------------------------------------------------------------------
print("\nFK / IK round trip")
# --------------------------------------------------------------------------

for side in (LegSide.LEFT, LegSide.RIGHT):
    leg = make_leg(side)
    rng = random.Random(12345)
    worst = 0.0
    tested = 0

    # Sweep the joint space rather than the task space: every angle triple
    # inside the limits is reachable by construction, so this generates only
    # valid targets and covers the whole workspace evenly.
    #
    # Poses in the folded-above-hip branch are skipped, because solve_ik
    # deliberately does not return that branch (see the derivation in leg.py,
    # step 2). Their count is printed so the skip stays visible rather than
    # quietly shrinking the test.
    skipped = 0
    for _ in range(20000):
        q = JointAngles(
            rng.uniform(*leg.limits.roll),
            rng.uniform(*leg.limits.hip),
            rng.uniform(*leg.limits.knee),
        )
        if leg.folded_above_hip(q):
            skipped += 1
            continue
        foot = leg.solve_fk(q)
        try:
            q_back = leg.solve_ik(foot)
        except Unreachable as e:
            check(f"{side.name}: IK rejected a pose FK produced", False, str(e))
            break
        foot_back = leg.solve_fk(q_back)
        err = foot.distance_to(foot_back)
        worst = max(worst, err)
        tested += 1

    check(
        f"{side.name}: {tested} random poses round-trip to within 1nm",
        worst < TOL,
        f"worst error {worst:.3e} m",
    )
    print(f"        worst position error: {worst:.2e} m over {tested} poses "
          f"({skipped} skipped as folded-above-hip)")


# --------------------------------------------------------------------------
print("\nJoint angles recovered exactly (not just the foot position)")
# --------------------------------------------------------------------------

# A weaker IK could land the foot correctly via a different arm configuration
# (elbow-up instead of elbow-down). Checking the *angles* match confirms the
# knee-sign convention is applied consistently.
leg = make_leg()
rng = random.Random(999)
worst_angle = 0.0
for _ in range(5000):
    q = JointAngles(
        rng.uniform(*leg.limits.roll),
        rng.uniform(*leg.limits.hip),
        rng.uniform(*leg.limits.knee),
    )
    if leg.folded_above_hip(q):
        continue
    q_back = leg.solve_ik(leg.solve_fk(q))
    worst_angle = max(worst_angle, max(abs(a - b) for a, b in zip(q.as_tuple(), q_back.as_tuple())))

check(
    "recovered angles match the originals (same IK branch chosen)",
    worst_angle < 1e-9,
    f"worst {math.degrees(worst_angle):.3e} deg",
)
print(f"        worst angle error: {math.degrees(worst_angle):.2e} deg")


# --------------------------------------------------------------------------
print("\nTask-space sweep (targets invented independently of the FK)")
# --------------------------------------------------------------------------

# The sweep above generates its targets *with* the forward kinematics, so a
# shared misunderstanding between FK and IK could in principle survive it.
# This one invents foot positions directly in a box under the robot -- no FK
# involved in choosing them -- and checks that whatever solve_ik returns does
# land the foot on the requested spot.

leg = make_leg()
rng = random.Random(4242)
worst = 0.0
reached = 0
rejected = 0
for _ in range(20000):
    target = Vec3(
        leg.origin.x + rng.uniform(-0.15, 0.15),
        leg.origin.y + rng.uniform(-0.15, 0.15),
        leg.origin.z + rng.uniform(-0.26, -0.02),
    )
    try:
        q = leg.solve_ik(target)
    except Unreachable:
        rejected += 1
        continue
    worst = max(worst, target.distance_to(leg.solve_fk(q)))
    reached += 1

check(f"{reached} independently-chosen targets are hit exactly",
      worst < TOL and reached > 1000, f"worst {worst:.3e} m, reached {reached}")
print(f"        worst position error: {worst:.2e} m "
      f"({reached} reachable, {rejected} correctly rejected)")


# --------------------------------------------------------------------------
print("\nKnown poses by hand")
# --------------------------------------------------------------------------

# Straight down: hip and knee at zero. The foot should sit directly under the
# thigh plane, l1+l2 below the hip, offset sideways by l0.
leg = Leg("straight", Vec3(0, 0, 0), LegSide.LEFT, l0=0.03, l1=0.10, l2=0.12,
          limits=JointLimits(knee=(math.radians(-160), 0.0)))
foot = leg.solve_fk(JointAngles(0.0, 0.0, 0.0))
check(
    "zero angles -> foot at (0, +l0, -(l1+l2))",
    abs(foot.x) < TOL and abs(foot.y - 0.03) < TOL and abs(foot.z + 0.22) < TOL,
    f"got {foot}",
)

# A right leg's hip offset must point the other way.
leg_r = Leg("straight_r", Vec3(0, 0, 0), LegSide.RIGHT, l0=0.03, l1=0.10, l2=0.12,
            limits=JointLimits(knee=(math.radians(-160), 0.0)))
foot_r = leg_r.solve_fk(JointAngles(0.0, 0.0, 0.0))
check("right leg's hip offset points -Y", abs(foot_r.y + 0.03) < TOL, f"got {foot_r}")

# 90 degrees of hip pitch with a straight knee lays the leg forward along +X.
foot = leg.solve_fk(JointAngles(0.0, math.radians(90), 0.0))
check(
    "hip=+90deg, knee=0 -> leg points forward (+X)",
    abs(foot.x - 0.22) < TOL and abs(foot.z) < TOL,
    f"got {foot}",
)

# 90 degrees of roll swings the leg from hanging-down to pointing sideways.
# The leg lies along +Y (left), and the hip offset that used to point sideways
# now points up -- so the foot ends up level with the hip plus l0.
foot = leg.solve_fk(JointAngles(math.radians(90), 0.0, 0.0))
check(
    "roll=+90deg -> leg swings out to +Y, foot at hip height + l0",
    abs(foot.y - 0.22) < TOL and abs(foot.z - 0.03) < TOL,
    f"got {foot}",
)


# --------------------------------------------------------------------------
print("\nUnreachable targets are reported, not silently fudged")
# --------------------------------------------------------------------------

leg = make_leg()

too_far = Vec3(leg.origin.x, leg.origin.y, leg.origin.z - 10.0)
try:
    leg.solve_ik(too_far)
    check("a target 10 m away raises Unreachable", False, "no exception")
except Unreachable:
    check("a target 10 m away raises Unreachable", True)

# Directly below the hip axis: inside the cylinder the hip offset sweeps out,
# so no roll angle can reach it however the knee bends.
inside_cylinder = Vec3(leg.origin.x, leg.origin.y, leg.origin.z - 0.01)
try:
    leg.solve_ik(inside_cylinder)
    check("a target inside the hip-offset cylinder raises", False, "no exception")
except Unreachable:
    check("a target inside the hip-offset cylinder raises", True)

# A pose that is geometrically fine but needs a joint past its stop.
extreme = leg.solve_fk(JointAngles(math.radians(44), 0.0, math.radians(-20)))
try:
    leg.solve_ik(extreme)          # inside limits, must pass
    tight = Leg("tight", leg.origin, leg.side, leg.l0, leg.l1, leg.l2,
                limits=JointLimits(roll=(math.radians(-5), math.radians(5))))
    tight.solve_ik(extreme)        # same target, narrow limits, must fail
    check("a joint-limit violation raises", False, "no exception")
except Unreachable:
    check("a joint-limit violation raises", True)

# clamp_to_limits must not raise -- the visualizer depends on that.
tight = Leg("tight", leg.origin, leg.side, leg.l0, leg.l1, leg.l2,
            limits=JointLimits(roll=(math.radians(-5), math.radians(5))))
try:
    q = tight.solve_ik(extreme, clamp_to_limits=True)
    check("clamp_to_limits returns a clamped pose instead of raising",
          abs(q.roll) <= math.radians(5) + 1e-9, f"roll={math.degrees(q.roll):.1f}deg")
except Unreachable as e:
    check("clamp_to_limits returns a clamped pose instead of raising", False, str(e))


# --------------------------------------------------------------------------
print("\nReach envelope")
# --------------------------------------------------------------------------

leg = make_leg()
check("max_reach matches a fully extended leg",
      abs(leg.max_reach - math.hypot(0.240, 0.035)) < TOL,
      f"got {leg.max_reach:.4f}")
check("min_reach is shorter than max_reach", leg.min_reach < leg.max_reach)

# The advertised envelope should agree with what solve_ik actually accepts.
straight_down = Vec3(leg.origin.x, leg.origin.y, leg.origin.z - leg.max_reach * 0.99)
check("a point just inside max_reach is reachable", leg.can_reach(straight_down))
check("a point just outside max_reach is not",
      not leg.can_reach(Vec3(leg.origin.x, leg.origin.y, leg.origin.z - leg.max_reach * 1.01)))


# --------------------------------------------------------------------------
print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All leg kinematics tests passed.")
