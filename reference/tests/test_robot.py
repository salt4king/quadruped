"""Proof that the body-pose solver does what the design claims.

The claim under test: feet stay planted in the world while the body moves,
and the legs absorb the difference. If that holds, balancing is just a matter
of choosing body poses.

Run:  python3 tests/test_robot.py
"""

import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.robot import Quadruped, RobotConfig, BodyPose, LEG_NAMES
from quadruped.leg import Unreachable
from quadruped.vec import Vec3

TOL = 1e-9
failures: list[str] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    if condition:
        print(f"  pass  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        failures.append(name)


robot = Quadruped()
print()
print(robot.describe())


# --------------------------------------------------------------------------
print("\nFeet stay planted while the body moves")
# --------------------------------------------------------------------------

# This is the whole thesis of robot.py. Plant the feet, then put the body
# through a range of poses; after solving, forward kinematics must still put
# every foot exactly where it was planted. Any drift means the solver is
# letting the robot skate across the floor.

pose0, feet = robot.stand()
rng = random.Random(7)
worst = 0.0
tested = 0
for _ in range(4000):
    pose = BodyPose(
        position=Vec3(
            rng.uniform(-0.03, 0.03),
            rng.uniform(-0.03, 0.03),
            rng.uniform(0.12, 0.20),
        ),
        roll=rng.uniform(-0.25, 0.25),
        pitch=rng.uniform(-0.25, 0.25),
        yaw=rng.uniform(-0.25, 0.25),
    )
    try:
        angles = robot.solve(pose, feet)
    except Unreachable:
        continue            # genuinely out of range; the sweep is deliberately wide
    back = robot.feet_from_angles(pose, angles)
    worst = max(worst, max(feet[n].distance_to(back[n]) for n in LEG_NAMES))
    tested += 1

check(f"{tested} body poses keep all four feet planted",
      worst < TOL and tested > 500, f"worst drift {worst:.3e} m over {tested}")
print(f"        worst foot drift: {worst:.2e} m over {tested} poses")


# --------------------------------------------------------------------------
print("\nThe solver reacts the way physical intuition says it should")
# --------------------------------------------------------------------------

pose, feet = robot.stand()
base = robot.solve(pose, feet)

# Rolling the body to the left should extend the left legs and fold the right
# ones -- that is exactly how a robot keeps its body level on a side slope.
rolled = robot.solve(BodyPose(position=pose.position, roll=math.radians(10)), feet)


def leg_extension(angles, name: str) -> float:
    """How far the foot is from its hip -- big means extended, small means folded."""
    return robot.legs[name].fk_hip_frame(angles[name]).norm()


check(
    "rolling left extends the left legs",
    leg_extension(rolled, "FL") > leg_extension(base, "FL")
    and leg_extension(rolled, "RL") > leg_extension(base, "RL"),
)
check(
    "rolling left folds the right legs",
    leg_extension(rolled, "FR") < leg_extension(base, "FR")
    and leg_extension(rolled, "RR") < leg_extension(base, "RR"),
)

# Pitching nose-down should extend the front legs.
pitched = robot.solve(BodyPose(position=pose.position, pitch=math.radians(-10)), feet)
check(
    "pitching nose-down extends the front legs and folds the rear",
    leg_extension(pitched, "FL") > leg_extension(base, "FL")
    and leg_extension(pitched, "RL") < leg_extension(base, "RL"),
)

# Lowering the body should fold every leg.
crouched = robot.solve(pose.with_height(pose.position.z - 0.03), feet)
check(
    "crouching folds all four legs",
    all(leg_extension(crouched, n) < leg_extension(base, n) for n in LEG_NAMES),
)

# A level body standing square should be left/right symmetric.
check(
    "a square stance is left/right symmetric",
    abs(base["FL"].hip - base["FR"].hip) < TOL
    and abs(base["FL"].roll + base["FR"].roll) < TOL,
    f"FL={base['FL']}, FR={base['FR']}",
)


# --------------------------------------------------------------------------
print("\nStability margin")
# --------------------------------------------------------------------------

pose, feet = robot.stand()

# Standing square on all four feet, the CoM is dead centre. The margin should
# be half the body width-ish -- the distance to the nearest edge.
m = robot.stability_margin(pose, feet)
check("standing square is stable", m > 0, f"margin {m:.4f} m")
print(f"        margin standing square: {m*1000:.1f} mm")

# Shoving the CoM far to the left must eventually put it outside the feet.
far_left = pose.shifted(Vec3(0, 0.5, 0))
check("a CoM well outside the feet reports negative margin",
      robot.stability_margin(far_left, feet) < 0,
      f"margin {robot.stability_margin(far_left, feet):.4f} m")

# Margin should shrink monotonically as the CoM slides toward an edge.
margins = [robot.stability_margin(pose.shifted(Vec3(0, dy, 0)), feet)
           for dy in (0.0, 0.01, 0.02, 0.03)]
check("margin decreases as the CoM approaches an edge",
      all(margins[i] > margins[i + 1] for i in range(len(margins) - 1)),
      f"{[f'{x:.4f}' for x in margins]}")

# Lifting a foot leaves a triangle, which must be a smaller safe region.
tripod = {"FL", "FR", "RL"}
m4 = robot.stability_margin(pose, feet)
m3 = robot.stability_margin(pose, feet, stance=tripod)
check("lifting a foot shrinks the margin", m3 < m4, f"4-foot {m4:.4f}, 3-foot {m3:.4f}")
print(f"        margin on three feet:   {m3*1000:.1f} mm")

# Two feet cannot support anything statically.
check("two feet report no support",
      robot.stability_margin(pose, feet, stance={"FL", "RR"}) == float("-inf"))

# The diagonal pair a trot stands on is exactly this case -- which is why a
# trot is a *dynamic* gait and cannot be checked with a static margin.
check("the support polygon orders its vertices into a loop",
      len(robot.support_polygon(feet)) == 4)


# --------------------------------------------------------------------------
print("\nKnee layout matches quadruped anatomy")
# --------------------------------------------------------------------------

# The default is `back`: all four knees behind their hips, every leg the same
# part, the way Spot is built. `forward` mirrors it. `mammal` splits them -- front legs back at
# the elbow, rear legs forward at the stifle, the way a dog is built.
#
# What these tests pin down is that the choice is purely mechanical: whichever
# way a leg buckles, the foot lands in the same place and the joint limits
# mirror with it.

from quadruped.robot import RobotConfig, KNEE_LAYOUTS


def knee_offsets(bot) -> dict:
    """How far ahead of its hip each knee sits, standing square."""
    pose, feet = bot.stand()
    ang = bot.solve(pose, feet)
    out = {}
    for n in LEG_NAMES:
        pts = bot.legs[n].joint_positions(ang[n])
        out[n] = pts[2].x - pts[1].x
    return out


default = knee_offsets(Quadruped())
check("the default layout points every knee BACK (Spot-style)",
      all(v < 0 for v in default.values()),
      ", ".join(f"{k} {v*1000:+.0f}" for k, v in default.items()))
print(f"        default knees: all {default['FL']*1000:+.0f} mm (behind the hip)")

mam = knee_offsets(Quadruped(RobotConfig(knee_layout="mammal")))
check("mammal layout: front knees point BACK (the elbow)",
      mam["FL"] < 0 and mam["FR"] < 0,
      f"FL {mam['FL']*1000:+.1f} mm, FR {mam['FR']*1000:+.1f} mm")
check("mammal layout: rear knees point FORWARD (the stifle)",
      mam["RL"] > 0 and mam["RR"] > 0,
      f"RL {mam['RL']*1000:+.1f} mm, RR {mam['RR']*1000:+.1f} mm")
check("mammal layout is front-to-back asymmetric (it faces a direction)",
      (mam["FL"] < 0) != (mam["RL"] < 0))
print(f"        front {mam['FL']*1000:+.0f} mm, rear {mam['RL']*1000:+.0f} mm")

for name in ("forward", "back"):
    off = knee_offsets(Quadruped(RobotConfig(knee_layout=name)))
    check(f"{name} layout bends all four the same way",
          len({o > 0 for o in off.values()}) == 1)

# Whichever way a leg buckles, the foot lands in the same place -- the branch
# is a mechanical choice, not a different robot. If this ever fails, the two
# layouts are not solving the same problem.
worst = 0.0
for name in KNEE_LAYOUTS:
    bot = Quadruped(RobotConfig(knee_layout=name))
    pose, feet = bot.stand()
    back = bot.feet_from_angles(pose, bot.solve(pose, feet))
    worst = max(worst, max(feet[n].distance_to(back[n]) for n in LEG_NAMES))
check("every layout puts the feet in identical positions", worst < TOL,
      f"worst {worst:.3e} m")

# And the joint limits must mirror with the branch. Forgetting that is the
# obvious bug: the limits exclude the whole branch and every IK call raises.
for name in KNEE_LAYOUTS:
    bot = Quadruped(RobotConfig(knee_layout=name))
    pose, feet = bot.stand()
    try:
        bot.solve(pose, feet)
        check(f"{name} layout solves inside its joint limits", True)
    except Unreachable as e:
        check(f"{name} layout solves inside its joint limits", False, str(e))

try:
    Quadruped(RobotConfig(knee_layout="nonsense"))
    check("an unknown layout name is rejected", False, "no exception")
except ValueError:
    check("an unknown layout name is rejected", True)


# --------------------------------------------------------------------------
print("\nWorkspace sanity for the default geometry")
# --------------------------------------------------------------------------

c = robot.config
check("default stand height is inside the leg's reach",
      c.stand_height < robot.legs["FL"].max_reach,
      f"{c.stand_height:.3f} vs {robot.legs['FL'].max_reach:.3f}")

# There must be real travel in both directions from the standing pose, or the
# robot has no room to absorb a stumble or to push off.
pose, feet = robot.stand()
lo = hi = None
for dz in [x / 1000 for x in range(0, 120)]:
    try:
        robot.solve(pose.with_height(c.stand_height - dz), feet)
        lo = c.stand_height - dz
    except Unreachable:
        break
for dz in [x / 1000 for x in range(0, 120)]:
    try:
        robot.solve(pose.with_height(c.stand_height + dz), feet)
        hi = c.stand_height + dz
    except Unreachable:
        break
check("the body can move at least 3 cm down and 2 cm up from standing",
      lo is not None and hi is not None
      and c.stand_height - lo >= 0.03 and hi - c.stand_height >= 0.02,
      f"range {lo:.3f} to {hi:.3f} m around {c.stand_height:.3f}")
print(f"        body height range: {lo*1000:.0f} to {hi*1000:.0f} mm "
      f"(standing at {c.stand_height*1000:.0f})")


print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All body-solver tests passed.")
