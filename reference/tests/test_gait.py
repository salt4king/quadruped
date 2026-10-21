"""Proof that the gait walks, turns, stays upright, and does not scuff.

Run:  python3 tests/test_gait.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.robot import Quadruped, BodyPose, LEG_NAMES
from quadruped.gait import GaitGenerator, GaitParams, Velocity, CRAWL, TROT
from quadruped.leg import Unreachable
from quadruped.vec import Vec3

failures: list[str] = []
DT = 1.0 / 200.0          # 200 Hz, a realistic control rate


def check(name: str, condition: bool, detail: str = "") -> None:
    if condition:
        print(f"  pass  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        failures.append(name)


def _solves(robot, st) -> bool:
    """Does every leg solve at this instant?"""
    try:
        robot.solve(st.pose, st.feet)
        return True
    except Unreachable:
        return False


def run(schedule, cmd: Velocity, seconds: float, params: GaitParams | None = None):
    """Walk for a while and collect every tick."""
    robot = Quadruped()
    gen = GaitGenerator(robot, schedule, params or GaitParams())
    states = []
    for _ in range(int(seconds / DT)):
        states.append(gen.step(DT, cmd))
    return robot, gen, states


# --------------------------------------------------------------------------
print("\nEvery commanded pose is physically solvable")
# --------------------------------------------------------------------------

# The gait is allowed to be slow or ugly, but it is never allowed to ask for
# a foot position the legs cannot reach -- on hardware that means a stalled
# servo. This walks through a range of commands and solves all twelve joints
# at every tick.

commands = [
    ("walk forward",   Velocity(vx=0.06)),
    ("walk backward",  Velocity(vx=-0.05)),
    ("strafe left",    Velocity(vy=0.05)),
    ("turn in place",  Velocity(omega=math.radians(30))),
    ("arc left",       Velocity(vx=0.06, omega=math.radians(20))),
    ("arc right",      Velocity(vx=0.06, omega=math.radians(-20))),
    ("flat out",       Velocity(vx=0.10, vy=0.03, omega=math.radians(25))),
]

for label, cmd in commands:
    robot, gen, states = run(CRAWL, cmd, 6.0)
    bad = None
    for st in states:
        try:
            robot.solve(st.pose, st.feet)
        except Unreachable as e:
            bad = f"t={st.time:.2f}s: {e}"
            break
    check(f"crawl, {label}: all 12 joints solvable for 6 s", bad is None, bad or "")


# --------------------------------------------------------------------------
print("\nThe crawl actually stays upright")
# --------------------------------------------------------------------------

# This is the test the whole gait design exists to pass. A static gait must
# keep the centre of mass inside the support polygon at every single instant.
# Without the body shift this number goes negative -- see below.

robot, gen, states = run(CRAWL, Velocity(vx=0.06), 10.0)
margins = [st.margin for st in states[200:]]     # skip the first cycle's start-up
worst = min(margins)
check("crawl keeps a positive stability margin throughout",
      worst > 0, f"worst margin {worst*1000:.1f} mm")
print(f"        worst margin over 10 s: {worst*1000:+.1f} mm "
      f"(mean {sum(margins)/len(margins)*1000:+.1f} mm)")

# And the control: with the lean switched off, it should fail. If this passes
# too, the body shift is not doing anything and the margin above is luck.
robot2, gen2, states2 = run(CRAWL, Velocity(vx=0.06), 10.0,
                            GaitParams(shift_gain=0.0))
worst_noshift = min(st.margin for st in states2[200:])
check("without the body shift the margin goes negative (the lean is load-bearing)",
      worst_noshift <= 0, f"worst {worst_noshift*1000:+.1f} mm")
print(f"        same gait, shift_gain=0:  {worst_noshift*1000:+.1f} mm")

# A trot on diagonal pairs is never statically stable, by definition.
robot3, gen3, states3 = run(TROT, Velocity(vx=0.10), 6.0)
two_foot = [st for st in states3 if len(st.stance) == 2]
check("trot spends time on two feet, where no static margin exists",
      len(two_foot) > 0 and all(st.margin == float("-inf") for st in two_foot),
      f"{len(two_foot)} two-foot ticks")
print(f"        trot: {len(two_foot)}/{len(states3)} ticks on two feet")


# --------------------------------------------------------------------------
print("\nIt goes where it was told")
# --------------------------------------------------------------------------

# Commanded 0.06 m/s forward for 10 s should cover about 0.6 m. It will not be
# exact -- the body weaves side to side as it leans, and the stride is clamped
# to what the legs can reach -- but the net travel must be close and straight.

# Measured along the path, not start-to-end: the body deliberately weaves
# side to side as it leans, and a straight-line measurement would score that
# weave as an error. (It would also be meaningless for a turning robot, whose
# chord is nothing like its arc.)
def path_length(states) -> float:
    total = 0.0
    for a, b in zip(states, states[1:]):
        total += math.hypot(b.pose.position.x - a.pose.position.x,
                            b.pose.position.y - a.pose.position.y)
    return total


def nominal_travel(gen, states) -> float:
    """Distance along the commanded path, ignoring the lean."""
    return math.hypot(gen._nominal.position.x - states[0].pose.position.x,
                      gen._nominal.position.y - states[0].pose.position.y)


robot, gen, states = run(CRAWL, Velocity(vx=0.06), 10.0)
travelled = nominal_travel(gen, states)
expected = 0.06 * 10.0
check("forward travel matches the command to within 5%",
      abs(travelled - expected) / expected < 0.05,
      f"travelled {travelled:.3f} m, expected {expected:.3f} m")
print(f"        commanded 0.600 m, travelled {travelled:.3f} m")

# REGRESSION. The lean is an offset on top of the nominal path, and it must
# stay bounded. Two separate mistakes made it run away -- folding it back into
# the integrated pose, and planning footsteps from the leaned body so that the
# feet and the lean chased each other. Both looked like mild tuning problems
# (the robot walked, just slower than asked) and neither was. The lean reached
# 136 mm on a 200 mm robot before anything visibly broke.
robot, gen, states = run(CRAWL, Velocity(vx=0.06), 30.0)
leans = []
g = GaitGenerator(Quadruped(), CRAWL, GaitParams())
for _ in range(int(30 / DT)):
    g.step(DT, Velocity(vx=0.06))
    leans.append(g._lean.norm())
early, late = max(leans[:int(5 / DT)]), max(leans[int(25 / DT):])
check("the body lean stays bounded over 30 s (no runaway feedback)",
      late < 0.10 and late < early * 1.5,
      f"peak lean early {early*1000:.1f} mm, late {late*1000:.1f} mm")
print(f"        peak lean: {early*1000:.1f} mm early, {late*1000:.1f} mm after 25 s")

drift = gen._nominal.position.y - states[0].pose.position.y
check("walking forward does not drift sideways",
      abs(drift) < 0.02, f"lateral drift {drift:+.3f} m")

# Turning in place: yaw should accumulate, position should not.
#
# The assertion is NOT "180 degrees were commanded so 180 were delivered".
# A spin spends leg workspace faster than anything else -- the outer feet
# travel omega*r every stance AND the body leans across the support
# triangle at the same time -- so the clamp is entitled to give up yaw
# rate. What it is NOT entitled to do is lie about it. So: the turn must
# go the commanded way, and `command_scale` must account for the whole
# shortfall. A controller that quietly delivered 76% while reporting 1.00
# is the actual bug this guards against.
robot, gen, states = run(CRAWL, Velocity(omega=math.radians(30)), 6.0)
yaw_change = states[-1].pose.yaw - states[0].pose.yaw
check("turning in place turns the commanded way",
      yaw_change > math.radians(45),
      f"turned {math.degrees(yaw_change):.1f} deg in 6 s")
delivered = yaw_change / 6.0
promised = math.radians(30) * gen.command_scale
check("command_scale accounts for the yaw actually delivered",
      abs(delivered - promised) < math.radians(4.0),
      f"delivered {math.degrees(delivered):.1f} deg/s, "
      f"scale {gen.command_scale:.3f} promises {math.degrees(promised):.1f} deg/s")
print(f"        commanded 30 deg/s, scaled to {gen.command_scale:.2f}, "
      f"delivered {math.degrees(delivered):.1f} deg/s "
      f"({math.degrees(yaw_change):.1f} deg in 6 s)")

check("turning in place does not wander off",
      math.hypot(states[-1].pose.position.x, states[-1].pose.position.y) < 0.08,
      f"drifted {math.hypot(states[-1].pose.position.x, states[-1].pose.position.y):.3f} m")

# Turning should make the outside legs take longer strides than the inside.
robot, gen, states = run(CRAWL, Velocity(vx=0.05, omega=math.radians(25)), 8.0)


def stride_length(states, leg: str) -> float:
    """How far this foot travelled per swing, averaged."""
    lengths, lifted = [], None
    for st in states:
        airborne = leg not in st.stance
        if airborne and lifted is None:
            lifted = st.feet[leg]
        elif not airborne and lifted is not None:
            lengths.append(math.hypot(st.feet[leg].x - lifted.x, st.feet[leg].y - lifted.y))
            lifted = None
    return sum(lengths) / len(lengths) if lengths else 0.0


# Turning left (+omega) means the right legs are on the outside.
outside = (stride_length(states, "FR") + stride_length(states, "RR")) / 2
inside = (stride_length(states, "FL") + stride_length(states, "RL")) / 2
check("turning left makes the right (outside) legs stride further",
      outside > inside, f"outside {outside*1000:.1f} mm, inside {inside*1000:.1f} mm")
print(f"        turning left: outside stride {outside*1000:.1f} mm, "
      f"inside {inside*1000:.1f} mm")


# --------------------------------------------------------------------------
print("\nThe feet do not scuff")
# --------------------------------------------------------------------------

# A foot still moving sideways when it lands drags the robot off course. The
# cycloid is supposed to bring horizontal speed to zero at touchdown; this
# measures whether it does, by comparing the foot's speed in its last airborne
# tick against its speed at mid-swing.

robot, gen, states = run(CRAWL, Velocity(vx=0.08), 8.0)
touchdown_speeds, midswing_speeds = [], []
for leg in LEG_NAMES:
    prev = None
    swing: list[Vec3] = []
    for st in states:
        if leg not in st.stance:
            swing.append(st.feet[leg])
        elif swing:
            if len(swing) > 6:
                # horizontal speed over the final tick of the swing
                a, b = swing[-2], swing[-1]
                touchdown_speeds.append(math.hypot(b.x - a.x, b.y - a.y) / DT)
                m = len(swing) // 2
                a, b = swing[m - 1], swing[m]
                midswing_speeds.append(math.hypot(b.x - a.x, b.y - a.y) / DT)
            swing = []

td = max(touchdown_speeds)
mid = sum(midswing_speeds) / len(midswing_speeds)
check("horizontal foot speed at touchdown is near zero",
      td < 0.02, f"{td*1000:.1f} mm/s")
check("...and far below mid-swing speed (the cycloid is doing the work)",
      td < mid * 0.2, f"touchdown {td*1000:.1f} vs mid-swing {mid*1000:.1f} mm/s")
print(f"        touchdown {td*1000:.1f} mm/s vs mid-swing {mid*1000:.1f} mm/s "
      f"({len(touchdown_speeds)} steps)")

# Feet must never go below the floor.
lowest = min(min(st.feet[n].z for n in LEG_NAMES) for st in states)
check("no foot is ever driven below the ground", lowest > -1e-9, f"lowest z {lowest:.6f}")

# And a swinging foot must actually clear the ground.
peak = max(max(st.feet[n].z for n in LEG_NAMES) for st in states)
check("swinging feet lift roughly the commanded step height",
      abs(peak - gen.params.step_height) < 0.005,
      f"peak {peak*1000:.1f} mm, commanded {gen.params.step_height*1000:.1f} mm")


# --------------------------------------------------------------------------
print("\nA stopped robot stands still")
# --------------------------------------------------------------------------

robot, gen, states = run(CRAWL, Velocity(), 3.0)
moved = max(states[0].feet[n].distance_to(states[-1].feet[n]) for n in LEG_NAMES)
check("zero command means the feet do not move", moved < 1e-12, f"moved {moved:.2e} m")
check("zero command means the gait clock does not advance",
      abs(states[-1].phase - states[0].phase) < 1e-12)


# --------------------------------------------------------------------------
print("\nGait schedules are what they claim to be")
# --------------------------------------------------------------------------

check("crawl never has more than one foot in the air",
      all(len(st.stance) >= 3 for st in run(CRAWL, Velocity(vx=0.06), 6.0)[2]))
check("trot lifts diagonal pairs together",
      all(("FL" in st.stance) == ("RR" in st.stance)
          for st in run(TROT, Velocity(vx=0.08), 6.0)[2]))
check("crawl's duty factor leaves 3+ feet down (0.80 > 0.75)",
      CRAWL.duty_factor > 0.75 and CRAWL.statically_stable)
check("trot is correctly flagged as not statically stable",
      not TROT.statically_stable and TROT.duty_factor <= 0.5)


# --------------------------------------------------------------------------
print("\nCommands beyond the robot's reach are scaled, not attempted")
# --------------------------------------------------------------------------

# A crawl leaves each foot planted for 80% of a cycle, so the faster the
# command the further the leg is dragged before it can step again. Past some
# speed it simply runs out of arm. The controller has to notice and give up
# speed -- never stall a servo, and never silently change direction.

robot, gen, states = run(CRAWL, Velocity(vx=0.06), 10.0)
check("a command inside the envelope is delivered in full",
      gen.command_scale == 1.0, f"scale {gen.command_scale:.3f}")

for label, cmd, ceiling in (
    ("0.10 m/s + strafe + 25 deg/s", Velocity(vx=0.10, vy=0.03, omega=math.radians(25)), 1.0),
    ("an absurd 0.5 m/s",            Velocity(vx=0.5, vy=0.3, omega=math.radians(90)), 0.35),
):
    robot, gen, states = run(CRAWL, cmd, 16.0)
    scales = []
    g = GaitGenerator(Quadruped(), CRAWL, GaitParams())
    for _ in range(int(16 / DT)):
        g.step(DT, cmd)
        scales.append(g.command_scale)
    worst_scale = min(scales)
    check(f"{label} is scaled back", worst_scale < ceiling, f"scale {worst_scale:.2f}")
    bad = None
    for st in states:
        try:
            robot.solve(st.pose, st.feet)
        except Unreachable as e:
            bad = str(e)
            break
    check(f"{label} still solves every joint", bad is None, bad or "")
    check(f"{label} stays statically stable",
          min(st.margin for st in states[int(4 / DT):]) > 0,
          f"{min(st.margin for st in states[int(4/DT):])*1000:+.1f} mm")
    print(f"        {label}: scaled to {worst_scale:.2f}")

# Scaling must preserve heading -- giving up speed is fine, wandering is not.
cmd = Velocity(vx=0.10, vy=0.03, omega=math.radians(25))
g = GaitGenerator(Quadruped(), CRAWL, GaitParams())
for _ in range(int(8 / DT)):
    g.step(DT, cmd)
ratio_cmd = cmd.vy / cmd.vx
check("scaling preserves the commanded direction",
      abs(ratio_cmd - (cmd.vy * g.command_scale) / (cmd.vx * g.command_scale)) < 1e-12)


# --------------------------------------------------------------------------
print("\nNo command stalls a servo, on either gait")
# --------------------------------------------------------------------------

# THE REGRESSION GUARD. Three separate bugs all showed up here and nowhere
# else, because each needed a particular KIND of command to expose it:
#
#   the arc        only under yaw. A planted foot traces an arc in the body
#                  frame while the body turns, and the arc bulges outside
#                  the chord between its ends. Straight-line commands were
#                  clean; a 90 deg/s spin put the knee 2 deg past its stop
#                  on one timestep in five.
#
#   the lag        only when swing is long. The footfall was aimed at where
#                  the body was, not where it would be when the foot
#                  landed. A crawl swings for 0.2 of a cycle and absorbed
#                  it; a trot swings for 0.5 and touched down a whole
#                  stance-length behind neutral.
#
#   the lean       only on a crawl, and only while STANDING STILL on that
#                  leg. The foot was planted; the body lurched 17 mm
#                  sideways as the support triangle changed and pulled the
#                  hip out from over it.
#
# So the matrix matters more than any single case. Forward, backward,
# strafe, spin, and the combinations, on both schedules.
CMDS = [
    ("stand",        Velocity()),
    ("walk",         Velocity(vx=0.06)),
    ("brisk",        Velocity(vx=0.15)),
    ("reverse",      Velocity(vx=-0.10)),
    ("strafe",       Velocity(vy=0.05)),
    ("spin",         Velocity(omega=math.radians(30))),
    ("hard spin",    Velocity(omega=math.radians(90))),
    ("walk + turn",  Velocity(vx=0.10, vy=0.03, omega=math.radians(25))),
    ("absurd",       Velocity(vx=0.5, vy=0.3, omega=math.radians(90))),
    ("absurd, other way", Velocity(vx=-0.5, vy=-0.3, omega=math.radians(-90))),
]

SETTLE = 6.0            # seconds allowed for the command ramp to finish


def overrun(robot, st) -> tuple[float, float]:
    """How far outside its envelope the worst leg is at this instant.

    Two numbers, because there are two ways to be outside it and they are
    not comparable: degrees past a joint stop, and millimetres past the
    leg's straight-line reach. A single figure would have to invent an
    exchange rate between them.
    """
    deg = mm = 0.0
    r_inv = st.pose.rotation.inverse()
    for name, leg in robot.legs.items():
        foot = r_inv.apply(st.feet[name] - st.pose.position)
        try:
            q = leg.solve_ik(foot)
        except Unreachable:
            d = foot - leg.origin
            mm = max(mm, (math.sqrt(d.x**2 + d.y**2 + d.z**2)
                          - leg.max_reach) * 1000.0)
            continue
        for angle, (lo, hi) in zip(q.as_tuple(),
                                   (leg.limits.roll, leg.limits.hip, leg.knee_limits)):
            deg = max(deg, math.degrees(max(lo - angle, angle - hi, 0.0)))
    return deg, mm


for sched, sname in ((CRAWL, "crawl"), (TROT, "trot")):
    bad, ramp = [], []
    for label, cmd in CMDS:
        robot, gen, states = run(sched, cmd, 20.0)
        after = [overrun(robot, st) for st in states if st.time > SETTLE]
        before = [overrun(robot, st) for st in states if st.time <= SETTLE]
        s_deg = max((d for d, _ in after), default=0.0)
        s_mm = max((m for _, m in after), default=0.0)
        e_deg = max((d for d, _ in before), default=0.0)
        e_mm = max((m for _, m in before), default=0.0)
        if s_deg > 1e-6 or s_mm > 1e-6:
            bad.append(f"{label} ({s_deg:.2f} deg, {s_mm:.1f} mm)")
        if e_deg > 1e-6 or e_mm > 1e-6:
            ramp.append((label, e_deg, e_mm))
    check(f"{sname}: once settled, no command touches a joint limit",
          not bad, "; ".join(bad))
    if ramp:
        # Stated, not hidden. These are all inside the first few seconds,
        # while the rate limiter is still winding the command up under feet
        # that started square. A real robot would be told to walk, not
        # slammed from standstill to an impossible command, but the number
        # belongs on the record either way.
        w_deg = max(r[1] for r in ramp)
        w_mm = max(r[2] for r in ramp)
        print(f"        during the {SETTLE:.0f} s ramp: "
              + ", ".join(f"{l} ({d:.2f} deg, {m:.1f} mm)" for l, d, m in ramp))
        check(f"{sname}: ramp overruns stay small",
              w_deg < 1.0 and w_mm < 10.0,
              f"worst {w_deg:.2f} deg past a stop, {w_mm:.1f} mm past reach")


# --------------------------------------------------------------------------
print("\nThe lift order is the same-side one")
# --------------------------------------------------------------------------

# Diagonally alternating lifts look better and measure worse: the body has to
# cross its own centre twice per cycle to follow the support triangle, and it
# arrives late every time (worst margin -0.2 mm, i.e. it tips). Stepping down
# one side and then the other gives the body somewhere to stay. This pins the
# ordering so it cannot be "tidied" back.
order = [n for _, n in sorted((round((CRAWL.duty_factor - o) % 1.0, 6), n)
                              for n, o in CRAWL.offsets.items())]
check("crawl lifts RL, FL, RR, FR (down one side, then the other)",
      order == ["RL", "FL", "RR", "FR"], f"got {order}")


def side(n: str) -> str:
    return n[1]


consecutive_same_side = sum(1 for i in range(4) if side(order[i]) == side(order[(i + 1) % 4]))
check("two of the four transitions stay on the same side",
      consecutive_same_side == 2, f"got {consecutive_same_side}")
print(f"        lift order: {' -> '.join(order)}")


print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All gait tests passed.")
