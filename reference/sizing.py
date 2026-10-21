"""What the hardware actually has to do.

Picking servos by eye is how people end up with a robot that stands fine and
collapses the moment it lifts a foot. The torque a joint sees is not a guess:
it follows from the geometry, the mass, and how the weight is shared between
whichever feet are down.

THE METHOD

A force F pushing up on the foot produces joint torques

    tau = J^T . F

where J is the leg's Jacobian -- how far the foot moves per radian of each
joint. The transpose relationship is the principle of virtual work: whatever
the foot gains in force, the joint pays for in torque, and J is the exchange
rate. A joint far from the foot with a long lever pays the most, which is why
the hip and knee dominate and the roll axis gets off lightly in most poses.

J is computed by finite differences rather than derived by hand. It is called
a few thousand times in a sizing script, never in the control loop, and a
numerical Jacobian cannot disagree with the forward kinematics the way a
hand-derived one can once someone edits a link length.

Run:  python3 sizing.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from quadruped.robot import Quadruped, BodyPose, LEG_NAMES
from quadruped.leg import Leg, JointAngles, Unreachable
from quadruped.vec import Vec3

G = 9.81
EPS = 1e-6


def jacobian(leg: Leg, q: JointAngles) -> list[list[float]]:
    """d(foot position) / d(joint angle), 3x3, by central differences."""
    cols = []
    for i in range(3):
        a = list(q.as_tuple())
        b = list(q.as_tuple())
        a[i] -= EPS
        b[i] += EPS
        pa = leg.fk_hip_frame(JointAngles(*a))
        pb = leg.fk_hip_frame(JointAngles(*b))
        cols.append([(pb[k] - pa[k]) / (2 * EPS) for k in range(3)])
    # cols[j][k] = d(axis k) / d(joint j); we want J[k][j]
    return [[cols[j][k] for j in range(3)] for k in range(3)]


def joint_torques(leg: Leg, q: JointAngles, force: Vec3) -> tuple[float, float, float]:
    """tau = J^T . F, in newton-metres."""
    J = jacobian(leg, q)
    return tuple(sum(J[k][j] * force[k] for k in range(3)) for j in range(3))


def survey(robot: Quadruped, load_fraction: float, label: str) -> None:
    """Worst-case torque per joint over the poses the robot actually uses."""
    c = robot.config
    weight = c.mass * G
    peak = {n: [0.0, 0.0, 0.0] for n in LEG_NAMES}
    worst_pose = {n: None for n in LEG_NAMES}

    heights = [c.stand_height * f for f in (0.75, 0.85, 1.0, 1.1)]
    offsets = [x / 100 for x in range(-8, 9, 2)]        # +/- 80 mm of body shift

    for h in heights:
        for dx in offsets:
            for dy in offsets:
                pose = BodyPose(position=Vec3(dx, dy, h))
                feet = robot.default_stance(BodyPose(position=Vec3(0, 0, h)))
                try:
                    angles = robot.solve(pose, feet)
                except Unreachable:
                    continue
                # Vertical ground reaction on the most-loaded foot.
                F = Vec3(0.0, 0.0, weight * load_fraction)
                for n in LEG_NAMES:
                    t = joint_torques(robot.legs[n], angles[n], F)
                    for j in range(3):
                        if abs(t[j]) > peak[n][j]:
                            peak[n][j] = abs(t[j])
                            worst_pose[n] = (h, dx, dy)

    worst = [max(peak[n][j] for n in LEG_NAMES) for j in range(3)]
    print(f"\n{label}")
    print(f"  (one foot carrying {load_fraction*100:.0f}% of "
          f"{c.mass:.1f} kg = {weight*load_fraction:.1f} N)")
    print(f"    {'joint':<8}{'N.m':>9}{'kg.cm':>10}   servo to buy")
    for j, name in enumerate(("roll", "hip", "knee")):
        nm = worst[j]
        kgcm = nm * 100 / G
        rec = math.ceil(kgcm * 1.5 / 5) * 5      # 50% headroom, round up to 5
        print(f"    {name:<8}{nm:>9.2f}{kgcm:>10.1f}   >= {rec:.0f} kg.cm")
    return worst


def main() -> None:
    robot = Quadruped()
    c = robot.config

    print("=" * 62)
    print("  HARDWARE SIZING")
    print("=" * 62)
    print(robot.describe())

    print("\n" + "-" * 62)
    print("SERVO TORQUE")
    print("-" * 62)
    print("""
Two load cases matter. Standing square, four feet share the weight, so each
carries about a quarter. But a crawl gait spends most of its time on three
feet, and the load is not shared evenly between them -- the foot diagonally
opposite the lifted one takes the largest share. Size for the three-foot case
or the robot will sag every time it takes a step.""")

    survey(robot, 0.25, "STANDING  -- four feet, evenly shared")
    worst3 = survey(robot, 0.50, "WALKING   -- three feet, worst-loaded foot")

    print("\n" + "-" * 62)
    print("WHAT THAT MEANS FOR PARTS")
    print("-" * 62)
    need = max(worst3) * 100 / G
    rec = math.ceil(need * 1.5 / 5) * 5
    print(f"""
The binding joint needs {need:.0f} kg.cm, so specify at least {rec:.0f} kg.cm.

Note what that rules out. The default hobby servo, an MG996R, is rated about
10 kg.cm -- which is to say it lands exactly on the requirement with nothing
left over. A servo run at its rating sags under load, runs hot, and strips
its gears; ratings are measured at stall on a fresh servo at the top of its
voltage range, not sustained mid-gait. This is the single most common way
these builds fail: the robot stands fine on four legs, then sags the first
time it picks one up and puts twice the load on the rest.

Realistic options at this size:
  * 20-25 kg.cm digital servos, metal gear, 6-7.4 V  -- cheapest route,
    around $15-20 each, twelve of them. Position control only: you command an
    angle and hope. No feedback, so the controller never learns it sagged.
  * Serial bus servos (Feetech STS3215, LX-16A)      -- around $20-25 each
    and they read angle, load and temperature BACK. That feedback is what
    turns a foot-contact guess into a measurement, and it daisy-chains on one
    wire instead of twelve.

Prefer the serial bus servos if the budget allows. Not for the torque -- for
the feedback. Half the debugging on a walking robot is finding out what the
legs actually did, and position-only servos cannot tell you.""")

    print("\n" + "-" * 62)
    print("POWER")
    print("-" * 62)
    stall = 2.5          # amps per servo, typical for this class
    active = 4           # servos meaningfully loaded at once during a crawl
    print(f"""
Twelve servos at roughly {stall:.1f} A stall each. They never all stall at
once, but {active} loaded at once during a step is normal:

    continuous    ~{active * stall * 0.4:.0f} A     transient peaks   ~{active * stall:.0f} A

So a 2S LiPo (7.4 V) with a 20 A+ continuous rating, and a BEC or separate
regulator for the Pi. Do NOT power the Pi off the same rail as the servos
without one: a servo inrush browns out the Pi mid-step and you get a robot
that reboots when it walks, which reads as a software bug for a long time.""")

    print("\n" + "-" * 62)
    print("CONTROL RATES")
    print("-" * 62)
    print(f"""
    balance loop     200 Hz    IMU -> body pose. Must be fast; this is the
                               loop that catches a stumble.
    gait clock        50 Hz    footstep planning. Nothing here changes fast.
    vision            10 Hz    plenty for obstacle avoidance at walking pace.

The core solves all twelve joints in well under a millisecond, so 200 Hz on a
Pi 5 is not close to a limit. The real constraint is the servos: hobby servos
accept updates at 50 Hz, serial bus servos at a few hundred. Running the
balance loop faster than the servos can listen buys nothing.""")


if __name__ == "__main__":
    main()
