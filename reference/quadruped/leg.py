"""One 3-DOF leg: forward and inverse kinematics.

This is the piece everything else stands on. If the IK is wrong, the gait is
wrong, the balancing is wrong, and the robot falls over -- so the derivation
is written out here rather than left as a magic formula.

THE LEG

Three joints, the same layout Spot and the MIT Mini Cheetah use:

    j0  hip roll   -- rotates about the body's X (forward) axis.
                      Swings the whole leg out sideways. This is what lets
                      the robot shift its weight left/right to balance.
    j1  hip pitch  -- rotates about Y (left). Swings the leg fore/aft.
    j2  knee       -- rotates about Y as well, so j1 and j2 together form a
                      flat 2-link arm living in a plane.

Three link parameters:

    l0  hip offset  -- sideways distance from the roll axis out to the plane
                       the thigh swings in. (The width of the hip bracket.)
    l1  thigh
    l2  shank

WHY THE SPLIT MATTERS

j0 alone decides which *plane* the leg operates in; j1 and j2 decide where in
that plane the foot sits. That separation is what makes the IK closed-form:
solve the sideways problem first, then a textbook 2-link arm. No iteration,
no Jacobian, no convergence failures -- it is exact and runs in microseconds,
which is what a 200 Hz balance loop needs.

FORWARD KINEMATICS

Work in the *hip frame*: origin at the roll axis, axes parallel to the body.
Before the roll is applied, in the leg's own plane:

    fx = l1 sin(q1) + l2 sin(q1 + q2)
    fy = side * l0                       (constant -- the hip bracket)
    fz = -l1 cos(q1) - l2 cos(q1 + q2)

`side` is +1 for left legs, -1 for right legs, so l0 always pushes the leg
away from the body. Angle sign conventions:

    q1 = 0 and q2 = 0  ->  leg hangs straight down
    q1 > 0             ->  thigh swings forward (+X)
    q2 < 0             ->  shank folds back under the thigh

Standing, that works out to thigh forward ~47 deg and shank back ~85 deg, so
the KNEE POINTS FORWARD -- the same way a human knee does, not the way a dog's
hock does. Worth being clear about, because it is the joint you will be
bolting a bracket to: the thigh runs down-and-forward from the hip, the shank
runs down-and-back to the foot, and the knee is the forward-most point of the
leg.

Then the roll rotates that point about X by q0 to give the hip-frame foot
position.

INVERSE KINEMATICS -- the derivation

Given a target foot position (x, y, z) in the hip frame:

Step 1: X is untouched by a rotation about X, so immediately  fx = x.

Step 2: The roll is a rotation in the Y-Z plane, and rotations preserve
        length, so  y^2 + z^2  ==  fy^2 + fz^2  ==  l0^2 + fz^2.
        Therefore              fz = +/- sqrt(y^2 + z^2 - l0^2)

        NOTE THE SIGN -- this is a real branch point, not a formality.
        Both roots are genuine solutions that put the foot on the exact same
        spot, using different arm configurations:

            fz < 0   foot hangs below the hip        <-- we take this one
            fz > 0   shank folded back up past the
                     hip, foot tucked above it

        The second one is reachable on paper and useless in practice: a
        walking robot never places a foot above its own hip, and on real
        hardware the folded shank would be somewhere inside the chassis. So
        this solver always takes the negative root, and its image is exactly
        the below-the-hip half of the workspace. `folded_above_hip()` below
        tests which branch a given joint triple belongs to.

        If y^2 + z^2 < l0^2 the target is inside the cylinder the hip offset
        sweeps out, and no roll angle can reach it -- that is unreachable.

Step 3: (y, z) is just (side*l0, fz) rotated by q0. The angle between two
        vectors is the difference of their atan2s, so
                              q0 = atan2(z, y) - atan2(fz, side*l0)

Step 4: What is left is a 2-link arm reaching (fx, fz) in its plane.
        Let r = hypot(fx, fz). Law of cosines on the knee:
                              cos(q2) = (r^2 - l1^2 - l2^2) / (2 l1 l2)
        Sanity check: q2 = 0 gives r = l1 + l2, a straight leg. Correct.
        Two solutions, +/-q2, mirror images about the straight line to the
        foot. We take the negative one, which folds the shank back under the
        thigh and puts the knee forward (see the sign conventions above).
        Taking +q2 instead gives a mirror-image robot that walks just as well
        -- it is a mechanical choice, not a mathematical one -- but it must be
        made consistently, because the two do not mix on one chassis.

Step 5: Aim the thigh. The target sits at angle  a = atan2(fx, -fz)  from
        straight-down. Bending the knee pulls the foot off that line by
                              b = atan2(l2 sin(q2), l1 + l2 cos(q2))
        so the thigh must lead by that much:   q1 = a - b.

Every step is exact. The only failure modes are geometric (out of reach),
and they are reported rather than silently clamped -- see `solve_ik`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .vec import Vec3, clamp


class LegSide(Enum):
    LEFT = 1
    RIGHT = -1


class Unreachable(ValueError):
    """The requested foot position is outside this leg's workspace."""


@dataclass(frozen=True)
class JointLimits:
    """Per-joint travel, in radians. Real servos cannot spin freely, and a
    commanded angle outside these would either stall the servo or drive a
    link through the chassis."""

    roll: tuple[float, float] = (math.radians(-35), math.radians(35))
    hip: tuple[float, float] = (math.radians(-90), math.radians(90))
    knee: tuple[float, float] = (math.radians(-150), math.radians(-15))

    def as_tuple(self) -> tuple[tuple[float, float], ...]:
        return (self.roll, self.hip, self.knee)


@dataclass(frozen=True)
class JointAngles:
    """The three joint angles for one leg, in radians."""

    roll: float
    hip: float
    knee: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.roll, self.hip, self.knee)

    def degrees(self) -> tuple[float, float, float]:
        return tuple(math.degrees(a) for a in self.as_tuple())

    def __repr__(self) -> str:
        r, h, k = self.degrees()
        return f"JointAngles(roll={r:+.1f}deg, hip={h:+.1f}deg, knee={k:+.1f}deg)"


@dataclass(frozen=True)
class Leg:
    """One leg's geometry and its solvers.

    `origin` is where this leg's roll axis sits in the body frame. `side`
    decides which way the hip offset points.
    """

    name: str
    origin: Vec3
    side: LegSide
    l0: float           # hip offset (roll axis -> thigh plane)
    l1: float           # thigh
    l2: float           # shank
    limits: JointLimits = JointLimits()
    # Which of the two IK branches this leg uses -- see `knee_sign` below.
    # True puts the knee ahead of the hip, False puts it behind.
    knee_forward: bool = True

    @property
    def knee_sign(self) -> int:
        """The sign of the knee angle this leg's IK returns.

        Step 4 of the derivation has two solutions, +/-q2, mirror images about
        the straight line from hip to foot. They reach the identical foot
        position; what differs is which way the leg buckles to get there.

            negative q2  ->  shank folds back under the thigh, KNEE FORWARD
            positive q2  ->  shank folds forward,              KNEE BACK

        This is a mechanical choice, not a mathematical one, and on a real
        animal it is not the same choice at both ends -- see `mammal_legs()`
        in robot.py.
        """
        return -1 if self.knee_forward else 1

    # -- reach envelope -----------------------------------------------------

    @property
    def max_reach(self) -> float:
        """Longest straight-line hip-to-foot distance, fully extended."""
        return math.hypot(self.l1 + self.l2, self.l0)

    @property
    def min_reach(self) -> float:
        """Shortest, with the knee folded as far as its limit allows."""
        q2 = max(self.knee_limits, key=abs)
        r = math.sqrt(self.l1**2 + self.l2**2 + 2 * self.l1 * self.l2 * math.cos(q2))
        return math.hypot(r, self.l0)

    # -- forward kinematics -------------------------------------------------

    def solve_fk(self, q: JointAngles) -> Vec3:
        """Joint angles -> foot position, in the *body* frame.

        Used to verify the IK (round-tripping FK(IK(p)) must return p) and to
        draw the robot.
        """
        return self.origin + self.fk_hip_frame(q)

    def fk_hip_frame(self, q: JointAngles) -> Vec3:
        """Joint angles -> foot position relative to this leg's hip."""
        # In the leg's own plane, before the roll.
        fx = self.l1 * math.sin(q.hip) + self.l2 * math.sin(q.hip + q.knee)
        fy = self.side.value * self.l0
        fz = -self.l1 * math.cos(q.hip) - self.l2 * math.cos(q.hip + q.knee)

        # Apply the hip roll: a rotation about X, which mixes Y and Z only.
        c, s = math.cos(q.roll), math.sin(q.roll)
        return Vec3(fx, fy * c - fz * s, fy * s + fz * c)

    def folded_above_hip(self, q: JointAngles) -> bool:
        """True if this joint triple puts the foot above the hip in the leg
        plane -- the IK branch `solve_ik` does not return.

        Worth checking when you invent a pose by hand or read angles back off
        real servos: a pose in this half of the workspace will not survive a
        round trip through `solve_ik`, because the solver will hand back the
        below-the-hip solution for the same foot position instead.
        """
        return -self.l1 * math.cos(q.hip) - self.l2 * math.cos(q.hip + q.knee) > 0.0

    def joint_positions(self, q: JointAngles) -> tuple[Vec3, Vec3, Vec3, Vec3]:
        """Every joint along the leg, in body frame: hip, roll-out, knee, foot.

        Only needed for drawing, but having it here keeps the geometry in one
        place -- a visualizer that re-derives link positions is a visualizer
        that will eventually disagree with the controller.
        """
        c, s = math.cos(q.roll), math.sin(q.roll)

        def roll(p: Vec3) -> Vec3:
            return Vec3(p.x, p.y * c - p.z * s, p.y * s + p.z * c)

        fy = self.side.value * self.l0
        hip = Vec3(0.0, fy, 0.0)
        knee = Vec3(self.l1 * math.sin(q.hip), fy, -self.l1 * math.cos(q.hip))
        foot = Vec3(
            knee.x + self.l2 * math.sin(q.hip + q.knee),
            fy,
            knee.z - self.l2 * math.cos(q.hip + q.knee),
        )
        return (
            self.origin,
            self.origin + roll(hip),
            self.origin + roll(knee),
            self.origin + roll(foot),
        )

    # -- inverse kinematics -------------------------------------------------

    def solve_ik(self, foot_body: Vec3, *, clamp_to_limits: bool = False) -> JointAngles:
        """Foot position (body frame) -> joint angles.

        Raises `Unreachable` if no joint configuration puts the foot there.
        That is deliberate: silently clamping an impossible target makes a
        robot that lurches somewhere unexpected instead of telling you your
        gait is asking for something the legs cannot do. The gait code is
        responsible for staying inside the workspace; this is the check that
        proves it did.

        `clamp_to_limits=True` squeezes the result into the joint limits
        instead of raising. Only for the visualizer, where showing a slightly
        wrong pose beats crashing mid-animation.
        """
        p = foot_body - self.origin          # into the hip frame
        x, y, z = p

        # --- Step 2: solve the sideways problem ---------------------------
        yz_sq = y * y + z * z
        l0_sq = self.l0 * self.l0
        if yz_sq < l0_sq:
            raise Unreachable(
                f"{self.name}: target {foot_body} is inside the hip-offset "
                f"cylinder (need sqrt(y^2+z^2) >= l0={self.l0:.3f}, "
                f"got {math.sqrt(yz_sq):.3f})"
            )
        fz = -math.sqrt(yz_sq - l0_sq)

        # --- Step 3: the roll angle ---------------------------------------
        q_roll = math.atan2(z, y) - math.atan2(fz, self.side.value * self.l0)
        q_roll = _wrap_pi(q_roll)

        # --- Step 4: the knee, by law of cosines --------------------------
        r_sq = x * x + fz * fz
        r = math.sqrt(r_sq)
        if r > self.l1 + self.l2:
            raise Unreachable(
                f"{self.name}: target {foot_body} is {r:.3f} from the hip "
                f"plane origin but the leg only extends to "
                f"{self.l1 + self.l2:.3f}"
            )
        if r < abs(self.l1 - self.l2):
            raise Unreachable(
                f"{self.name}: target {foot_body} is too close in "
                f"({r:.3f} < {abs(self.l1 - self.l2):.3f}); the leg cannot "
                f"fold that tightly"
            )

        cos_knee = (r_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        # The reach checks above guarantee this is in [-1, 1]; the clamp is
        # purely defensive against floating-point drift at the exact limits,
        # where acos() of 1.0000000000000002 would raise a domain error.
        q_knee = self.knee_sign * math.acos(clamp(cos_knee, -1.0, 1.0))

        # --- Step 5: aim the thigh ----------------------------------------
        a = math.atan2(x, -fz)
        b = math.atan2(self.l2 * math.sin(q_knee), self.l1 + self.l2 * math.cos(q_knee))
        q_hip = a - b

        angles = JointAngles(q_roll, q_hip, q_knee)

        if clamp_to_limits:
            lim = self.limits
            return JointAngles(
                clamp(q_roll, *lim.roll),
                clamp(q_hip, *lim.hip),
                clamp(q_knee, *self.knee_limits),
            )

        self.check_limits(angles)
        return angles

    @property
    def knee_limits(self) -> tuple[float, float]:
        """This leg's knee travel, mirrored to match its branch.

        `JointLimits.knee` is written for a knee-forward leg, where the angle
        is negative. A knee-back leg runs through positive angles over exactly
        the same range of bend, so its limits are the negation, swapped back
        into (low, high) order. Forgetting this mirror is the obvious bug: the
        limits silently exclude the entire branch and every single IK call
        raises.
        """
        lo, hi = self.limits.knee
        return (lo, hi) if self.knee_forward else (-hi, -lo)

    def check_limits(self, q: JointAngles) -> None:
        """Raise if any joint is outside its travel."""
        names = ("roll", "hip", "knee")
        limits = (self.limits.roll, self.limits.hip, self.knee_limits)
        for name, angle, (lo, hi) in zip(names, q.as_tuple(), limits):
            if not (lo - 1e-9) <= angle <= (hi + 1e-9):
                raise Unreachable(
                    f"{self.name}: {name} joint would need "
                    f"{math.degrees(angle):+.1f}deg, limit is "
                    f"[{math.degrees(lo):+.1f}, {math.degrees(hi):+.1f}]"
                )

    def can_reach(self, foot_body: Vec3) -> bool:
        """Non-throwing reachability test, for gait planning."""
        try:
            self.solve_ik(foot_body)
            return True
        except Unreachable:
            return False


def _wrap_pi(a: float) -> float:
    """Fold an angle into [-pi, pi]. Keeps the difference-of-atan2s in step 3
    from coming back as e.g. +350deg when it means -10deg."""
    return (a + math.pi) % (2 * math.pi) - math.pi
