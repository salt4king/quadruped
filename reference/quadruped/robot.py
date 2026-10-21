"""The whole robot: four legs, one body, and the pose solver that links them.

THE CENTRAL IDEA -- and it is the thing worth understanding about this project

A standing robot's feet are on the ground. The ground does not move. So when
you want the body to lean, rise, or shift its weight, you do not move the
feet -- you move the *body*, and the legs absorb the difference.

That inverts the usual way of thinking about it. The controller's job is not
"what angle should each servo be at"; it is "where do I want the body, and
where are the feet planted", and the IK turns that into twelve angles. Every
higher-level behaviour then becomes a statement about the body:

    balancing  ->  hold the body level while the ground tilts
    crouching  ->  lower the body, feet unchanged
    leaning    ->  shift the body over the support polygon
    walking    ->  move the body forward, and periodically pick up a foot and
                   put it back down further ahead

All four are the same solver with different inputs. That is why this file is
short.

THE MATH

One coordinate change. A foot's position in the world and in the body frame
are related by the body's pose:

    p_world = R_body . p_body + t_body

so going the other way,

    p_body = R_body^T . (p_world - t_body)

`R_body^T` because the inverse of a rotation is its transpose. Feed p_body to
the leg's IK and you are done. Tilting the body changes R_body, which changes
every p_body, which changes all twelve joint angles -- automatically, with no
special case for "leaning" anywhere in the code.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from .leg import Leg, LegSide, JointAngles, JointLimits, Unreachable
from .vec import Vec3, Rot


@dataclass(frozen=True)
class BodyPose:
    """Where the body is and how it is oriented, in the world frame.

    This is the controller's entire output vocabulary at the top level: six
    numbers that say what the chassis should be doing.
    """

    position: Vec3 = Vec3(0.0, 0.0, 0.18)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @property
    def rotation(self) -> Rot:
        return Rot.from_rpy(self.roll, self.pitch, self.yaw)

    def with_height(self, z: float) -> "BodyPose":
        return replace(self, position=Vec3(self.position.x, self.position.y, z))

    def shifted(self, d: Vec3) -> "BodyPose":
        return replace(self, position=self.position + d)

    def __repr__(self) -> str:
        return (
            f"BodyPose(pos={self.position}, "
            f"rpy=({math.degrees(self.roll):+.1f}, "
            f"{math.degrees(self.pitch):+.1f}, "
            f"{math.degrees(self.yaw):+.1f})deg)"
        )


@dataclass(frozen=True)
class RobotConfig:
    """Every dimension of the robot, in metres and kilograms.

    These are the numbers you will later have to commit to in CAD, so they
    live in one place. The defaults describe a small desk-sized dog roughly
    the scale of a Raspberry-Pi-and-hobby-servo build -- see the README for
    where they come from.
    """

    body_length: float = 0.200      # hip-to-hip, front to back
    body_width: float = 0.110       # hip-to-hip, left to right
    # Roll axis out to the thigh plane (l0).
    #
    # 56 mm, not 35. The pitch servo's body reaches INBOARD from the leg
    # plane, so the leg plane has to stand off the roll axis by more than
    # one servo length (40.4 mm) plus the roll servo's own housing, or the
    # two occupy the same space. At 35 mm they overlapped by 18.4 mm and
    # the assembly simply could not be built.
    #
    # It costs roll torque -- the lever arm grows with it -- and buys a
    # wider, more stable stance. See sizing.py for the current numbers.
    #
    # 46, and the two steps there are worth keeping apart.
    #
    # 40 -> 43: the thigh cleared the roll servo itself by 4.3 mm but
    # its CASING by 0.16 mm at full roll, which is a hit once anything
    # is printed. That clearance is normal to the thigh's plane, so it
    # moves with this almost one for one.
    #
    # 43 -> 46: the knee journal. Its length is in the chain that lands
    # on this number, so the 3 mm it needed to get off 25 MPa of bearing
    # pressure had to come from somewhere -- either from here, or out of
    # the gap that holds the thigh clear of the roll casing. Taking it
    # from the gap would have undone the step above.
    hip_offset: float = 0.0460
    thigh: float = 0.110            # l1
    shank: float = 0.130            # l2
    # Total mass. Not an estimate for the printed share: 718 g is the
    # measured volume of the exported STLs at PETG density and 50% infill
    # -- run cad/bom.py, it prints that number. The rest: 12 servos at
    # ~60 g (720 g), 2S LiPo (340 g), board and wiring (120 g),
    # fasteners (100 g).
    #
    # It was 1.88 when the printed share was 599 g. Everything that made
    # the robot stronger also made it heavier -- the solid hip collars,
    # the wider arm, the knee bracket, the roof -- and torque scales
    # directly with this, so it does not get left behind.
    mass: float = 2.06
    limits: JointLimits = field(default_factory=JointLimits)
    # Which way each leg's knee buckles. See KNEE_LAYOUTS below.
    knee_layout: str = "back"
    # Roll travel, in degrees, measured from straight down. Inboard is
    # limited because that is the direction that puts a foot under the
    # robot and eventually into the opposite leg; outboard only widens the
    # stance. See the derivation in Quadruped.__init__.
    # Roll travel. Every degree here is turn rate: the gait saturates
    # this limit on every turning command, and clipping it to 20 cut the
    # delivered yaw from 17.6 to 9.6 deg/s. So it does not get traded
    # away to buy clearance -- the clearance gets fixed instead.
    #
    # It was 45 once. 35 is where the shoulder arm stops reaching
    # through the chassis plate.
    roll_inboard_deg: float = 35.0
    roll_outboard_deg: float = 35.0

    @property
    def stand_height(self) -> float:
        """A sensible default body height: legs bent enough to have travel in
        both directions. Standing near-straight looks impressive and leaves no
        room to absorb a stumble, so aim for about 75% of full extension."""
        return 0.75 * math.sqrt((self.thigh + self.shank) ** 2 - self.hip_offset ** 2)


# Leg order is fixed and used everywhere -- arrays of four values are always
# in this order, which avoids a whole family of "which leg was index 2" bugs.
LEG_NAMES = ("FL", "FR", "RL", "RR")


# ---------------------------------------------------------------------------
# WHICH WAY THE KNEES BEND
#
# Each leg here is two links and one bend joint, so it can buckle forward or
# backward. Both reach the identical foot position; the choice is mechanical.
# It is also the thing that makes a robot look like it is facing somewhere.
#
# What a real quadruped does, and the point is that it is NOT the same at both
# ends. A dog's front leg runs scapula -> humerus -> ELBOW -> forearm -> foot,
# and the elbow protrudes backward. Its hind leg runs pelvis -> femur ->
# STIFLE -> tibia -> hock -> foot, and the stifle protrudes forward. Front and
# rear bend opposite ways.
#
# That asymmetry is not decoration. It is why you can tell at a glance which
# way an animal is facing, and why a robot with all four knees the same way
# looks equally plausible walking either direction -- there is no cue.
#
# The three layouts worth having:
#
#   back      all four knees behind their hips. THE DEFAULT, and what Spot
#             runs. Every leg is the same part, and the knee TRAILS instead of
#             leading -- so the foot is the part that reaches out ahead, and
#             the knee is not the first thing to meet a step or a carpet edge.
#             The shank swings forward under the body to place the foot, which
#             is also the direction it needs to push.
#
#   forward   the mirror: all four knees ahead of their hips. Same one-part
#             saving, same stability, but the knee leads into whatever the
#             robot is walking toward.
#
#   mammal    front knees back, rear knees forward -- what a dog actually
#             does, since its front leg bends at the elbow (backward) and its
#             hind leg at the stifle (forward). Most animal-like, and what MIT
#             Cheetah copies, but front and rear become different parts.
#
# All three reach identical foot positions and measure the same stability
# margin to the millimetre. This is a mechanical choice, not a performance
# one -- the difference shows up in what the leg collides with, not in how
# well it walks on a flat floor.
# ---------------------------------------------------------------------------
KNEE_LAYOUTS: dict[str, dict[str, bool]] = {
    # True = knee ahead of the hip
    "back":    {"FL": False, "FR": False, "RL": False, "RR": False},
    "forward": {"FL": True,  "FR": True,  "RL": True,  "RR": True},
    "mammal":  {"FL": False, "FR": False, "RL": True,  "RR": True},
}


class Quadruped:
    """Four legs and the solver that poses them."""

    def __init__(self, config: RobotConfig | None = None) -> None:
        self.config = config or RobotConfig()
        c = self.config
        hl, hw = c.body_length / 2, c.body_width / 2

        if c.knee_layout not in KNEE_LAYOUTS:
            raise ValueError(
                f"unknown knee_layout {c.knee_layout!r}; "
                f"expected one of {sorted(KNEE_LAYOUTS)}"
            )
        knees = KNEE_LAYOUTS[c.knee_layout]

        # Roll limits are ASYMMETRIC and per-side.
        #
        # Rolling a leg outboard just splays the stance wider, and only the
        # leg's own workspace stops it. Rolling INBOARD swings the foot
        # under the robot -- at -28.8 deg on a left leg the foot reaches
        # the centreline.
        #
        # The crossing angle depends on HIP_OFFSET: a leg standing further
        # out has to roll further before its foot reaches the centreline.
        # At the 56 mm offset that was -28.8 deg; at 102.4 mm it is
        # -48.8 deg. The limit has to follow, or it becomes the thing
        # blocking the gait rather than protecting it -- which is exactly
        # what happened when the offset grew and this stayed at 30.
        #
        # Inboard is -roll on the left legs and +roll on the right, hence
        # the per-side limits.
        roll_in  = math.radians(c.roll_inboard_deg)
        roll_out = math.radians(c.roll_outboard_deg)
        side_roll = {
            LegSide.LEFT:  (-roll_in,  roll_out),
            LegSide.RIGHT: (-roll_out, roll_in),
        }

        # Hip positions in the body frame. +X forward, +Y left.
        origins = {
            "FL": (Vec3(+hl, +hw, 0.0), LegSide.LEFT),
            "FR": (Vec3(+hl, -hw, 0.0), LegSide.RIGHT),
            "RL": (Vec3(-hl, +hw, 0.0), LegSide.LEFT),
            "RR": (Vec3(-hl, -hw, 0.0), LegSide.RIGHT),
        }
        self.legs: dict[str, Leg] = {
            name: Leg(name, origin, side, c.hip_offset, c.thigh, c.shank,
                      replace(c.limits, roll=side_roll[side]),
                      knee_forward=knees[name])
            for name, (origin, side) in origins.items()
        }

    # -- the solver ---------------------------------------------------------

    def solve(
        self,
        pose: BodyPose,
        feet_world: dict[str, Vec3],
        *,
        clamp_to_limits: bool = False,
    ) -> dict[str, JointAngles]:
        """Body pose + where the feet are planted -> twelve joint angles.

        This is the function the whole controller runs through. Balance, gait
        and teleoperation all reduce to picking `pose` and `feet_world`.
        """
        r_inv = pose.rotation.inverse()
        out: dict[str, JointAngles] = {}
        for name in LEG_NAMES:
            # The one coordinate change, from the module docstring.
            foot_body = r_inv.apply(feet_world[name] - pose.position)
            out[name] = self.legs[name].solve_ik(foot_body, clamp_to_limits=clamp_to_limits)
        return out

    def feet_from_angles(
        self, pose: BodyPose, angles: dict[str, JointAngles]
    ) -> dict[str, Vec3]:
        """The inverse of `solve`: joint angles -> foot positions in the world.

        Used to check the solver, and to work out where the feet actually
        ended up when angles came back off real servos.
        """
        r = pose.rotation
        return {
            name: r.apply(self.legs[name].solve_fk(angles[name])) + pose.position
            for name in LEG_NAMES
        }

    # -- stances ------------------------------------------------------------

    def default_stance(self, pose: BodyPose | None = None, spread: float = 1.0) -> dict[str, Vec3]:
        """Where the feet go when just standing: directly below each hip, on
        the ground at z=0.

        `spread` widens the stance sideways. Wider is more stable (a bigger
        support polygon) but costs hip-roll torque the whole time it stands,
        so 1.0 -- feet directly under the hips -- is the efficient default.
        """
        pose = pose or BodyPose(position=Vec3(0, 0, self.config.stand_height))
        r = pose.rotation
        feet: dict[str, Vec3] = {}
        for name in LEG_NAMES:
            leg = self.legs[name]
            hip_world = r.apply(leg.origin) + pose.position
            lateral = leg.side.value * self.config.hip_offset * spread
            feet[name] = Vec3(hip_world.x, hip_world.y + lateral, 0.0)
        return feet

    def stand(self, height: float | None = None) -> tuple[BodyPose, dict[str, Vec3]]:
        """A ready-to-use standing pose and its footprint."""
        pose = BodyPose(position=Vec3(0, 0, height or self.config.stand_height))
        return pose, self.default_stance(pose)

    # -- stability ----------------------------------------------------------

    def support_polygon(self, feet_world: dict[str, Vec3], stance: set[str] | None = None) -> list[Vec3]:
        """The feet currently on the ground, ordered into a convex loop.

        The robot is statically stable exactly when its centre of mass
        projects inside this polygon. Ordering matters: the point-in-polygon
        test below needs the vertices to go round the perimeter, not criss-
        cross, so they are sorted by angle about their own centroid.
        """
        names = sorted(stance if stance is not None else set(LEG_NAMES))
        pts = [feet_world[n] for n in names]
        if len(pts) < 3:
            return pts
        cx = sum(p.x for p in pts) / len(pts)
        cy = sum(p.y for p in pts) / len(pts)
        return sorted(pts, key=lambda p: math.atan2(p.y - cy, p.x - cx))

    def stability_margin(self, pose: BodyPose, feet_world: dict[str, Vec3],
                         stance: set[str] | None = None) -> float:
        """Distance from the centre of mass to the nearest edge of support.

        Positive = stable, and the number is how far the robot could be shoved
        before it tips. Negative = the CoM is outside its feet and it is
        already falling. This is the single number a static gait has to keep
        above zero at every instant.

        The CoM is approximated as the body's origin. That is a real
        simplification -- the legs weigh something and they move -- but for a
        chassis-heavy robot (battery and Pi in the body, light legs) it is
        close, and it is the standard first-order model.
        """
        poly = self.support_polygon(feet_world, stance)
        if len(poly) < 3:
            return float("-inf")        # a line or a point supports nothing
        com = (pose.position.x, pose.position.y)

        inside = True
        best = float("inf")
        n = len(poly)
        for i in range(n):
            ax, ay = poly[i].x, poly[i].y
            bx, by = poly[(i + 1) % n].x, poly[(i + 1) % n].y
            ex, ey = bx - ax, by - ay
            # Cross product sign says which side of this edge the CoM is on.
            # Vertices are counter-clockwise, so inside means every cross > 0.
            if ex * (com[1] - ay) - ey * (com[0] - ax) < 0:
                inside = False
            edge_len = math.hypot(ex, ey)
            if edge_len > 1e-12:
                best = min(best, abs(ex * (com[1] - ay) - ey * (com[0] - ax)) / edge_len)
        return best if inside else -best

    def is_stable(self, pose: BodyPose, feet_world: dict[str, Vec3],
                  stance: set[str] | None = None, margin: float = 0.01) -> bool:
        """Stable with at least `margin` metres to spare (default 1 cm)."""
        return self.stability_margin(pose, feet_world, stance) >= margin

    # -- diagnostics --------------------------------------------------------

    def describe(self) -> str:
        c = self.config
        leg = self.legs["FL"]
        return (
            f"Quadruped  body {c.body_length*1000:.0f} x {c.body_width*1000:.0f} mm, "
            f"mass {c.mass:.1f} kg\n"
            f"  links     hip offset {c.hip_offset*1000:.0f}, thigh {c.thigh*1000:.0f}, "
            f"shank {c.shank*1000:.0f} mm\n"
            f"  reach     {leg.min_reach*1000:.0f} to {leg.max_reach*1000:.0f} mm "
            f"per leg\n"
            f"  stands at {c.stand_height*1000:.0f} mm\n"
            f"  knees     {c.knee_layout} "
            f"(front {'fwd' if self.legs['FL'].knee_forward else 'back'}, "
            f"rear {'fwd' if self.legs['RL'].knee_forward else 'back'})"
        )
