"""Mapping between joint angles and servo angles.

THE KNEE IS DRIVEN THROUGH A PARALLELOGRAM LINKAGE. The servo sits part way
up the thigh and pushes a rod down to a short rocker on the shank, so the
knee joint itself carries no motor -- which is the layout in the concept
sketch, and the reason the shank can be slim through its middle.

    C  crank axis, fixed to the thigh, LINK_D above the knee
    K  knee axis
    crank  length LINK_A, on the servo at C
    rocker length LINK_A, rigid with the shank, at K
    rod    length LINK_D, crank tip to rocker tip

Equal cranks plus a rod equal to CK is a parallelogram, so the rocker stays
parallel to the crank and

    servo angle (measured from the thigh) = knee angle + LINK_PHI

exactly -- at every position, not just near one. That is the property the
two earlier attempts could not get, and it is worth being precise about why,
because "a four-bar knee" sounds like one idea and is really three:

  coaxial parallelogram     Also exact, but its crank axis IS the hip pitch
                            axis, so both servos want the same line and stack
                            end to end along it. Measured on the real stack
                            that needs 128 mm a side -- a 366 mm wide robot
                            with a 235 mm stance, knee servos hanging 66 mm
                            past their own feet. Exactness was never the
                            problem; packaging was.

  general (non-coaxial)     Compact, inverse still closed form, and wrong in
  four-bar                  a way that only shows up under load, because a
                            general four-bar's ratio varies with position. Of
                            45 geometries that covered the envelope
                            monotonically, the best had a worst-case ratio of
                            0.12 -- one degree of crank to eight of knee. The
                            knee's 11.1 kg.cm becomes 92.5 kg.cm at the servo,
                            against a 25 kg.cm part.

  this one                  Non-coaxial, so it packages; parallelogram, so
                            the ratio is exactly 1.0 everywhere. What the
                            first attempt missed is that the crank axis does
                            not have to be the hip axis -- it only has to be
                            FIXED TO THE THIGH. Then the crank turns with the
                            thigh, the servo reads the knee angle directly,
                            and the hip is left alone.

WHAT IT COSTS. A parallelogram folds into its crossed twin if the crank ever
lines up with CK. The knee's 135 degrees of travel has to sit inside a 180
degree window, so the margin at each end is at most 22.5 degrees, and
LINK_PHI is chosen to centre the travel and get exactly that. Under servo
control the crank cannot reach a dead point; the printed stops on the thigh
are there for the case where something backdrives it.

WHAT IT BUYS. The knee servo moves from the knee to LINK_D above it, so its
share of the leg's swing inertia falls with the square of the radius -- see
`inertia_saving()`. That is the whole point of a linkage, and it is why the
sketch is right.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .leg import JointAngles

# ---------------------------------------------------------------------
# LINKAGE GEOMETRY, in metres. cad/sync_params.py writes these same
# numbers into params.scad, so the CAD and the controller cannot drift.
#
# LINK_A is set by the rod force (knee torque / LINK_A) and by needing the
# crank's swept disc clear of the rocker's. LINK_D is set by how far up the
# thigh the servo can go before the crank's travel reaches the hip hardware.
# Both were swept numerically before anything was drawn.
# ---------------------------------------------------------------------
# Both of these are squeezed between two constraints that pull opposite
# ways, because CRANK_C + LINK_D is fixed at the thigh's length:
#
#   LINK_D small   puts the crank axis far from the hip, where its sweep
#                  clears the pitch servo's body
#   LINK_D large   keeps the crank's swept disc clear of the rocker's,
#                  which needs 2*(LINK_A + end radius) < LINK_D
#
# A shorter crank relaxes both at the cost of rod force, and the rod is
# nowhere near its limit, so the crank is what gives.
LINK_A = 0.015      # crank and rocker -- EQUAL lengths are what make it exact
LINK_D = 0.046      # crank axis to knee axis, and the rod's length

KNEE_RANGE = (math.radians(15.0), math.radians(150.0))

# Fixed angle between the shank and its rocker. Centres the crank's travel
# in the half-plane behind the thigh, as far from both dead points as the
# knee's range allows. The shank only ever swings forward of the thigh, so
# behind it is empty.
LINK_PHI = math.radians(
    -(math.degrees(KNEE_RANGE[0]) + math.degrees(KNEE_RANGE[1])) / 2 + 90.0 - 180.0
)


def crank_angle(knee: float) -> float:
    """Crank angle measured from the thigh line. The parallelogram makes
    this the servo's own output angle."""
    return knee + LINK_PHI


def dead_point_margin(knee: float) -> float:
    """How far this knee angle keeps the crank from collapsing the linkage
    into its crossed twin, in radians. Zero means crank collinear with CK."""
    t = crank_angle(knee)
    return min(abs(t), abs(abs(t) - math.pi))


def inertia_saving(servo_mass: float, thigh: float) -> tuple[float, float]:
    """Knee servo's contribution to the leg's inertia about the hip, driven
    directly at the knee versus on the thigh at LINK_D above it.

    Point mass is enough here: the servo is small next to the radius, and
    the comparison is a ratio.
    """
    direct = servo_mass * thigh ** 2
    linked = servo_mass * (thigh - LINK_D) ** 2
    return direct, linked


@dataclass(frozen=True)
class ServoAngles:
    """What the three servos on one leg are commanded."""

    roll: float
    servo1: float      # thigh, at the hip
    servo2: float      # knee, on the thigh, through the linkage

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.roll, self.servo1, self.servo2)

    def degrees(self) -> tuple[float, float, float]:
        return tuple(math.degrees(a) for a in self.as_tuple())

    def __repr__(self) -> str:
        r, a, b = self.degrees()
        return f"ServoAngles(roll={r:+.1f}, s1={a:+.1f}, s2={b:+.1f})"


def to_servos(q: JointAngles) -> ServoAngles:
    return ServoAngles(roll=q.roll, servo1=q.hip, servo2=crank_angle(q.knee))


def to_joints(s: ServoAngles) -> JointAngles:
    return JointAngles(roll=s.roll, hip=s.servo1, knee=s.servo2 - LINK_PHI)


# Travel required, in radians. Measured across every gait and the full
# balance envelope, not assumed. servo2's is the knee's, shifted by the
# linkage -- which is the whole content of the mapping.
TRAVEL = {
    "roll":   (math.radians(-30.0), math.radians(+30.0)),
    "servo1": (math.radians(-90.0), math.radians(+20.0)),
    "servo2": (crank_angle(KNEE_RANGE[0]), crank_angle(KNEE_RANGE[1])),
}
