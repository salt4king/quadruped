"""A quadruped robot controller: kinematics, gait and balance.

The control core is stdlib-only, so it runs identically on a laptop and on a
Raspberry Pi with nothing to install.
"""

from .vec import Vec3, Rot
from .leg import Leg, LegSide, JointAngles, JointLimits, Unreachable

__all__ = [
    "Vec3", "Rot",
    "Leg", "LegSide", "JointAngles", "JointLimits", "Unreachable",
]
