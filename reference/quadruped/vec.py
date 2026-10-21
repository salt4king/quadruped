"""Minimal 3D math. Stdlib only, on purpose.

A quadruped controller touches 12 joints at a few hundred Hz. That is a
tiny amount of arithmetic -- numpy's per-call overhead would actually cost
more than it saves at this size. Staying dependency-free means this same
code runs on a Raspberry Pi with nothing to install.

Frame convention used everywhere in this project (right-handed):

    +X  forward   (the direction the robot walks)
    +Y  left
    +Z  up

So a foot on the ground is at negative Z relative to the body.
"""

from __future__ import annotations

import math
from typing import NamedTuple


class Vec3(NamedTuple):
    """A 3D point or direction. Immutable, so it can't be aliased by accident."""

    x: float
    y: float
    z: float

    def __add__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, k: float) -> "Vec3":
        return Vec3(self.x * k, self.y * k, self.z * k)

    __rmul__ = __mul__

    def __truediv__(self, k: float) -> "Vec3":
        return Vec3(self.x / k, self.y / k, self.z / k)

    def __neg__(self) -> "Vec3":
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, o: "Vec3") -> float:
        return self.x * o.x + self.y * o.y + self.z * o.z

    def cross(self, o: "Vec3") -> "Vec3":
        return Vec3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self) -> "Vec3":
        n = self.norm()
        if n == 0.0:
            raise ValueError("cannot normalize a zero vector")
        return self / n

    def distance_to(self, o: "Vec3") -> float:
        return (self - o).norm()

    def __repr__(self) -> str:  # readable in test failures
        return f"({self.x:+.4f}, {self.y:+.4f}, {self.z:+.4f})"


ZERO = Vec3(0.0, 0.0, 0.0)


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


# ---------------------------------------------------------------------------
# Rotations
#
# Body attitude is stored as roll / pitch / yaw (radians), applied in that
# order as an extrinsic Z-Y-X sequence -- i.e. R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
# That is the aerospace convention, and it is what an IMU will hand you, so
# matching it here avoids a conversion layer later.
# ---------------------------------------------------------------------------


class Rot:
    """A 3x3 rotation matrix, stored row-major as a flat 9-tuple."""

    __slots__ = ("m",)

    def __init__(self, m: tuple[float, ...]) -> None:
        self.m = m

    @staticmethod
    def identity() -> "Rot":
        return Rot((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))

    @staticmethod
    def from_rpy(roll: float, pitch: float, yaw: float) -> "Rot":
        """Rz(yaw) . Ry(pitch) . Rx(roll), multiplied out."""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return Rot((
            cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp,     cp * sr,                cp * cr,
        ))

    def apply(self, v: Vec3) -> Vec3:
        m = self.m
        return Vec3(
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z,
        )

    def transpose(self) -> "Rot":
        m = self.m
        return Rot((m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]))

    # For a rotation matrix the inverse *is* the transpose, and computing it
    # that way is both faster and numerically exact.
    inverse = transpose

    def __matmul__(self, o: "Rot") -> "Rot":
        a, b = self.m, o.m
        return Rot(tuple(
            sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3))
            for r in range(3) for c in range(3)
        ))
