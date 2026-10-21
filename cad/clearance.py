"""Exact minimum distance between two triangulated parts.

WHY THIS EXISTS. The interference check answers "do these two solids share
material", which is a yes/no with a volume attached. That is the wrong
question to be asking late in a design: by the time it says yes, the part
is already wrong, and when it says no it tells you nothing about whether
you cleared by 5 mm or by 0.05.

This answers "how close do they get". A clearance of 0.3 mm passes an
interference check and fails on a printer.

WHY NOT OCCT. cadquery/OCP has BRepExtrema_DistShapeShape and it is exact,
but on these meshes it takes 3.2 s a pair -- about 90 minutes for the
4 legs x 15 poses x 27 pairs the checker sweeps. This does the same job in
milliseconds, and it is exact too: the minimum distance between two
disjoint convex sets is attained either vertex-to-face or edge-to-edge, so
taking the smallest of the 9 edge pairs and the 6 vertex-face pairs is not
an approximation.

Intersecting triangles return 0.0. Depth of overlap is check_fit.py's job.
"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def load_stl(path: str | Path) -> np.ndarray:
    """Binary STL -> (N, 3, 3) array of triangle vertices, in mm."""
    d = Path(path).read_bytes()
    n = struct.unpack("<I", d[80:84])[0]
    # 50 bytes a facet: 12 normal, 36 vertices, 2 attribute.
    raw = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


def transform(tris: np.ndarray, m: np.ndarray) -> np.ndarray:
    """Apply a 4x4 to every vertex."""
    return tris @ m[:3, :3].T + m[:3, 3]


def _seg_seg(p1, q1, p2, q2):
    """Distance between segments p1q1 and p2q2, vectorised over the first axis.

    The textbook clamped solve. The degenerate branches matter here: a
    triangulated part has plenty of near-parallel edge pairs, and letting
    the denominator go to zero quietly returns nonsense.
    """
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a = np.einsum("ij,ij->i", d1, d1)
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    c = np.einsum("ij,ij->i", d1, r)
    b = np.einsum("ij,ij->i", d1, d2)
    denom = a * e - b * b

    s = np.zeros_like(a)
    nz = denom > 1e-12
    s[nz] = np.clip((b[nz] * f[nz] - c[nz] * e[nz]) / denom[nz], 0.0, 1.0)

    e_safe = np.where(e > 1e-12, e, 1.0)
    t = (b * s + f) / e_safe
    t = np.where(e > 1e-12, t, 0.0)

    t_cl = np.clip(t, 0.0, 1.0)
    a_safe = np.where(a > 1e-12, a, 1.0)
    s = np.where(a > 1e-12, np.clip((b * t_cl - c) / a_safe, 0.0, 1.0), 0.0)

    diff = (p1 + d1 * s[:, None]) - (p2 + d2 * t_cl[:, None])
    return np.linalg.norm(diff, axis=1)


def _point_tri(p, tri):
    """Distance from points p (M,3) to triangles tri (M,3,3)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    ab, ac, ap = b - a, c - a, p - a
    d1 = np.einsum("ij,ij->i", ab, ap)
    d2 = np.einsum("ij,ij->i", ac, ap)
    bp = p - b
    d3 = np.einsum("ij,ij->i", ab, bp)
    d4 = np.einsum("ij,ij->i", ac, bp)
    cp = p - c
    d5 = np.einsum("ij,ij->i", ab, cp)
    d6 = np.einsum("ij,ij->i", ac, cp)

    vc = d1 * d4 - d3 * d2
    vb = d5 * d2 - d1 * d6
    va = d3 * d6 - d5 * d4
    denom = va + vb + vc
    safe = np.where(np.abs(denom) > 1e-12, denom, 1.0)
    v = np.clip(vb / safe, 0.0, 1.0)
    w = np.clip(vc / safe, 0.0, 1.0)
    over = v + w > 1.0
    scale = np.where(over, v + w, 1.0)
    v, w = v / scale, w / scale

    closest = a + ab * v[:, None] + ac * w[:, None]
    inside = np.linalg.norm(p - closest, axis=1)

    # Degenerate triangles fall back to their edges.
    edge = np.minimum.reduce([
        _seg_seg(p, p, a, b), _seg_seg(p, p, b, c), _seg_seg(p, p, c, a)])
    return np.where(np.abs(denom) > 1e-12, inside, edge)


def _pair_dist(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Exact distance for paired triangles A[i], B[i]."""
    out = np.full(len(A), np.inf)
    for i in range(3):
        for j in range(3):
            out = np.minimum(out, _seg_seg(
                A[:, i], A[:, (i + 1) % 3], B[:, j], B[:, (j + 1) % 3]))
    for i in range(3):
        out = np.minimum(out, _point_tri(A[:, i], B))
        out = np.minimum(out, _point_tri(B[:, i], A))
    return out


def min_distance(A: np.ndarray, B: np.ndarray, cutoff: float = 25.0) -> float:
    """Smallest distance between two triangle soups, in mm.

    `cutoff` only bounds the search: anything further apart than this is
    reported as the cutoff, because a part 25 mm away is not interesting
    and widening the net costs time.
    """
    if len(A) == 0 or len(B) == 0:
        return cutoff

    ca, cb = A.mean(axis=1), B.mean(axis=1)
    ra = np.linalg.norm(A - ca[:, None, :], axis=2).max(axis=1)
    rb = np.linalg.norm(B - cb[:, None, :], axis=2).max(axis=1)

    # Cheap upper bound first, so the real query has a small radius.
    tree = cKDTree(ca)
    d0, i0 = tree.query(cb, k=1)
    best = float(min(cutoff, (d0 + ra[i0] + rb).min()))

    radius = best + ra.max() + rb.max()
    pairs = tree.query_ball_point(cb, radius)

    bi = np.concatenate([np.full(len(p), k, dtype=np.int64)
                         for k, p in enumerate(pairs) if p] or [np.empty(0, np.int64)])
    ai = np.concatenate([np.asarray(p, dtype=np.int64)
                         for p in pairs if p] or [np.empty(0, np.int64)])
    if len(ai) == 0:
        return best

    # Drop pairs whose bounding spheres already exceed the best bound.
    sep = np.linalg.norm(ca[ai] - cb[bi], axis=1) - ra[ai] - rb[bi]
    keep = sep < best
    ai, bi = ai[keep], bi[keep]
    if len(ai) == 0:
        return best

    return float(min(best, _pair_dist(A[ai], B[bi]).min()))
