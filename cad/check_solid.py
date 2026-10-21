"""Is each part ONE piece?

A boss that touches nothing, a rib left behind when the plate around it
was cut away, a pad hanging in mid-air: all of these render perfectly,
pass an interference check, pass a wall-thickness audit, and come off the
printer as loose bits in the bottom of the bed.

Nothing else in this project would notice. check_fit asks whether parts
overlap each other; check_walls asks whether a feature is thick enough;
neither asks whether a part is connected to itself.

Method: weld vertices that coincide (STL stores each triangle's corners
independently, so a shared edge is two separate but identical points),
then walk the triangle graph. Each component is a separate lump of
plastic.

Run:  python3 cad/check_solid.py
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

CAD = Path(__file__).resolve().parent
WELD = 1e-4          # mm; STL is float32, so exact equality is too strict


def load(path: Path) -> np.ndarray:
    d = path.read_bytes()
    n = struct.unpack("<I", d[80:84])[0]
    raw = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


def components(tris: np.ndarray) -> tuple[int, np.ndarray]:
    """Number of separate lumps, and the volume of each."""
    pts = tris.reshape(-1, 3)
    # Weld by rounding onto a grid: everything that lands in the same cell
    # is the same vertex.
    key = np.round(pts / WELD).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    vid = inv.reshape(-1, 3)

    nt = len(tris)
    rows = np.repeat(np.arange(nt), 3)
    cols = vid.ravel()
    # triangle <-> vertex incidence, then square it to get triangle
    # adjacency through shared vertices
    inc = coo_matrix((np.ones(len(rows), bool), (rows, cols)),
                     shape=(nt, vid.max() + 1)).tocsr()
    adj = inc @ inc.T
    ncomp, label = connected_components(adj, directed=False)

    vols = np.zeros(ncomp)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    tv = (np.einsum("ij,ij->i", a, np.cross(b, c))) / 6.0
    for i in range(ncomp):
        vols[i] = abs(tv[label == i].sum()) / 1000.0
    return ncomp, np.sort(vols)[::-1]


def main() -> None:
    stl = sorted((CAD / "stl").glob("*.stl"))
    if not stl:
        raise SystemExit("no STLs -- run cad/export.sh first")
    print("\nIs each part one piece?\n")
    print(f"{'part':<18}{'lumps':>7}{'largest cm3':>14}   loose bits")
    print("-" * 66)
    bad = []
    for p in stl:
        n, vols = components(load(p))
        loose = vols[1:]
        note = "-" if n == 1 else ", ".join(f"{v:.2f}" for v in loose[:6]) + \
               (" ..." if len(loose) > 6 else "")
        if n != 1:
            bad.append((p.stem, n, loose))
        print(f"{p.stem:<18}{n:>7}{vols[0]:>14.2f}   {note}")
    print("-" * 66)
    if bad:
        print()
        for name, n, loose in bad:
            print(f"{name}: {n} separate pieces. "
                  f"{len(loose)} would fall off the bed, "
                  f"totalling {loose.sum():.2f} cm3.")
        sys.exit(1)
    print("\nEvery part is a single connected solid.")


if __name__ == "__main__":
    main()
