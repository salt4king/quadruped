# attic

Superseded files, kept only so the reasoning is recoverable. **Nothing in
the build reads anything here.**

- `assembly.four-bar.scad` — the leg when the knee was driven through a
  four-bar linkage. The linkage was abandoned (see
  `quadruped/transmission.py` for the measurements that killed it) and
  this file still refers to `CRANK`, `ROD_Y` and friends, which no longer
  exist. OpenSCAD does not error on an unknown variable, it quietly
  substitutes `undef` and renders nonsense — which is exactly how the
  shank ended up as a hairline thread for several revisions. Keep dead
  geometry out of the build directory.
