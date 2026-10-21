#!/usr/bin/env bash
# Render every part to STL. Run cad/sync_params.py first if RobotConfig changed.
set -uo pipefail
cd "$(dirname "$0")"
mkdir -p stl
render() {  # file part
  printf '  %-14s ' "$2"
  if openscad -o "stl/$2.stl" -D "PART=\"$2\"" --export-format=binstl "$1" 2>/dev/null; then
    printf '%s\n' "$(du -h "stl/$2.stl" | cut -f1)"
  else printf 'FAILED\n'; fi
}
# LEFT parts, as modelled. The assembly uses these unmirrored on the
# left pair of legs.
echo "leg, left:";  for p in thigh shank crank pushrod; do render leg.scad "$p"; done
# RIGHT parts. thigh, shank and crank have no mirror plane, so the
# right side genuinely needs its own print; pushrod does, so it is not
# repeated here. See the note at the bottom of leg.scad.
render_r() {
  printf '  %-14s ' "$2_right"
  if openscad -o "stl/$2_right.stl" -D "PART=\"$2\"" -D "HAND=-1" \
       --export-format=binstl "$1" 2>/dev/null; then
    printf '%s\n' "$(du -h "stl/$2_right.stl" | cut -f1)"
  else printf 'FAILED\n'; fi
}
echo "leg, right:"; for p in thigh shank crank; do render_r leg.scad "$p"; done
# One filler for all twelve servos. They used to need two versions
# because the roll servo's mount was handed the other way; it is not any
# more -- it loads from the back like the rest.
echo "filler:";   render leg.scad filler

# ONE shoulder. There used to be two mirror-image versions, because the
# roll servo mirrored fore/aft and the arm had to be held forward of the
# thigh regardless. Nothing mirrors fore/aft now -- all four legs are
# the same assembly -- so there is one part.
echo "shoulder:"; render shoulder.scad shoulder
# The whole plate, for reference and for the assembly view. It is 280
# long and does not fit on a 235 mm bed -- print the two halves below.
echo "chassis:"; render chassis.scad chassis
echo "chassis, split to fit the bed:"
render chassis.scad chassis_front
render chassis.scad chassis_rear
# The lid. Print it upside down: every overhang is then a wall or the
# barb's own ramp, so it needs no support.
echo "roof:";    render chassis.scad roof
echo
echo "Per robot: 2+2 thigh, 2+2 shank, 2+2 crank (left and right),"
echo "4x pushrod,"
echo "12x filler (one per servo), 1x roof,"
echo "4x shoulder, 1x chassis.  Every leg is the same."
echo "The roll servo mounts are part of the chassis."
echo "Crank and pushrod print flat; M3 bolts at every linkage pivot."
echo "PETG."
echo "ORIENTATION. Lay the thigh, shank, crank and pushrod FLAT, largest"
echo "face on the bed, so the layers run ALONG the part and bending"
echo "tension stays inside a layer instead of pulling layers apart. The"
echo "note here used to say \"layer lines across the long axis\", which"
echo "read literally means standing them on end -- the weakest way to"
echo "print a beam, and the opposite of what check_walls.py assumes when"
echo "it derates PETG by 0.6 for cross-layer loading."
echo "Roof: skin on the bed, upside down. Chassis: as modelled."
