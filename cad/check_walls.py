"""Minimum-thickness audit of every part.

Printed parts fail at their thinnest section, not their average one, and
the thin spots in a parametric model are usually EMERGENT -- nobody types
"make this 1.2 mm", it falls out of one diameter being subtracted from
another. So this reads the real numbers out of params.scad and recomputes
every wall, web and boss the design creates.

Thresholds for a 0.4 mm nozzle in PETG:
    < 0.8 mm   will not print as a solid wall at all
    < 2.0 mm   prints, but is perimeters only with no infill behind them
    >= 3.0 mm  comfortable

Run:  python3 cad/check_walls.py
"""

import math
import re
import sys
from pathlib import Path

# Read the numbers out of the files that define them, params.scad and
# the part files alike -- each in its own namespace, so nothing shadows
# anything.
#
# There used to be a hand-written LOCAL dict here for the values that
# live in the part files. It went stale exactly as its own comment
# warned: it still said the chassis plate was 3 mm after it became 8,
# and the hex web 3.4 after it became 5. An audit reporting numbers the
# model does not have is worse than no audit, because it reads green.
def scan(name):
    src = (Path(__file__).resolve().parent / name).read_text()
    return {k: float(v) for k, v in
            re.findall(r"^(\w+)\s*=\s*([-\d.]+)\s*;", src, re.M)}


P = scan("params.scad")
LEG = scan("leg.scad")
CHASSIS = scan("chassis.scad")
SHOULDER = scan("shoulder.scad")
V = P
BOSS_D = V["JOINT_BORE_D"] + 2 * V["WALL"]

CHECKS = [
    # part,            feature,                              thickness
    ("chassis",  "plate skin",                    CHASSIS["CHASSIS_T"]),
    ("chassis",  "perimeter / spine rail",        CHASSIS["RAIL_W"]),
    ("chassis",  "web between hexagons",          CHASSIS["HEX_GAP"]),
    ("chassis",  "servo housing wall",            V["WALL"]),
    ("thigh",    "plate thickness",               V["PLATE_T"]),
    ("thigh",    "truss rail",                    V["TRUSS_RAIL"]),
    # No "spigot wall" for the thigh any more: its hip collar is a solid
    # disc, because a servo horn bolts to its face. The shank's is still
    # a ring -- the knee pin goes through it.
    ("thigh",    "knee-servo bracket, crossing", LEG["CROSS_H"]),
    ("thigh",    "knee-servo bracket, rail",      LEG["RAIL_T"]),
    ("thigh",    "boss wall around bore",         V["WALL"]),
    ("thigh",    "material around knee pivot",
                 V["KNEE_R"] - V["PIVOT_D"] / 2),
    ("shank",    "beam thickness (lateral)",      V["SHANK_T"]),
    ("shank",    "beam width at the knee",        V["SHANK_W_KNEE"]),
    ("shank",    "beam width at the ankle",       V["SHANK_W_FOOT"]),
    ("shank",    "material around knee pivot",
                 V["KNEE_R"] - V["PIVOT_D"] / 2),
    ("shank",    "spigot wall",
                 (V["JOINT_BORE_D"] - 2*V["CLEAR"] - (V["SERVO_HORN_D"] + 2)) / 2),
    ("shank",    "foot shell over the pad pocket",
                 V["FOOT_R"] - 3.0),
    ("mount",    "wall around each M3 insert",
                 (V["M3_INSERT"] + 2*3.0 - V["M3_INSERT"]) / 2),
    ("mount",    "pad depth for the insert",       8.0),
    ("mount",    "housing wall",                   V["WALL"]),
    ("crank",    "plate thickness",               V["LINK_T"]),
    ("crank",    "material around the pivot",
                 V["LINK_TIP_R"] + 1 - V["PIVOT_D"] / 2),
    ("crank",    "hub wall around the horn pocket",
                 ((V["SERVO_HORN_D"] + 2*V["CLEAR"] + 2*V["WALL"])
                  - (V["SERVO_HORN_D"] + 2*V["CLEAR"])) / 2),
    ("crank",    "slim hub wall at the rod's plane",
                 (16.0 - 10.0) / 2),
    ("pushrod",  "plate thickness",               V["ROD_T"]),
    ("pushrod",  "material around each pivot",
                 V["LINK_TIP_R"] - V["PIVOT_D"] / 2),
    ("shoulder", "housing wall",                  V["WALL"]),
    ("shoulder", "arm rail width",                SHOULDER["ARM_W"]),
    ("shoulder", "arm rail depth",                V["ARM_T"]),
    ("shoulder", "collar wall",                   (BOSS_D - V["JOINT_BORE_D"]) / 2),
    ("shoulder", "ear plate around bolt hole",
                 (V["SERVO_W"] + 13 - V["SERVO_HOLE_DY"] - V["M3_CLEAR"]) / 2),
]

FAIL, WARN = 0.8, 2.0
bad, marginal = [], []

print("\nMinimum-thickness audit  (0.4 mm nozzle, PETG)\n")
print(f"{'part':<10}{'feature':<34}{'mm':>7}   verdict")
print("-" * 72)
last = None
for part, feat, t in CHECKS:
    if part != last:
        print() if last else None
        last = part
    if t < FAIL:
        v = "WILL NOT PRINT"; bad.append((part, feat, t))
    elif t < WARN:
        v = "thin - perimeters only"; marginal.append((part, feat, t))
    elif t < 3.0:
        v = "ok, tight"
    else:
        v = "ok"
    print(f"{part:<10}{feat:<34}{t:>7.2f}   {v}")

print("\n" + "-" * 72)
if bad:
    print(f"{len(bad)} FEATURE(S) BELOW {FAIL} mm -- these are not printable:")
    for p, f, t in bad:
        print(f"  {p}: {f} = {t:.2f} mm")
    sys.exit(1)
if marginal:
    print(f"{len(marginal)} thin but printable (>= {FAIL} mm):")
    for p, f, t in marginal:
        print(f"  {p}: {f} = {t:.2f} mm")
print("No feature is below the printable limit.")


# =====================================================================
# WILL IT SNAP?
#
# Printable is not the same as strong. Everything above only says the
# slicer can lay the walls down. This says whether the section survives
# the load, which is the question actually being asked when a part
# "looks like it will break".
#
# Load case: the robot lands on ONE leg and the foot scuffs sideways.
# That is the worst thing a walking gait does to a leg -- it puts the
# whole weight at the end of the longest lever, in the direction the
# section is thinnest.
#
# Allowable: PETG yields near 50 MPa in bulk. Printed parts break along
# the layer lines, so derate hard -- 30 MPa, and then want a safety
# factor of 2 on top because this thing falls over while you tune it.
# =====================================================================
G = 9.81
MASS = V["MASS_EST"]
PETG_YIELD = 50.0      # MPa, bulk
DERATE = 0.6           # across layer lines
ALLOW = PETG_YIELD * DERATE
WANT_SF = 2.0

def beam(name, width, thick, lever, load, about):
    """Rectangular section. 'width' is in the bending plane, 'thick' across it."""
    b, h = (thick, width) if about == "edge" else (width, thick)
    Z = b * h * h / 6.0                 # mm^3
    M = load * lever                    # N.mm
    sigma = M / Z                       # MPa
    return name, Z, M, sigma, ALLOW / sigma

print()
print("=" * 72)
print(f"Bending check -- {MASS:.2f} kg on one leg, sideways scuff at the foot")
print(f"PETG {PETG_YIELD:.0f} MPa derated x{DERATE} across layers "
      f"= {ALLOW:.0f} MPa allowable, want SF >= {WANT_SF:.0f}")
print("=" * 72)
print(f"{'section':<34}{'Z mm3':>9}{'M N.mm':>9}{'MPa':>8}{'SF':>7}   verdict")
print("-" * 72)

F = MASS * G                            # N, all of it on one foot
rows = [
    # The shank is the long lever. A side load bends it the hard way,
    # across the 8 mm, so that is the case that matters.
    beam("shank at knee, side load",
         V["SHANK_W_KNEE"], V["SHANK_T"], V["SHANK"], F, "flat"),
    beam("shank at knee, fore/aft load",
         V["SHANK_W_KNEE"], V["SHANK_T"], V["SHANK"], F, "edge"),
    beam("shank at ankle, side load",
         V["SHANK_W_FOOT"], V["SHANK_T"], 30.0, F, "flat"),
    # The thigh sees the same force one link further up.
    beam("thigh at knee end, side load",
         2 * V["KNEE_R"], V["PLATE_T"], V["SHANK"] + V["THIGH"], F, "flat"),
    beam("thigh at hip end, fore/aft load",
         2 * V["HEAD_R"], V["PLATE_T"], V["SHANK"] + V["THIGH"], F, "edge"),
]

# The linkage is loaded differently -- the rod is a strut, not a beam, so
# what matters is whether it buckles, and the arms are short cantilevers
# carrying the rod's force at their tip.
ROD_F = 1.088 / (V["LINK_A"] / 1000.0)       # N, knee torque / crank length
E_PETG = 2000.0                              # MPa, stiffness not strength
I_rod = V["ROD_T"] * (2*V["LINK_TIP_R"])**3 / 12.0        # mm^4, weak axis
I_rod = min(I_rod, (2*V["LINK_TIP_R"]) * V["ROD_T"]**3 / 12.0)
P_cr = math.pi**2 * E_PETG * I_rod / V["LINK_D"]**2
rows.append(beam("crank arm at its root",
                 2 * 13.0, V["LINK_T"], V["LINK_A"], ROD_F, "flat"))
rows.append(beam("rocker arm at its root",
                 2 * V["KNEE_R"], V["LINK_T"], V["LINK_A"], ROD_F, "flat"))

# THE SHOULDER'S ARM, and why it gets its own two rows.
#
# It is the single load path between the roll collar and everything
# below it -- both servos, thigh, shank, and whatever the foot is
# standing on -- and it is a 9 mm rail. Bending it is the obvious case
# and not the worst one.
#
# The arm RUNS ALONG Y. A sideways scuff at the foot puts a moment about
# X across it, which it takes in bending. A fore/aft scuff puts that
# same moment about Y, which is along the arm's own axis -- so the arm
# does not bend, it TWISTS. An open rectangle is poor in torsion and
# that is the case that governs here.
#
# Rectangular torsion: tau_max = T / (alpha * a * b^2), a the long side,
# b the short one. Shear allowable is about 0.6 of tensile.
TORSION_ALPHA = [(1.0, 0.208), (1.5, 0.231), (2.0, 0.246), (2.5, 0.258),
                 (3.0, 0.267), (4.0, 0.282), (6.0, 0.299), (10.0, 0.312)]
SHEAR_ALLOW = ALLOW * 0.6


def alpha_for(ratio):
    if ratio <= TORSION_ALPHA[0][0]:
        return TORSION_ALPHA[0][1]
    for (r0, a0), (r1, a1) in zip(TORSION_ALPHA, TORSION_ALPHA[1:]):
        if ratio <= r1:
            return a0 + (a1 - a0) * (ratio - r0) / (r1 - r0)
    return 0.333


def twist(name, side1, side2, lever, load):
    """Rectangular section in torsion. Returns a row shaped like beam()."""
    a, b = max(side1, side2), min(side1, side2)
    T = load * lever
    tau = T / (alpha_for(a / b) * a * b * b)
    return name, alpha_for(a / b) * a * b * b, T, tau, SHEAR_ALLOW / tau


RIDE = V["RIDE_HEIGHT"]
ARM_W = SHOULDER["ARM_W"]
# The arm is two members at right angles and they do not see the same
# thing. The LONG RUN goes along Y, so a fore/aft scuff twists it. The
# TONGUE goes along X out of the boss, so the same scuff BENDS it and a
# sideways one twists it instead. The tongue is the rooted end and it
# carries everything, and its section is the band it crosses in -- not
# the rail's width, which is its length.
ARM_BAND = V["ARM_Y1"] - V["ARM_Y0"]
rows.append(beam("shoulder arm long run, side",
                 ARM_W, V["ARM_T"], RIDE, F, "flat"))
rows.append(twist("shoulder arm long run (TWIST)",
                  ARM_W, V["ARM_T"], RIDE, F))
rows.append(beam("shoulder tongue, fore/aft",
                 ARM_BAND, V["ARM_T"], RIDE, F, "flat"))
rows.append(twist("shoulder tongue, side (TWIST)",
                  ARM_BAND, V["ARM_T"], RIDE, F))

# The knee-servo bracket carries the servo's own reaction torque, not
# the robot's weight -- the thigh plate carries that. The crossing runs
# along Y and the torque is about Y, so this one twists too.
KNEE_TORQUE = 1110.0        # N.mm, the worst walking knee torque
rows.append(twist("knee bracket crossing (TWIST)",
                  LEG["CROSS_X1"] - LEG["CROSS_X0"], LEG["CROSS_H"],
                  1.0, KNEE_TORQUE))

worst = None
for name, Z, M, sigma, sf in rows:
    if sf < 1.0:        verdict = "BREAKS"
    elif sf < WANT_SF:  verdict = "holds, no margin"
    else:               verdict = "ok"
    if worst is None or sf < worst[1]:
        worst = (name, sf)
    print(f"{name:<34}{Z:>9.0f}{M:>9.0f}{sigma:>8.1f}{sf:>7.1f}   {verdict}")

print("-" * 72)
print(f"Weakest section: {worst[0]}, safety factor {worst[1]:.1f}")


# =====================================================================
# WILL THE JOINTS DEVELOP PLAY?
#
# A servo's output shaft is built for torque, not bending. So every
# driven joint here has a JOURNAL -- a printed collar turning in a
# printed bore -- whose only job is to take the bending instead. The one
# thing that decides whether it can is how LONG it is, and nothing else
# in this file looks at that: the thickness audit is a printability
# floor, and a 4 mm collar with a 5.7 mm wall sails through it. That is
# how the pitch journal came to react a 240 mm leg over 4 mm of bearing.
#
# Same load case as the bending check above. The side load at the foot
# is a moment at the joint; a journal of length L in a bore of diameter
# D carries it as a linearly varying pressure peaking at the ends:
#
#     p_max = 6 M / (D L^2)
#
# so it falls with the SQUARE of the length. PETG on PETG, barely
# moving, wants to stay well under about 10 MPa -- past that it cold
# flows under a standing load and the joint grows play, which shows up
# as a leg that wobbles before anything has actually broken.
# =====================================================================
JOURNAL_ALLOW = 10.0      # MPa, PETG on PETG, near-static
JOURNAL_D = V["JOINT_BORE_D"] - 2 * V["CLEAR"]

print()
print("=" * 72)
print("Journal bearing -- the collars that keep bending off the servo shafts")
print(f"p_max = 6M/(D.L^2), want under {JOURNAL_ALLOW:.0f} MPa "
      f"or the joint develops play")
print("=" * 72)
print(f"{'journal':<30}{'D mm':>7}{'L mm':>7}{'lever':>8}{'MPa':>8}   verdict")
print("-" * 72)

journals = [
    # name,                      length,             lever out to the foot
    ("hip pitch (thigh collar)", V["HIP_COLLAR_H"],  V["THIGH"] + V["SHANK"]),
    ("knee (shank collar)",      V["KNEE_COLLAR_H"], V["SHANK"]),
]
loose = []
for nm, L, lever in journals:
    pmax = 6.0 * (F * lever) / (JOURNAL_D * L * L)
    ok = pmax < JOURNAL_ALLOW
    if not ok:
        want = (6.0 * F * lever / (JOURNAL_D * JOURNAL_ALLOW)) ** 0.5
        loose.append((nm, L, pmax, want))
    print(f"{nm:<30}{JOURNAL_D:>7.1f}{L:>7.1f}{lever:>8.0f}{pmax:>8.1f}   "
          f"{'ok' if ok else 'COLD FLOWS'}")
print("-" * 72)
for nm, L, pmax, want in loose:
    print(f"  {nm}: {L:.1f} mm gives {pmax:.1f} MPa. Needs {want:.1f} mm. "
          f"Length is the only lever -- pressure falls with L squared.")
print()
print(f"Pushrod, as a strut: carries {ROD_F:.0f} N, buckles at {P_cr:.0f} N "
      f"-- safety factor {P_cr/ROD_F:.0f}")
if P_cr / ROD_F < 3.0:
    print("  THE ROD BUCKLES. Thicken it or shorten it.")
    sys.exit(1)
if worst[1] < 1.0:
    sys.exit(1)
