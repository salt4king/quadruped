"""What to print and what to buy.

Read off the exported STLs and params.scad rather than typed, so it
cannot drift from the model the way a hand-kept list does. That goes for
the FASTENER LENGTHS too: they moved twice while the hip stack was being
fixed, and a typed list would still be quoting the old ones.

Run:  python3 cad/bom.py
"""

from __future__ import annotations

import re
import struct
from pathlib import Path

CAD = Path(__file__).resolve().parent


def scan(name):
    src = (CAD / name).read_text()
    return {k: float(v) for k, v in
            re.findall(r"^(\w+)\s*=\s*([-\d.]+)\s*;", src, re.M)}


P = scan("params.scad")
SHOULDER = scan("shoulder.scad")

PETG, INFILL = 1.27, 0.50      # g/cm3, and the effective solid fraction

# LEFT AND RIGHT. The assembly mirrors each leg in Y for the right
# hand side, so a part serves both sides only if it has a mirror plane
# of its own. Measured by intersecting each part with its reflection:
# chassis, roof, pushrod and filler are symmetric in Y, the shoulder in
# Z, and thigh/shank/crank in nothing -- those three are 2 + 2.
PARTS = [
    ("chassis_front", 1, "167 x 152; the roll servo mounts are part of it"),
    ("chassis_rear", 1, "143 x 152; laps under the front half"),
    ("shoulder", 4, "symmetric in Z -- one print, turned over for the right"),
    ("thigh", 2, "LEFT. Carries the knee servo and the knee bearing"),
    ("thigh_right", 2, "RIGHT. Mirrored; it has no mirror plane of its own"),
    ("shank", 2, "LEFT. Beam, rocker lug and foot in one"),
    ("shank_right", 2, "RIGHT. 99.0% self-overlapping, and still not the same part"),
    ("crank", 2, "LEFT. On the knee servo's horn"),
    ("crank_right", 2, "RIGHT. The hub's D-flat does not survive a reflection"),
    ("pushrod", 4, "crank tip to rocker tip; symmetric, one print"),
    ("filler", 12, "closes every servo pocket -- one part, all three mounts"),
    ("roof", 1, "snaps over the rim; print it upside down, no support"),
]


def volume(path: Path) -> float:
    d = path.read_bytes()
    n = struct.unpack("<I", d[80:84])[0]
    v = 0.0
    for i in range(n):
        o = 84 + i * 50 + 12
        a, b, c = [struct.unpack("<fff", d[o + k * 12:o + k * 12 + 12])
                   for k in range(3)]
        v += (a[0] * (b[1] * c[2] - c[1] * b[2])
              - a[1] * (b[0] * c[2] - c[0] * b[2])
              + a[2] * (b[0] * c[1] - c[0] * b[1])) / 6.0
    return abs(v) / 1000.0


# ---------------------------------------------------------------------
# FASTENER LENGTHS, from the stack.
#
# A SERVO HORN SCREW passes through the driven part at the bolt circle
# and threads into the horn, so its length is the part's thickness there
# plus whatever it can get in the horn. The horn is 2 mm, so that
# engagement is shallow by nature: use thread-lock and do not torque
# these down hard.
# ---------------------------------------------------------------------
STD = [6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50]   # lengths you can buy


def standard(x: float) -> int:
    for n in STD:
        if n >= x - 0.01:
            return n
    raise SystemExit(
        f"no stock length reaches {x:.1f} mm -- the list stops at "
        f"{STD[-1]}. It used to fall through to the longest one and "
        f"quietly spec a screw too short to reach.")


HORN_TH = P["SERVO_HORN_TH"]       # 2.0 -- the only thread in the joint
POCKET = HORN_TH + P["CLEAR"]      # pocket sunk into the part's face


def horn_screw(outer: float, face: float, bore: float = 0.0) -> int:
    """Longest stock screw that bites the horn without reaching the servo.

    The horn is 2 mm thick and the servo's output boss is immediately
    behind it, so the usable window is exactly that 2 mm:

        L > |outer - horn's near face|     or it does not bite
        L <= |outer - horn's far face|     or it bottoms on the servo

    This used to round UP to the next stock length, which is the wrong
    direction when the far end is a servo. It put M3x45 on the roll
    horns, where the window closes at 40.5 -- 4.5 mm of screw into the
    servo's boss, and the shoulder would never have pulled down.
    `bore` is any counterbore sunk at the entry face.
    """
    span = abs(face - outer) - bore
    lo, hi = span - HORN_TH, span
    fits = [n for n in STD if lo < n <= hi]
    if not fits:
        raise SystemExit(
            f"no stock screw lands in the horn: the window is "
            f"{lo:.1f} < L <= {hi:.1f} mm. Counterbore the entry face "
            f"to shift it onto a stock size.")
    return max(fits)


#   pitch  through the thigh, inboard face out to the hip collar's pocket
#   knee   through the crank, arm's inboard face out to the hub's pocket
#   roll   through the shoulder's roll collar and its boss
PITCH_SCREW = horn_screw(P["THIGH_IN"], P["PITCH_FACE"], bore=1.5)
KNEE_SCREW = horn_screw(P["LINK_IN"], P["KNEE_FACE"], bore=1.1)
# The roll screws pass through the WHOLE shoulder now -- collar, boss
# and the arm's tongue, which runs out to SWEEP_CLR + ARM_W + 0.5. So
# this one is measured from the horn's seat to that outer face, not
# from a part's inboard face like the other two.
ROLL_FACE = P["SWEEP_CLR"] + SHOULDER["ARM_W"] + 0.5
ROLL_SCREW = horn_screw(ROLL_FACE, 0.0)
MOUNT_SCREW = standard(3.0 + 2.5 + 6.0)            # filler + servo ear + insert

# Every joint on the robot and what holds it. Counts are per robot.
JOINTS = [
    ("roll servo -> shoulder", 4,
     f"4x M3x{ROLL_SCREW} into the horn",
     "horn pocket + 4 holes on a 15 mm circle, through the roll collar"),
    ("pitch servo -> thigh", 4,
     f"4x M3x{PITCH_SCREW} into the horn",
     "same pattern, through the thigh's hip collar -- which is SOLID, so "
     "the horn seats on a face instead of hanging off the screws"),
    ("knee servo -> crank", 4,
     f"4x M3x{KNEE_SCREW} into the horn",
     "same pattern, through the crank's hub"),
    ("thigh -> shank (the knee)", 4,
     "1x M4 shoulder bolt, 5 mm dia x 10 mm",
     "plain shank turns in the thigh, threads into an M4 insert in the shank"),
    ("crank -> pushrod", 4,
     "1x M3 shoulder bolt, 4 mm dia x 6 mm",
     "rod turns on the shoulder, threads into an M3 insert in the crank"),
    ("pushrod -> rocker", 4,
     "1x M3 shoulder bolt, 4 mm dia x 6 mm",
     "same again, into the shank's rocker"),
    ("chassis front -> rear", 1,
     "6x M3x12 + M3 nut, plus two printed snap latches",
     "halved lap: click the halves together, then bolt. The latches "
     "hold it square while you do -- they are not the joint"),
    ("servo -> bracket", 12,
     f"4x M3x{MOUNT_SCREW} + M3 heat-set insert",
     "through the filler, through the servo's ear, into the bracket"),
]

# What to order. Per robot, no spares.
BUY = [
    (f"M3 x {ROLL_SCREW} socket cap", 4 * 4, "roll servo horns"),
    (f"M3 x {PITCH_SCREW} socket cap", 4 * 4, "pitch servo horns"),
    (f"M3 x {KNEE_SCREW} socket cap", 4 * 4, "knee servo horns"),
    (f"M3 x {MOUNT_SCREW} socket cap", 12 * 4, "servo mounts, through the fillers"),
    ("M3 x 12 socket cap + M3 nut", 6, "the chassis lap joint, through both halves"),
    ("M3 shoulder bolt, 4 mm dia x 6 mm", 8, "linkage pivots, both ends of each rod"),
    ("M4 shoulder bolt, 5 mm dia x 10 mm", 4, "the knee pins"),
    # STATE THE OUTSIDE DIAMETER, not just the thread. A heat-set insert
    # grips because the hole is UNDER its OD and the brass displaces
    # plastic into the knurl -- so the OD is the dimension that has to
    # match, and M3 inserts are sold in both 4.0 and 4.6 mm OD. A 4.0 OD
    # insert in this model's 4.0 mm hole is a slip fit: it drops in and
    # spins the first time a screw is tightened. "M3 brass insert" was
    # not enough information to buy from.
    (f"M3 heat-set insert, {P['M3_INSERT'] + 0.6:.1f} mm OD", 12 * 4 + 8,
     f"servo mounts and linkage pivots. The hole is {P['M3_INSERT']:.1f} -- "
     f"buy the OD that wants it"),
    (f"M4 heat-set insert, {P['M4_INSERT'] + 0.5:.1f}-{P['M4_INSERT'] + 0.7:.1f} mm OD", 4,
     f"knee pins, in the shanks. The hole is {P['M4_INSERT']:.1f}"),
    ("DS3225 servo, 25 kg.cm", 12, "or any standard-size 25 kg.cm equivalent"),
    ("2S LiPo, ~5000 mAh", 1, "140 x 46 mm, strapped through the spine"),
]


# ---------------------------------------------------------------------
# ELECTRONICS.
#
# NO SERVO HUB. A PCA9685 is the usual answer and it is not needed here:
# a Teensy 4.1 has 35 hardware PWM pins, so all twelve servos hang off
# it directly with better timing and far better resolution than 12 bits
# over I2C gives. The hub only earns its place if a Linux board is
# driving the servos itself, because a Pi's GPIO PWM is software-timed
# and jitters.
#
# TWO TIERS, IF AI COMES LATER. Keep the MCU for the 200 Hz loop and add
# an SBC above it for perception. Linux is not deterministic and a servo
# deadline should not depend on what a vision model is doing. So the
# Teensy is not a stopgap -- it is the part you keep.
#
# CURRENT, from the torque the gait actually produces rather than from
# the stall rating. See tests/test_torque.py.
STALL_TQ, STALL_I = 25.0, 2.6      # kg.cm, A at 6.8 V -- CHECK YOUR SERVO
RMS_TQ, PEAK_TQ = 6.0, 15.2        # kg.cm, measured over a gait cycle
NSERVO = 12

A_RMS = RMS_TQ / STALL_TQ * STALL_I * NSERVO
A_PEAK = PEAK_TQ / STALL_TQ * STALL_I * 3 + RMS_TQ / STALL_TQ * STALL_I * 9
A_STALL = STALL_I * NSERVO

ELECTRONICS = [
    ("Teensy 4.1", 1,
     "35 PWM pins -- drives all 12 servos with no hub. Hardware float, "
     "which the IK wants: trig and roots, twelve times a tick at 200 Hz"),
    ("BNO055 IMU (or ICM-20948)", 1,
     "the robot BALANCES; the body-pose solver needs attitude. The BNO "
     "fuses on-chip, which saves writing a filter for the first build"),
    ("6 V UBEC, 15 A cont. / 30 A burst", 1,
     f"NOT optional: a 2S pack is 7.4-8.4 V and the servo is rated "
     f"4.8-6.8. Sized for {A_PEAK:.0f} A transient -- the 3 A hobby "
     f"UBECs will cook"),
    ("4700 uF 16 V low-ESR electrolytic", 2,
     "across the servo rail. Twelve servos starting together is a "
     "current step the regulator alone will not hold up through"),
    ("2S LiPo, 5000 mAh", 1,
     "140 x 46 mm, strapped through the spine"),
    ("XT60 pair", 1, "battery to the distribution bus"),
    ("Switch, 30 A", 1, "sized for the stalled case, not the walking one"),
    ("Fuse + holder, 30 A", 1, "between the pack and everything else"),
    ("Servo extension lead, 300 mm", 12, "hip to the boards, one per servo"),
    ("Hookup wire, 16 AWG", 2, "metres, red and black -- the servo rail"),
    ("Hookup wire, 24 AWG", 1, "metre -- signal, to the Teensy"),
]

OPTIONAL = [
    ("Raspberry Pi 5, 8 GB", "the AI tier, when you get there. Talks to "
     "the Teensy over USB serial; it does NOT drive servos"),
    ("Jetson Orin Nano", "instead of the Pi, if you want real NN "
     "throughput rather than classical CV"),
    ("PCA9685 16-channel hub", "ONLY if you drop the MCU and drive the "
     "servos from a Pi. One, not two -- 16 channels covers 12. Run it "
     "at 333 Hz: at 50 Hz its 12 bits work out to 0.66 deg a step, "
     "which is 2.8 mm at the foot"),
]


def main() -> None:
    print("\n" + "=" * 78)
    print("  PRINT")
    print("=" * 78)
    print(f"{'part':<16}{'off':>5}{'cm3 ea':>9}{'g ea':>8}{'g total':>9}   note")
    tot = 0.0
    for name, n, note in PARTS:
        f = CAD / "stl" / f"{name}.stl"
        if not f.exists():
            print(f"{name:<16}{n:>5}   -- not exported, run cad/export.sh")
            continue
        v = volume(f)
        g = v * PETG * INFILL
        tot += g * n
        print(f"{name:<16}{n:>5}{v:>9.1f}{g:>8.1f}{g*n:>9.1f}   {note}")
    print(f"{'':<16}{'':>5}{'':>9}{'printed':>8}{tot:>9.1f} g"
          f"   PETG at {INFILL*100:.0f}% effective")

    print("\n" + "=" * 78)
    print("  HOW EACH JOINT IS MADE")
    print("=" * 78)
    for joint, n, fastener, how in JOINTS:
        print(f"\n  {joint}   x{n}")
        print(f"      {fastener}")
        print(f"      {how}")

    print("\n" + "=" * 78)
    print("  BUY  --  per robot, no spares")
    print("=" * 78)
    print(f"  {'qty':>4}   {'part':<38}what for")
    print("  " + "-" * 72)
    # Merge lines that are the same part. Three of the horn sizes work
    # out the same, and ordering "16 of M3x20" twice is how you end up
    # short of them.
    merged: dict[str, tuple[int, list[str]]] = {}
    for name, n, what in BUY:
        qty, uses = merged.get(name, (0, []))
        merged[name] = (qty + n, uses + [what])
    for name, (n, uses) in merged.items():
        print(f"  {n:>4}   {name:<38}{', '.join(uses)}")

    print("\n" + "=" * 78)
    print("  ELECTRONICS  --  no servo hub; the MCU drives them direct")
    print("=" * 78)
    print(f"  {'qty':>4}   {'part':<36}what for")
    print("  " + "-" * 72)
    for name, n, what in ELECTRONICS:
        print(f"  {n:>4}   {name:<36}{what}")

    print(f"\n  Current, from the gait rather than the stall rating:")
    print(f"      {A_RMS:>5.1f} A   all twelve, continuous")
    print(f"      {A_PEAK:>5.1f} A   realistic transient")
    print(f"      {A_STALL:>5.0f} A   all twelve stalled, which is what the")
    print(f"              switch and fuse have to survive")

    print("\n  LATER, OR INSTEAD:")
    for name, what in OPTIONAL:
        print(f"    {name}")
        print(f"        {what}")

    print(f"\n  Insert pilots in the model: M3 {P['M3_INSERT']:.1f} mm, "
          f"M4 {P['M4_INSERT']:.1f} mm.")
    print("  CHECK YOUR OWN INSERTS -- pilot sizes vary by brand, and")
    print("  reprinting four thighs because the inserts spin is a bad day.")
    print("  CHECK YOUR OWN SERVO too: the 40 x 20 x 40.4 body is near")
    print("  universal, the 49.5 mm ear spacing is not.")


if __name__ == "__main__":
    main()
