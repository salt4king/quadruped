include <params.scad>
use <servo.scad>
use <leg.scad>
use <shoulder.scad>

// Driven by check_screws.py. Puts a real screw where the build
// instructions say to put one, and intersects it with the part.
//
// Nothing here is arithmetic. check_screws.py works the length out
// from params.scad; this renders the screw as a solid in the part's
// own frame and asks the geometry whether it fits. The two can
// disagree, and when they did -- an M3x45 specified for a hole whose
// window closes at 40.5 mm -- only a check like this catches it.
//
// SEAT comes in as a parameter rather than being computed here: the
// roll screw's entry face depends on ARM_W, which lives in
// shoulder.scad, and `use` does not import variables. Reading it from
// here would silently give undef.

$fn = 32;
JOINT = "roll";      // "roll" | "pitch" | "knee"
TEST  = "shank";     // "shank" | "head" | "bore"
SEAT  = 40.5;        // the part's entry face, in its own frame
LEN   = 40;          // screw length, measured from its seat
BORE  = 0;           // counterbore sunk at that face

HEAD_D  = 5.5;       // M3 socket cap
HEAD_H  = 3.0;
SHANK_D = 3.0;       // under M3_CLEAR, so this tests the hole, not its wall
R       = 7.5;       // the horn's bolt circle

// Drawn pointing +Z INTO the part from its face at z = 0. The seat --
// where the head lands -- is BORE deep, so each feature places itself
// against that; an outer translate cannot do it, because the
// counterbore probe is measured from the FACE and the other two from
// the SEAT. Shifting all three together put the counterbore probe
// past the end of the counterbore and reported it missing.
module screw() {
    if (TEST == "shank")
        for (a = [0:90:359]) rotate([0, 0, a])
            translate([R, 0, BORE]) cylinder(d = SHANK_D, h = LEN);
    else if (TEST == "head")
        for (a = [0:90:359]) rotate([0, 0, a])
            translate([R, 0, BORE - HEAD_H]) cylinder(d = HEAD_D, h = HEAD_H);
    else   // the counterbore has to be there for the head to sink into
        for (a = [0:90:359]) rotate([0, 0, a])
            translate([R, 0, 0.02]) cylinder(d = HEAD_D + 0.4, h = BORE - 0.04);
}

if (JOINT == "roll")
    intersection() {
        shoulder(1);
        // enters at the arm's outer face, travelling back to the servo
        translate([SEAT, 0, 0]) rotate([0, -90, 0]) screw();
    }
else if (JOINT == "pitch")
    intersection() {
        thigh();
        translate([0, SEAT, 0]) rotate([-90, 0, 0]) screw();
    }
else
    intersection() {
        crank();
        translate([0, SEAT, 0]) rotate([-90, 0, 0]) screw();
    }
