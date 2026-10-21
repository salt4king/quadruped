include <params.scad>
use <servo.scad>
use <leg.scad>

// Leg assembled in the standing pose, four-bar drawn, viewed side-on so
// it can be held against the concept sketch.
//
// PLACEMENT MATHS, written out because an earlier version had a sign
// wrong and silently rendered the shank floating in space next to the
// knee it was supposed to pivot on.
//
// OpenSCAD's rotate([0,a,0]) takes the -Z unit vector to
//     (-sin a, 0, -cos a)
// so a link of length L drawn down -Z and rotated by a ends at
//     L * (-sin a, 0, -cos a)
// The controller's joint angles run the other way round (its +q swings
// the foot toward +X), hence the negations below. Mixing the two
// conventions is the whole bug.

HIP_Q  = -46.6;    // standing pose, from quadruped/robot.py
KNEE_Q =  84.6;

THIGH_ROT = -HIP_Q;              // rotation applied to the thigh
SHANK_ROT = -(HIP_Q + KNEE_Q);   // shank's ABSOLUTE rotation

function down(a)  = [-sin(a), 0, -cos(a)];
function up(a)    = [ sin(a), 0,  cos(a)];

KNEE    = THIGH * down(THIGH_ROT);
ARMDIR  = up(SHANK_ROT);              // upper arm points opposite the shank
CR_TIP  = CRANK * ARMDIR;             // crank tip, about the pitch axis
ARM_TIP = KNEE + UPPER_ARM * ARMDIR;  // pushrod's far pivot

// The parallelogram, checked rather than asserted by eye: with
// CRANK == UPPER_ARM the pushrod vector reduces to the thigh vector, so
// its length must come out as THIGH exactly.
echo(str("pushrod length = ", norm(ARM_TIP - CR_TIP), "  (must equal THIGH = ", THIGH, ")"));
echo(str("foot height    = ", -(KNEE + SHANK*down(SHANK_ROT))[2],
         "  (must equal STAND_HEIGHT = ", STAND_HEIGHT, ")"));

// Angle that takes +X (a rod's own axis) onto a given link direction.
function rod_a_down(a) = 90 + a;
function rod_a_up(a)   = a - 90;

module leg_assembly() {
    rotate([0, THIGH_ROT, 0]) color("SteelBlue") thigh();
    translate(KNEE) rotate([0, SHANK_ROT, 0]) color("CadetBlue") shank();

    translate([0, LINK_DY, 0]) rotate([0, rod_a_up(SHANK_ROT), 0])
        rotate([-90, 0, 0]) color("Goldenrod") crank();

    translate(CR_TIP + [0, ROD_Y, 0]) rotate([0, rod_a_down(THIGH_ROT), 0])
        rotate([-90, 0, 0]) color("Tomato") pushrod();

    color("DimGray", 0.75) rotate([-90, 0, 0]) ds3225();
    color("DimGray", 0.55)
        translate([0, CRANK_Y, 0]) rotate([90, 0, 0]) ds3225();
}

module reference() {
    color("Tomato", 0.18) translate([0, 0, -STAND_HEIGHT])
        cube([300, 70, 0.8], center = true);
}

leg_assembly();
reference();
