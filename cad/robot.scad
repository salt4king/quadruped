include <params.scad>
use <servo.scad>
use <chassis.scad>
use <shoulder.scad>
use <leg.scad>
use <hip.scad>

// Whole robot, standing. Direct drive: three servos per leg, each on its
// own axis. Parts carry their own hip-stack Y, so they are placed at the
// hip and nothing is offset by hand.
//
// Leg links are never mirrored fore/aft -- the roll servos point outboard
// at both ends so the shoulder mirrors in X, but mirroring the thigh with
// it would flip the rear legs to knees-forward.

$fn = 18;
ROLL_Q = POSE_ROLL; HIP_Q = POSE_HIP; KNEE_Q = POSE_KNEE;
THIGH_ROT = -HIP_Q;

// Everything in the thigh's frame is measured from the thigh line, so one
// convention covers all of it: a part at `a` degrees from the thigh is
// rotate([0,-a,0]), and a length L in that direction ends at along(a,L).
// The shank sits at the knee angle; the crank and the rocker sit at the
// knee angle plus LINK_PHI, which is the whole content of the linkage.
function along(a, L) = [L*sin(a), 0, -L*cos(a)];
CRANK_Q   = KNEE_Q + LINK_PHI;
CRANK_TIP = [0, 0, -CRANK_C] + along(CRANK_Q, LINK_A);

module leg_links() {
    rotate([0, THIGH_ROT, 0]) {
        color("SteelBlue") thigh();
        // Knee servo, carried by the THIGH at C -- not by the knee.
        color("DimGray") translate([0, 0, -CRANK_C])
            translate([0, KNEE_FACE, 0]) rotate([90,0,0]) ds3225();
        color("Tomato") translate([0, 0, -CRANK_C])
            translate([0, KNEE_FACE, 0]) rotate([90,0,0]) ds3225_filler(WALL, 1);
        color("Goldenrod") translate([0, 0, -CRANK_C])
            rotate([0, -CRANK_Q, 0]) crank();
        // The rod needs no rotation at all: in a parallelogram it stays
        // parallel to the crank axis -> knee axis line, which IS the thigh.
        color("Peru") translate(CRANK_TIP) pushrod();
        translate([0, 0, -THIGH]) rotate([0, -KNEE_Q, 0])
            color("CadetBlue") shank();
    }
    color("DimGray") translate([0, PITCH_FACE, 0]) rotate([90,0,0]) ds3225();
    color("Tomato") translate([0, PITCH_FACE, 0]) rotate([90,0,0]) ds3225_filler(WALL, 1);
}

module one_leg(sx, sy) {
    // The roll servo and its filler are placed by hip.scad's frame, the
    // same one the chassis cuts the pocket with -- not by a local copy
    // of the transform, which is how the assembly and the chassis used
    // to disagree about which way the servo faced.
    color("DimGray") roll_frame(sx, sy) ds3225();
    color("Tomato")  roll_frame(sx, sy) ds3225_filler(WALL, 1);
    translate([sx*BODY_LEN/2, sy*BODY_WID/2, HIP_Z])
        mirror([0, sy > 0 ? 0 : 1, 0]) rotate([ROLL_Q,0,0]) {
            color("Silver") shoulder(1);
            leg_links();
        }
}

GROUP = "all";
module g_each(which) {
    if (which == "servo" || which == "filler")
        for (sx = [-1,1], sy = [-1,1]) roll_frame(sx, sy) {
            if (which == "servo") ds3225(); else ds3225_filler(WALL, 1);
        }
    for (sx = [-1,1], sy = [-1,1])
        translate([sx*BODY_LEN/2, sy*BODY_WID/2, HIP_Z]) {
            if (which == "shoulder")
                mirror([0,sy>0?0:1,0]) rotate([ROLL_Q,0,0]) shoulder(1);
            else if (which == "servo") {
                mirror([0,sy>0?0:1,0]) rotate([ROLL_Q,0,0]) {
                    translate([0, PITCH_FACE, 0]) rotate([90,0,0]) ds3225();
                    rotate([0,THIGH_ROT,0]) translate([0,0,-CRANK_C])
                        translate([0, KNEE_FACE, 0]) rotate([90,0,0]) ds3225();
                }
            }
            else if (which == "filler") {
                mirror([0,sy>0?0:1,0]) rotate([ROLL_Q,0,0]) {
                    translate([0, PITCH_FACE, 0]) rotate([90,0,0]) ds3225_filler(WALL, 1);
                    rotate([0,THIGH_ROT,0]) translate([0,0,-CRANK_C])
                        translate([0, KNEE_FACE, 0]) rotate([90,0,0]) ds3225_filler(WALL, 1);
                }
            }
            else mirror([0,sy>0?0:1,0]) rotate([ROLL_Q,0,0]) rotate([0,THIGH_ROT,0]) {
                if (which == "thigh")   thigh();
                if (which == "crank")   translate([0,0,-CRANK_C])
                                            rotate([0,-CRANK_Q,0]) crank();
                if (which == "pushrod") translate(CRANK_TIP) pushrod();
                if (which == "shank")   translate([0,0,-THIGH])
                                            rotate([0,-KNEE_Q,0]) shank();
            }
        }
}

if (GROUP == "all") {
    color("LightSteelBlue") chassis_plate();
    for (sx = [-1,1], sy = [-1,1]) one_leg(sx, sy);
}
else if (GROUP == "chassis")  chassis_plate();
else if (GROUP == "roof")     roof();
else if (GROUP == "shoulder") g_each("shoulder");
else if (GROUP == "thigh")    g_each("thigh");
else if (GROUP == "shank")    g_each("shank");
else if (GROUP == "servo")    g_each("servo");
else if (GROUP == "crank")    g_each("crank");
else if (GROUP == "pushrod")  g_each("pushrod");
else if (GROUP == "filler")   g_each("filler");
