include <params.scad>
use <servo.scad>
use <chassis.scad>
use <shoulder.scad>
use <leg.scad>
use <hip.scad>

// Pairwise interference check for ONE NAMED LEG. SX/SY pick it, and the
// mirrors below are copied from robot.scad's one_leg() exactly -- including
// the fore/aft mirror that applies to the shoulder and NOT to the links.
//
// It used to hardcode the front-left leg, "because all the assembly
// mirrors are the identity there". That is true and it is precisely why
// checking only that leg proves nothing about the other three.

$fn = 20;
// Pose. Defaults to standing; check_fit.py overrides KNEE_Q and HIP_Q to
// walk the whole envelope, because a LINKAGE can be clear at one pose and
// foul at another -- that is the failure mode a single static check cannot
// see, and the one a mechanism actually has.
ROLL_Q = POSE_ROLL; HIP_Q = POSE_HIP; KNEE_Q = POSE_KNEE;
THIGH_ROT = -HIP_Q;
function along(a, L) = [L*sin(a), 0, -L*cos(a)];
CRANK_Q   = KNEE_Q + LINK_PHI;
CRANK_TIP = [0, 0, -CRANK_C] + along(CRANK_Q, LINK_A);
SX = 1; SY = 1;                 // which leg: +1 front/left, -1 rear/right
HIP = [SX*BODY_LEN/2, SY*BODY_WID/2, HIP_Z];

// The shoulder and the roll servo mirror fore/aft; the links do not.
module at_hip()  { translate(HIP) mirror([0, SY > 0 ? 0 : 1, 0]) children(); }
// No fore/aft mirror: every leg is the same assembly now.
module at_shou() { translate(HIP) mirror([0, SY > 0 ? 0 : 1, 0]) children(); }

module in_thigh()    { at_hip() rotate([ROLL_Q,0,0]) rotate([0,THIGH_ROT,0]) children(); }

module p_chassis()   { chassis_plate(); }
module p_roof()      { roof(); }
module p_shoulder()  { at_shou() rotate([ROLL_Q,0,0]) shoulder(1); }
module p_rollservo() { roll_frame(SX, SY) ds3225(); }
module p_rollfill()  { roll_frame(SX, SY) ds3225_filler(WALL, 1); }
module p_pitchfill() { at_hip() rotate([ROLL_Q,0,0])
                         translate([0,PITCH_FACE,0]) rotate([90,0,0])
                           ds3225_filler(WALL, 1); }
module p_kneefill()  { in_thigh() translate([0,0,-CRANK_C])
                         translate([0,KNEE_FACE,0]) rotate([90,0,0])
                           ds3225_filler(WALL, 1); }
module p_pitchservo(){ at_hip() rotate([ROLL_Q,0,0])
                         translate([0,PITCH_FACE,0]) rotate([90,0,0]) ds3225(); }
module p_thigh()     { in_thigh() thigh(); }
module p_kneeservo() { in_thigh() translate([0,0,-CRANK_C])
                         translate([0,KNEE_FACE,0]) rotate([90,0,0]) ds3225(); }
module p_crank()     { in_thigh() translate([0,0,-CRANK_C])
                         rotate([0,-CRANK_Q,0]) crank(); }
module p_pushrod()   { in_thigh() translate(CRANK_TIP) pushrod(); }
module p_shank()     { in_thigh() translate([0,0,-THIGH])
                         rotate([0,-KNEE_Q,0]) shank(); }

module named(n) {
    if      (n=="chassis")    p_chassis();
    else if (n=="roof")       p_roof();
    else if (n=="shoulder")   p_shoulder();
    else if (n=="rollservo")  p_rollservo();
    else if (n=="pitchservo") p_pitchservo();
    else if (n=="kneeservo")  p_kneeservo();
    else if (n=="thigh")      p_thigh();
    else if (n=="shank")      p_shank();
    else if (n=="crank")      p_crank();
    else if (n=="pushrod")    p_pushrod();
    else if (n=="rollfill")   p_rollfill();
    else if (n=="pitchfill")  p_pitchfill();
    else if (n=="kneefill")   p_kneefill();
}

MODE = "pair"; A = "thigh"; B = "shoulder";
// "solo" renders A on its own, so check_fit.py can prove the renderer
// works before it believes a column of 0.000 results.
if (MODE == "solo") { named(A); }
else if (MODE == "pair") { intersection() { named(A); named(B); } }
