include <params.scad>
use <servo.scad>

// ===================================================================
// SHOULDER -- rotates on the roll servo, carries the pitch servo.
//
// The pitch servo's BODY points OUTBOARD, away from the robot. Pointing
// it inboard put its full 40.4 mm length between the roll axis and the
// leg, which is what drove HIP_OFFSET to 102 mm and splayed the legs
// out from under the robot.
//
//   PITCH_FACE..PITCH_OUT   servo, in its housing
//   HIP_COLLAR..PITCH_FACE  bore; the thigh's collar runs in it
//   roll collar             on the X axis, outboard of the servo face
//
// THE ARM HAS A SIDE, AND IT IS NOT THE SAME SIDE AS THE MIRROR.
//
// The shoulder is mirrored fore/aft in the assembly so its roll collar
// meets the roll servo's spline, which also mirrors. The THIGH is not
// mirrored -- it must not be, or the rear knees would point the wrong
// way. So a one-sided arm that clears the thigh on the front legs ends
// up on the thigh's own side at the back, and buries itself in the
// plate: 0.597 cm3 per rear leg, at the standing pose.
//
// `arm` undoes that. It picks the side the arm is DRAWN on, so that
// after the assembly's mirror it always lands forward of the thigh.
// Pass the leg's fore/aft sign. The roll boss is symmetric about the
// axis so the arm can leave from either side of it.
// ===================================================================

BORE_D   = 36.0;
BOSS_D   = BORE_D + 2*WALL + 4;
COLLAR_D = 36;
RCOLL_H  = 10;
RBOSS_D  = COLLAR_D + 2*WALL;
// The arm has to cross the thigh plate's band (THIGH_IN..THIGH_OUT) on
// its way to the pitch housing, and the thigh sweeps a 21 mm disc about
// the pitch axis. So it crosses OUTSIDE that disc, at x >= SWEEP_CLR.
ROLL_BOSS_X = 11;
// SWEEP_CLR, SHELL_Z, ARM_Y0 and ARM_Y1 come from params.scad -- see
// SHOULDER ARM there. chassis.scad reads the same numbers to cut room.
SHELL_X  = SERVO_L + 2*WALL;
// The arm rail's section. It is the single load path between the roll
// collar and everything below it, and a fore/aft scuff at the foot
// TWISTS it rather than bending it -- the arm runs along Y and that
// moment is about Y. An open rectangle is poor in torsion, and at
// 9 x 14 it came out at safety factor 1.4, the weakest thing on the
// robot. Torsion goes as alpha*a*b^2 with b the SHORT side, so the
// width is worth more than the depth here.
//
// It can only grow OUTBOARD in x. The thigh sweeps a 21 mm disc about
// the pitch axis and the arm has to cross outside it, so the inner edge
// stays at SWEEP_CLR; the rail grows away from the axis instead.
// 15, which is as wide as it can be. The rail's inner edge is pinned
// at SWEEP_CLR by the thigh's 21 mm sweep disc, and the chassis only
// cuts room out to SWEEP_CLR + 16 -- so 25.5 to 41, and 15.5 of rail
// fits between them.
ARM_W    = 15;     // the arm rail's width, across its run
ARM_X    = SWEEP_CLR + ARM_W/2 + 0.5;

module shoulder(arm = 1) {
    difference() {
        union() {
            // The housing is OFF-CENTRE, by the servo's own shaft
            // offset -- so it has to follow the servo, which belongs to
            // the leg and does not mirror fore/aft. Mirroring the
            // housing without the servo put them 20 mm apart on the
            // rear legs and left the servo hanging out of its own box.
            scale([arm, 1, 1]) translate([0, PITCH_FACE, 0])
                rotate([90,0,0]) ds3225_mount_solid(WALL);
            // Boss the thigh's collar runs in, reaching inboard.
            translate([0, HIP_COLLAR, 0]) rotate([-90,0,0])
                cylinder(d = BOSS_D, h = PITCH_FACE - HIP_COLLAR);
            // Roll collar and boss stay on +X: the collar meets the roll
            // servo's horn, and the servo mirrors with the shoulder.
            // (A symmetric boss was tried, to let the arm leave from
            //  either side -- it swallowed the roll servo, 9.5 cm3.)
            // The boss starts exactly where the collar ends. ROLL_BOSS_X
            // was 11 against a 10 mm collar -- a 1 mm gap, so the collar
            // was a separate disc floating inside the part. The rear
            // shoulder got away with it because its arm happens to cross
            // the collar; the front one printed in two pieces.
            rotate([0,90,0]) cylinder(d = COLLAR_D, h = RCOLL_H);
            translate([RCOLL_H, 0, 0]) rotate([0,90,0])
                cylinder(d = RBOSS_D, h = ROLL_BOSS_X + 9 - RCOLL_H);
            // Arm, routed at x >= ROLL_BOSS_X to clear both the roll
            // servo's body and the chassis's own collar boss.
            // THE ARM, and the band it crosses in.
            //
            // It has to get from the roll axis out to the pitch housing,
            // which means crossing the thigh plate's own band (THIGH_IN
            // to THIGH_OUT). Two things are in the way and they are on
            // opposite sides, so there is exactly one route:
            //
            //   the thigh sweeps everything BEHIND the hip, and below
            //   it -- so the crossing goes in front, at |x| = SWEEP_CLR
            //
            //   the roll servo's body fills the other side, but only
            //   over its own 20 mm width -- so the arm leaves the boss
            //   OUTBOARD of that, past ARM_Y0, before it turns
            //
            // `arm` flips only the x half of that, so the crossing lands
            // in front of the thigh on the rear legs too.
            // The tongue: a straight run from the boss across to the
            // crossing point, in the band ARM_Y0..ARM_Y1 so it passes
            // outboard of the roll servo's 20 mm width. On a rear leg
            // it has to cross the roll axis to reach the forward side,
            // and the chassis cuts its swept arc out of the collar boss
            // to let it -- see chassis.scad.
            hull() {
                translate([ROLL_BOSS_X + ARM_W/2, (ARM_Y0 + ARM_Y1)/2, 0])
                    cube([ARM_W, ARM_Y1 - ARM_Y0, ARM_T], center = true);
                translate([arm*ARM_X, (ARM_Y0 + ARM_Y1)/2, 0])
                    cube([ARM_W, ARM_Y1 - ARM_Y0, ARM_T], center = true);
            }
            translate([arm*ARM_X, (ARM_Y1 + PITCH_FACE + 10)/2, 0])
                cube([ARM_W, PITCH_FACE + 10 - ARM_Y1, SHELL_Z], center = true);
            // Bridge from the rail across to the housing. Reaches the
            // rail's OUTER edge, so widening the rail cannot leave a
            // step the bridge no longer covers.
            translate([arm*(ARM_X + ARM_W/2 + 13)/2, (PITCH_FACE + PITCH_OUT)/2, 0])
                cube([ARM_X + ARM_W/2 - 13, PITCH_OUT - PITCH_FACE, SHELL_Z],
                     center = true);
        }
        // Pitch servo: horn inboard, body outboard. Holes come from
        // ds3225_holes() in the servo's own frame so they cannot drift.
        scale([arm, 1, 1]) translate([0, PITCH_FACE, 0]) rotate([90,0,0]) {
            ds3225_mount_cuts(SERVO_FIT);
            // AND THE LEAD HAS TO GET PAST THE ARM.
            //
            // ds3225_mount_cuts clears the first 25 mm, which is enough
            // in the chassis and the thigh -- nothing of theirs is out
            // there. The shoulder is the exception: the bridge from the
            // rail across to the housing runs x = 13 to 40.5, straight
            // through the cable's path. Traced with a 90 mm run, it put
            // 0.55 cm3 of solid arm in front of the pitch servo's lead.
            //
            // 50 mm clears the bridge. `grow` widens it to 14 x 14 so
            // it breaks out of the housing's outer face rather than
            // leaving a 1 mm wall there, which also means the cable can
            // be laid in rather than threaded through.
            ds3225_wire(50, 2);
        }
        translate([0, HIP_COLLAR - 0.01, 0]) rotate([-90,0,0])
            cylinder(d = BORE_D, h = PITCH_FACE - HIP_COLLAR + 0.02);
        // Roll horn: same stepped cut as the other two, but the screw
        // holes run the WHOLE way out now.
        //
        // They used to stop at RCOLL_H + 16 = 26, which was past the
        // boss and past everything there was. Then the tongue was
        // rooted to the roll axis and grew out to x = 40.5 -- and
        // three of the four screws ended up behind 14 mm of solid
        // arm, with no way in. Probed with a 3 mm pin down the bolt
        // circle: 0.204 cm3 of plastic in their way.
        //
        // Through-drilled rather than counterbored. A counterbore big
        // enough for an M3 head leaves 0.4 mm of wall where the bolt
        // circle meets the tongue's edge at z = +/-11; a 3.4 mm hole
        // leaves 1.8. It costs 7% of the tongue's section and asks for
        // a 45 mm screw instead of a 20, and that screw then runs as a
        // steel dowel through the arm's root, which is no bad thing.
        rotate([0,90,0])
            ds3225_horn_cuts(CLEAR, ARM_X + ARM_W/2 + 2);
        // AND AN ACCESS BORE DOWN THE AXIS. The horn is held to the
        // servo's spline by its own central screw, and the tongue now
        // covers the axis -- so without this there is no way to reach
        // it once the shoulder is on. Carried right through to the
        // outer face, so a long hex key goes straight down it.
        //
        // It costs 62 mm2 of a 748 mm2 section. The spline relief in
        // ds3225_horn_cuts is this diameter already; this is the same
        // hole, not stopped at 7 mm deep.
        rotate([0,90,0])
            cylinder(d = SERVO_SPLINE_D + 3, h = ARM_X + ARM_W/2 + 4);
        // NO LIGHTENING HOLES IN THE ARM. There were two 11 mm hexes
        // through it here. The arm carries the entire leg -- pitch
        // servo, knee servo, thigh, shank, and whatever the foot is
        // standing on -- cantilevered off the roll collar, and it is
        // the single load path between the two. Perforating that to
        // save a couple of grams is the wrong trade. The hexes belong
        // in the chassis plate, where they are, and not here.
        //
        // (It was a 9 mm rail when that was written. It is 15 x 22
        //  now, and the tongue 22 x 22, which is the whole reason the
        //  horn screws above had to be re-drilled through it.)
    }
}

PART = "shoulder"; ARM = 1;
if (PART == "shoulder") shoulder(ARM);
