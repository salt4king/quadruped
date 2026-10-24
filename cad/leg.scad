include <params.scad>
use <servo.scad>

// ===================================================================
// LEG LINKS -- the knee runs through a parallelogram linkage.
//
// Every Y position comes from the HIP STACK in params.scad. Nothing here
// decides where it sits.
//
//   thigh   plate THIGH_IN..THIGH_OUT, driven at its outboard face by the
//           pitch servo. Carries the knee servo on a bracket at CRANK_C
//           down its length, and the knee bearing at its far end.
//   crank   LINK_A long, on the knee servo at C
//   rod     LINK_D long, crank tip to rocker tip, in its own plane
//   shank   rocker lug, collar in the thigh's bore, beam, foot
//
// THE CRANK AND THE ROCKER SHARE THE SHANK'S PLANE, which costs no width:
// the rocker is just a backward lug on the shank, and the crank sits at C,
// 64 mm up the thigh where the shank never reaches. Only the rod needs a
// plane of its own.
//
// Both arms sweep BEHIND the thigh -- the shank only ever folds forward,
// so the back is empty. That is also what leaves the thigh's front edge
// free to carry the servo bracket out past them.
//
// Each joint: a collar carries the bending; a horn or bolt carries only
// torque.
// ===================================================================

// Outline and joint sizes come from params.scad -- see LEG PLAN there.
BORE_D  = JOINT_BORE_D;
COLL_D  = BORE_D - 2*CLEAR;
COLL_ID = SERVO_HORN_D + 2;
BOSS_D  = BORE_D + 2*WALL + 4;
RAIL    = TRUSS_RAIL;
TRI_R   = TRUSS_FILLET;

module tri(a,b,c) { offset(r=TRI_R) offset(delta=-TRI_R) polygon([a,b,c]); }
function half_w(y) = HEAD_R + (KNEE_R - HEAD_R) * y / THIGH;
module truss(y0, y1, n) {
    seg = (y1 - y0) / n;
    for (i = [0 : n-1]) {
        ya = y0+i*seg+RAIL/2; yb = y0+(i+1)*seg-RAIL/2; ym = (ya+yb)/2;
        s = (i % 2 == 0) ? 1 : -1;
        tri([s*(half_w(ya)-RAIL),ya],[s*(half_w(yb)-RAIL),yb],
            [-s*(half_w(ym)-RAIL),ym]);
    }
}
// A journal that runs in a bore. Hollow at the knee, where the M4 pin
// passes through it -- SOLID at the hip, where a servo horn has to bolt
// to its face.
//
// It was hollow at both. COLL_ID is the horn's diameter plus 2, so the
// hip collar was a ring with the horn sitting in the hole: slice the
// thigh and there is 0.000 cm3 of material inside the screw circle
// anywhere between y = 35 and the servo's face at 39. The pitch horn
// was held 4 mm off the nearest plastic by four M3 screws spanning open
// air, taking the whole leg's weight in bending on the screws alone.
// Solid gives the horn a face to pull against, and ds3225_horn_cuts
// still sinks its pocket into it.
module collar(y0, y1, solid = false) {
    difference() {
        translate([0,y0,0]) rotate([-90,0,0]) cylinder(d = COLL_D, h = y1-y0);
        if (!solid)
            translate([0,y0-1,0]) rotate([-90,0,0])
                cylinder(d = COLL_ID, h = y1-y0+2);
    }
}
module bore(y0, y1) {
    translate([0,y0-0.01,0]) rotate([-90,0,0]) cylinder(d = BORE_D, h = y1-y0+0.02);
}

// Where the servo mount crosses the shank's plane. Squeezed from both
// sides: nearer the hip and the pitch servo's body catches it when the
// thigh swings back; further down and the folded shank reaches it.
//
// The window was re-measured when the crossing was made taller, because
// the pitch housing had just moved 6 mm outboard with the longer hip
// collar. Sweeping CROSS_H against CROSS_Z at hip -88 and at knee 120
// to 150:
//
//        CROSS_Z    48      50      52      54
//   H=14           ok      ok      ok      ok
//   H=18        0.219      ok      ok      ok
//   H=20        0.387   0.052      ok      ok
//
// So the limit is a diagonal, not a fixed depth: every mm taller costs
// about a mm of depth. 20 at 54 is clear with room on both sides, and
// it is 2x the section modulus of the 14 it started at.
//
// And it crosses on the FORWARD side. Both arms sweep behind the thigh,
// so forward of the thigh line the crank never reaches; a folded shank
// passes through here too, but 38 mm out at the worst knee angle.
CROSS_Z  = 54.0;   // down the thigh
CROSS_X0 = 5.0;    // forward of the thigh line, near edge
// 20, and it does NOT get to be wider. Forward is the safe side for
// the CRANK, which never reaches it, but not for the shank: folded
// right up at knee 150 with the hip back and the leg rolled over, the
// shank comes to this edge. Measured at that pose:
//
//   CROSS_X1   20     22     23     24
//   shank      ok   0.031  0.112  0.207 cm3
//
// The comment that used to sit here said a folded shank cleared by
// 38 mm. It does not; that was the clearance at some other knee angle.
CROSS_X1 = 20.0;   //                            far edge
CROSS_H  = 20.0;   // its height along the thigh
// The crossing and the rail are the whole knee-servo bracket: a
// cantilever hanging off the thigh plate, and the only thing holding
// the servo that drives the knee. 14 mm and 6 was thin for that. 18 and
// 9 is about double the section modulus in each, and the sweep says
// neither reaches anything.
// 13, not 9. This rail is the only thing between the thigh plate and
// the knee servo, and printed with the plate flat on the bed it stands
// up as a column -- so its bending runs across the layer lines, which
// is the case PETG is worst at and the one check_walls.py derates 40%
// for. It grows OUTBOARD into the band the servo body already
// occupies, so it costs no width anywhere.
//
// The thickness is the free direction and the one that was thin:
// 15 x 9 = 135 mm2 becomes 15 x 13 = 195, a 44% bigger section without
// going anywhere near the shank.
RAIL_T  = 13.0;    // outboard rail carrying the servo down to C

module thigh() {
    difference() {
        union() {
            translate([0, THIGH_IN, 0]) rotate([-90,0,0])
                linear_extrude(THIGH_OUT - THIGH_IN)
                    difference() {
                        hull() { circle(r=HEAD_R); translate([0,THIGH]) circle(r=KNEE_R); }
                        truss(HEAD_R+1, THIGH-KNEE_R-1, 6);
                    }
            collar(HIP_COLLAR, PITCH_FACE, true);
            // NO PRINTED CRANK STOPS, deliberately. A parallelogram has
            // a crossed twin on the far side of a dead point, and the
            // obvious guard is a post the crank runs into -- but the
            // crank's plane is the shank's plane, and the shank folds
            // through it, so there is nowhere to put one. What keeps the
            // linkage on its branch instead is that the crank cannot
            // reach a dead point: its travel stops 22.5 degrees short at
            // both ends, held there by the servo's own range and by the
            // IK's joint limits. Assemble it on the right branch and it
            // has no way to leave under power. A knock hard enough to
            // backdrive a servo past its travel is the case this does
            // not cover.
            //
            // KNEE SERVO MOUNT. Getting from the thigh plate out to the
            // servo means crossing the shank's plane, and there is only
            // one place on the whole thigh where that is safe.
            //
            // Not at C, on the front: the shank folds forward through
            // there (1.4 cm3 of shared material at knee 150).
            // Not at C, behind: the crank and the rocker sweep there.
            //
            // So cross up by the HIP, where a folded shank is already
            // 40 mm forward of the thigh line, then run outboard of
            // everything down to the servo. Measured, not guessed.
            translate([0, 0, -CROSS_Z])
                translate([(CROSS_X0 + CROSS_X1)/2, (THIGH_IN + KNEE_FACE)/2, 0])
                    cube([CROSS_X1 - CROSS_X0, KNEE_FACE - THIGH_IN, CROSS_H],
                         center = true);
            // Rail down the outboard face. Outboard of the rod's plane,
            // so nothing that moves can reach it.
            hull() {
                translate([(CROSS_X0 + CROSS_X1)/2, KNEE_FACE + RAIL_T/2, -CROSS_Z])
                    cube([CROSS_X1 - CROSS_X0, RAIL_T, CROSS_H], center = true);
                translate([(CROSS_X0 + CROSS_X1)/2, KNEE_FACE + RAIL_T/2, -CRANK_C])
                    cube([CROSS_X1 - CROSS_X0, RAIL_T, SERVO_W], center = true);
            }
            translate([0, 0, -CRANK_C]) translate([0, KNEE_FACE, 0])
                rotate([90,0,0]) ds3225_mount_solid(WALL);
            // Knee bearing: a boss the shank's collar runs in.
            translate([0,0,-THIGH]) translate([0, KNEE_COLLAR, 0]) rotate([-90,0,0])
                cylinder(d = BOSS_D, h = SHANK_IN - KNEE_COLLAR);
        }
        // Pitch horn: pocket, stepped bore and four screws, all from
        // one module so they cannot disagree. The 16 mm bore that used
        // to be here swallowed the screw circle whole.
        translate([0, PITCH_FACE, 0]) rotate([90,0,0])
            ds3225_horn_cuts(CLEAR, PITCH_FACE - THIGH_IN + 2);
        // Counterbored 1.5 mm at the inboard face. Without it the screw
        // window here is 16.0 < L <= 18.0 and NO stock length is in it:
        // an M3x16 touches the horn with nothing to bite and an M3x20
        // runs 2 mm past it into the servo. Sinking the head 1.5 moves
        // the window to 14.5 < L <= 16.5, so a plain M3x16 lands 1.5 mm
        // into the horn's 2.
        translate([0, THIGH_IN - 0.01, 0]) rotate([-90,0,0])
            ds3225_horn_access(1.5);
        // Knee servo pocket, its bolt pattern, the channel it slides in
        // through and the slot its cable leaves by -- all in the servo's
        // own frame so none of them can drift. Body runs OUTBOARD;
        // inboard is the chassis.
        translate([0, 0, -CRANK_C]) translate([0, KNEE_FACE, 0]) rotate([90,0,0]) {
            ds3225_mount_cuts(SERVO_FIT);
        }
        // Room for the whole crank -- arm, hub and the servo's gearbox
        // boss. From LINK_IN, not LINK_OUT: the arm's root circle lives
        // in the band below the hub and the servo bracket crosses right
        // through it.
        translate([0, 0, -CRANK_C]) translate([0, LINK_IN, 0]) rotate([-90,0,0])
            // LINK_TIP_R + 1 because that is the crank's actual tip
            // radius. Sizing this to LINK_TIP_R made the relief 0.7 mm
            // SMALLER than the thing it is meant to clear.
            cylinder(d = 2*(LINK_A + LINK_TIP_R + 1 + CLEAR),
                     h = KNEE_FACE - LINK_IN + 0.01);
        // THE KNEE PIN. The collar in the bore locates the shank
        // radially; nothing was stopping it walking out sideways. An M4
        // shoulder bolt comes in from inboard through this clearance
        // hole and threads into the shank -- its plain shank spans the
        // thigh so the joint still turns, exactly like the linkage
        // pivots.
        translate([0,0,-THIGH]) translate([0, THIGH_IN - 2, 0]) rotate([-90,0,0])
            cylinder(d = KNEE_CLEAR_D, h = KNEE_COLLAR - THIGH_IN + 4);
        translate([0,0,-THIGH]) bore(KNEE_COLLAR - CLEAR, SHANK_IN + 1);
    }
}

// -------------------------------------------------------------------
// THE LINKAGE. Two flat parts, both printed on their face so the layer
// lines run across the load rather than along the split.
// -------------------------------------------------------------------
// Drawn pointing -Z, the same direction the thigh runs, so every angle in
// the assembly is measured the same way: rotate([0,-a,0]) swings a part to
// `a` degrees from the thigh line.
// `hole1` is the pivot at the far end and it differs by part: the rod
// gets clearance so it can turn, the crank and rocker get an insert for
// the shoulder bolt to thread into. The root hole is left to the caller
// -- it is a spline bore on the crank and a knee pin on the rocker.
module arm(len, thick, r0, r1, hole1 = 0) {
    difference() {
        linear_extrude(thick)
            hull() { circle(r = r0); translate([len, 0]) circle(r = r1); }
        if (hole1 > 0)
            translate([len, 0, -1]) cylinder(d = hole1, h = thick + 2);
    }
}

// The rod's swept volume, expressed in the CRANK's own frame.
//
// The rod runs parallel to the thigh and pivots on the crank's tip, so
// relative to the crank it swings through the knee's whole range -- and
// at full extension its edge passes within 0.3 mm of the crank's own
// axis. No hub diameter clears that; a 16 mm one was touching. What
// saves it is that the rod always passes on the SAME SIDE, so the hub
// can be a D instead of a circle. Cutting the sweep itself rather than
// guessing a flat keeps the two in step if either changes.
module rod_sweep(c = CLEAR + 0.2) {
    steps = 24;
    lo = CRANK_SWEEP[0]; hi = CRANK_SWEEP[1];
    translate([0, 0, -LINK_A])
        for (i = [0 : steps]) rotate([0, lo + (hi - lo)*i/steps, 0])
            translate([0, ROD_IN - c, 0]) rotate([-90,0,0]) rotate([0,0,90])
                arm(LINK_D, ROD_T + 2*c, LINK_TIP_R + c, LINK_TIP_R + c);
}

module crank() {
    difference() {
        union() {
            translate([0, LINK_IN, 0]) rotate([-90,0,0]) rotate([0,0,90])
                arm(LINK_A, LINK_T, 13, LINK_TIP_R + 1, M3_INSERT);
            // Hub reaching outboard past the rod's plane to the horn.
            // SLIM where it crosses that plane: the rod's end swings to
            // within 9 mm of the shaft, so a 20 mm hub all the way out
            // rubs it. Full diameter only outboard of the rod.
            translate([0, LINK_OUT, 0]) rotate([-90,0,0])
                cylinder(d = 16, h = ROD_OUT + 0.8 - LINK_OUT);
            // Full diameter outboard of the rod: it has to hold the
            // horn pocket with a printable wall around it, which the
            // 24 mm I first used did not (0.7 mm).
            translate([0, ROD_OUT + 0.8, 0]) rotate([-90,0,0])
                cylinder(d = SERVO_HORN_D + 2*CLEAR + 2*WALL,
                         h = KNEE_FACE - ROD_OUT - 0.8);
        }
        // Horn, stepped bore and screws, same module as the thigh.
        // rotate([90,0,0]), matching how the servo itself is placed:
        // its body runs outboard, so its +Z -- the side the crank is on
        // -- runs inboard. Using -90 pointed every cut into thin air
        // beside the servo and left the crank with no screw holes.
        translate([0, KNEE_FACE, 0]) rotate([90,0,0])
            ds3225_horn_cuts(CLEAR, KNEE_FACE - LINK_IN + 2);
        // Same treatment, 1.1 mm. An M3x20 was landing 0.4 mm into the
        // horn -- inside the window, but barely a thread.
        translate([0, LINK_IN - 0.01, 0]) rotate([-90,0,0])
            ds3225_horn_access(1.1);
        rod_sweep();
    }
}

// Both ends clearance: the rod is the part that turns, at both pivots.
module pushrod() {
    difference() {
    translate([0, ROD_IN, 0]) rotate([-90,0,0]) rotate([0,0,90])
        arm(LINK_D, ROD_T, LINK_TIP_R, LINK_TIP_R, PIVOT_ROD_D);
    translate([0, ROD_IN - 1, 0]) rotate([-90,0,0])
        cylinder(d = PIVOT_ROD_D, h = ROD_T + 2);
    }
}

module shank() {
    difference() {
        union() {
            // Rocker: a backward lug in the shank's OWN plane, which is
            // what makes the linkage cost no extra width. LINK_PHI is the
            // fixed angle between it and the shank, and the parallelogram
            // turns that into servo = knee + LINK_PHI.
            rotate([0, -LINK_PHI, 0])
                translate([0, LINK_IN, 0]) rotate([-90,0,0]) rotate([0,0,90])
                    arm(LINK_A, LINK_T, KNEE_R, LINK_TIP_R + 1);
            collar(KNEE_COLLAR + CLEAR, SHANK_IN);
            // Tapered beam, not the hairline rod the sketch showed. The
            // sketch's shank leaned on a lever; this knee is direct
            // drive, so the shank carries its own bending. Widths come
            // from params.scad and are stress-checked by check_walls.py.
            // (They used to come from ROD_W, which was deleted with the
            //  four-bar linkage -- OpenSCAD silently read it as undef and
            //  printed the shank as a one-pixel thread.)
            translate([0, SHANK_IN, 0]) rotate([-90,0,0])
                linear_extrude(SHANK_T) {
                    hull() { circle(r = KNEE_R);
                             translate([0,24]) circle(d = SHANK_W_KNEE); }
                    hull() { translate([0,24]) circle(d = SHANK_W_KNEE);
                             translate([0,SHANK-FOOT_R]) circle(d = SHANK_W_FOOT); }
                }
            translate([0, SHANK_IN + SHANK_T/2, -SHANK]) sphere(r = FOOT_R);
        }
        // THE ROD'S PIVOT, cut here and not inside arm(). The beam's
        // knee end is a disc of radius KNEE_R = 15.5 in the SAME plane
        // as the rocker, and the pivot sits at LINK_A = 15.0 from the
        // axis -- so a hole cut in the rocker first is filled straight
        // back in by the beam, and 15.5 - 15.0 = 0.5 mm of it survives.
        // It printed as a crescent with nothing to screw into.
        rotate([0, -LINK_PHI, 0])
            translate([0, LINK_IN, 0]) rotate([-90,0,0]) rotate([0,0,90])
                translate([LINK_A, 0, -1])
                    cylinder(d = M3_INSERT, h = LINK_T + 2);
        // The other half of the knee pin: an M4 insert the shoulder
        // bolt threads into. The shank is the part it is fixed to; the
        // thigh is the part it turns in.
        translate([0, KNEE_COLLAR + CLEAR, 0]) rotate([-90,0,0])
            cylinder(d = M4_INSERT, h = 10);
        translate([0, SHANK_IN + SHANK_T/2, -SHANK - FOOT_R + 0.5])
            cylinder(d = 12, h = 6, center = true);
    }
}

// LEFT AND RIGHT. The assembly mirrors each leg in Y for the right
// hand side, so a part is only one print for both sides if it has a
// mirror plane of its own -- then its mirror image is just the same
// solid turned round. Measured by intersecting each part with its own
// reflection:
//
//   chassis, roof, pushrod, filler   symmetric in Y  -> one print
//   shoulder                         symmetric in Z  -> one print,
//                                    installed turned over
//   thigh, shank, crank              no mirror plane -> two prints
//
// The shank comes out 99.0% self-overlapping and the crank 95.2%,
// which is the trap: they LOOK symmetric. The shank's rocker lug sits
// at LINK_PHI off the beam and the crank's hub is a D-flat cut by the
// rod's sweep, and neither survives a reflection. Print 2 + 2.
PART = "thigh"; FILLER_SIDE = 1; HAND = 1;
mirror([0, HAND > 0 ? 0 : 1, 0]) {
    if      (PART == "thigh")   thigh();
    else if (PART == "filler")  ds3225_filler(WALL, FILLER_SIDE);
    else if (PART == "shank")   shank();
    else if (PART == "crank")   crank();
    else if (PART == "pushrod") pushrod();
}
