include <params.scad>
use <servo.scad>
use <hip.scad>

// ===================================================================
// CHASSIS -- H / hourglass plan.
//
// Two wide end bars carrying the four hip servos, joined by a narrow
// spine. Material sits where the load is (the corners, where the legs
// hang) and is absent down the middle, where a plate would only be
// carrying itself. The waist is sized to the battery, which is the one
// item with no flexibility about where it goes.
//
// The hip servo mounts are part of this plate, not a separate bracket.
// That removes a part, a bolted joint, and the stack-up error that joint
// would add between the chassis and the roll axis.
//
// WHAT GOES ON IT
//   battery  2S LiPo, on the spine, strapped through the slots
//   boards   microcontroller and servo driver, on the pads either side
//   servos   4 x hip roll, one per corner, shafts on the roll axes
//
// The other 8 servos ride on the legs: pitch and knee live in the
// shoulder, which is the part that rotates on the roll servo's output.
//
// LIGHTENING: hexagons. They tile with no wasted web between them, and
// every remaining member meets its neighbours at 120 degrees, so load
// splits three ways instead of concentrating. They also print without
// support at any orientation -- no part of a hex outline is more than
// 30 degrees off vertical.
// ===================================================================

CHASSIS_T  = 8.0;   // plate skin. NOT params.scad's PLATE_T,
                    // which is the thigh's 8 mm -- this file used
                    // to shadow that name and silently win.
RAIL_H   = 10.0;
RAIL_W   = 4.0;
// No floor under the roll pockets any more, and none needed. It existed
// because the servo stood on end with its bottom 4 mm sunk into the
// plate, so the slide channel and the driver scallops broke out through
// the underside and you could see the filler from below. The servo lies
// flat now (hip.scad), 26 mm tall, sitting ON the 8 mm skin -- so the
// pocket never reaches the underside and there is nothing to cover.
// End bar, along the body. Stays at 74: widening it to 94 to reach the
// rear servo mounts put chassis into the THIGH's sweep. Each mount gets
// a pad the shape of its own footprint instead -- see roll_pad().
BAR_X    = 74;
// End bar, across. It stops SHORT of the roll axes, not past them: the
// shoulder's arm sweeps from ARM_Y0 outboard of each axis, so a bar that
// runs out to 74 (19 mm past the hip) is sitting in that sweep. It was
// being cut away there and leaving the outer rail as four loose strips.
BAR_Y    = 128;    // hips at +/-55, so the bar ends 9 mm outboard of them
SPINE_X  = 156;    // spine length, overlapping both bars
SPINE_Y  = 72;     // waist, set by the battery
BATT_X   = 140;    // 2S 5000 mAh LiPo
BATT_Y   = 46;
HEX_A    = 11.0;
HEX_GAP  = 5.0;
PAD      = 12.0;   // board mounting grid
COLLAR_D = 36.0;   // must match shoulder.scad
// Depth of the recess the SHOULDER's roll collar sits in. Named for
// this file only: params.scad used to emit a COLLAR_H of its own for
// the leg journals, and this line silently shadowed it for the whole
// file -- the same trap PLATE_T sprang. The leg's two are now
// HIP_COLLAR_H and KNEE_COLLAR_H, and nothing is called COLLAR_H twice.
RCOLLAR_RECESS_H = 10.0;  // matches shoulder.scad's RCOLL_H
ROLL_BOSS_H = 10.0;

// ---------------------------------------------------------------
// CABLE CLIPS. Twelve servo leads have to reach the driver, and loose
// wire in a walking machine eventually finds a linkage.
//
// A C-section the lead presses into: 5.2 mm bore, which takes a
// standard 3-core servo lead with its connector left on, a mouth
// narrower than the bore so it stays put, and 1.6 mm of wall -- four
// perimeters, no infill, nothing to warp.
//
// They sit along the WAIST, on the outside. The waist is the one part
// of the outline that is straight, nothing sweeps near it, and it is
// the shortest run from any leg to the boards on the spine.
// ---------------------------------------------------------------
CLIP_R     = 2.6;
CLIP_WALL  = 1.6;
CLIP_H     = 6.0;
CLIP_MOUTH = 3.6;        // < 2*CLIP_R, so the lead snaps in and stays
CLIP_Z     = CHASSIS_T + 3;
CLIP_X     = [-46, -23, 0, 23, 46];

// ---------------------------------------------------------------
// ROOF. A lid over the battery and the boards, on four snap tabs so
// it comes off without tools.
//
// It has to stand off, not lie flat: a 2S 5000 mAh pack is 25 mm tall
// and sits on the plate, so the inside has to clear that and the
// boards on their standoffs. The skin is 2 mm with the same hexagons
// as the plate -- it carries nothing but itself, so there is no reason
// for it to be solid.
// ---------------------------------------------------------------
ROOF_T      = 2.0;
ROOF_CLEAR  = 27.0;                       // headroom above the plate
ROOF_UNDER  = CHASSIS_T + ROOF_CLEAR;     // underside of the skin
RAIL_TOP    = CHASSIS_T + RAIL_H;         // the skirt lands here
// The lid CLIPS OVER THE RIM rather than reaching down inside it.
// Inside, the tab would have to live in the waist rail's own 4 mm of
// material and still have something to hook on; outside, the rim is
// the thing it hooks on, the finger prints as a plain vertical wall,
// and you can see and reach the catch to release it.
FING_W      = 12.0;
FING_T      = 2.0;
FING_BOT    = RAIL_TOP - 9.0;      // how far down the outside it reaches
BARB        = 1.4;
CATCH_H     = 4.0;
CATCH_SLOP  = 0.6;
// Between the cable clips, not on top of them.
FING_X      = [-34.5, 34.5];

module hex(a) { rotate([0, 0, 30]) circle(d = a / cos(30), $fn = 6); }

// Opens in +y, drawn flat and extruded, so it prints as a vertical C
// with no overhang anywhere on it.
module wire_clip() {
    linear_extrude(CLIP_H) difference() {
        circle(r = CLIP_R + CLIP_WALL, $fn = 28);
        circle(r = CLIP_R, $fn = 28);
        translate([-CLIP_MOUTH/2, 0])
            square([CLIP_MOUTH, CLIP_R + CLIP_WALL + 1]);
    }
}

// Where each clip sits: back buried in the waist rail, mouth outward.
// Buried 3 mm and not 1. At 1 the contact was a 5 mm lens and six of
// the ten printed as loose pieces -- a C touching a wall on a sliver
// is not joined to it. The rail is 4 mm, so 3 still leaves wall behind.
module at_clips() {
    for (sy = [-1, 1], x = CLIP_X)
        translate([x, sy*(SPINE_Y/2 + CLIP_R + CLIP_WALL - 3), CLIP_Z])
            rotate([0, 0, sy > 0 ? 0 : 180]) children();
}

// The groove each finger's barb hooks into: a shallow pocket in the
// OUTSIDE of the waist rail. 1.6 mm deep, so it leaves most of the
// 4 mm rail behind it.
CATCH_DEPTH = 1.6;
module roof_catches() {
    for (sy = [-1, 1], x = FING_X)
        translate([x, sy*(SPINE_Y/2 - CATCH_DEPTH/2 + 0.5),
                   FING_BOT + (CATCH_H + CATCH_SLOP)/2])
            cube([FING_W + 0.8, CATCH_DEPTH + 1, CATCH_H + CATCH_SLOP],
                 center = true);
}

module outline() {
    offset(r = 13) offset(delta = -13) union() {
        for (sx = [-1, 1])
            translate([sx*BODY_LEN/2, 0]) square([BAR_X, BAR_Y], center = true);
        // ...plus a pad under each roll servo mount, which hangs off its
        // bar rather than sitting inside it. Taken as the projection of
        // the mount itself so it cannot be the wrong size, which is the
        // mistake that left the rear mounts on 94 mm2 of plate.
        for (sx = [-1, 1], sy = [-1, 1]) roll_pad(sx, sy);
        square([SPINE_X, SPINE_Y], center = true);
    }
}

// Roll servo: shaft on the roll axis pointing outboard along X, body
// lying flat so it adds 20 mm of height rather than 40, and raised to sit
// ON the plate rather than half-buried through it.
//
// HIP_Z is where this puts the roll axis, measured from the underside of
// the plate. The kinematic model's hip origins live in that plane -- the
// model never cared about the plate's own position, only that all four
// hips share one.
// HIP_Z is emitted by sync_params.py from this same expression, so
// robot.scad and the interference checker read the one value instead of
// each re-deriving it. They did not agree: the assembly had it at 26 mm
// and the checker at 16, so every render put the legs 10 mm clear of the
// chassis they are bolted to.

// NOT mirrored fore/aft any more. It used to be, to keep all four servo
// housings tucked inboard -- and that is exactly what broke the rear
// legs: the servo body and the shoulder's arm both ended up pointing at
// the body centre, so rolling a rear leg inboard drove the arm into its
// own servo. Measured, it touched at 15 degrees of roll and shared
// 0.56 cm3 at 35. The front pair were clean because their arm and their
// servo point opposite ways.
//
// So every leg is now the same assembly, and the rear servos hang off
// the back of the bar instead of reaching under it. That is what the
// plate's extra length at the rear is for.
//
// The transform itself lives in hip.scad, in two versions: roll_frame()
// for anything servo-shaped, hip_at() for the arm sweeps, which are
// quoted in world Y and want Z still pointing up. Mixing those two up
// transposes a cut, and the pair of them used to be one frame with
// forty lines of comment explaining which way round each cut had to go.

// WHERE THE HOLES GO, AND WHERE THEY DO NOT.
//
// One rule: lighten what is doing nothing, leave what is carrying a leg.
//
// The old field had it backwards -- it skipped the spine and perforated
// both END BARS, so all four hip loads were entering a honeycomb. It
// also used a 15 mm hex on a 3 mm skin with a 3.4 mm web, which is most
// of the way to a lattice. Now the holes keep 34 mm clear of every hip
// and 20 mm clear of the battery straps, and they are smaller with more
// web between them.
module hex_field() {
    pitch = HEX_A + HEX_GAP;
    for (r = [-8 : 8], c = [-11 : 11]) {
        x = c*pitch + (r % 2 == 0 ? 0 : pitch/2);
        y = r*pitch*0.866;
        // Keep well away from the hips -- that is where a leg's whole
        // load enters the plate, and it is where the holes used to be.
        near_hip = min([for (sy = [-1,1], sx = [-1,1])
                        norm([x - sx*BODY_LEN/2, y - sy*BODY_WID/2])]) < 34;
        // The spine gets none at all. It is 72 mm wide with the battery
        // straps down it, so a hex row there merges into the slots and
        // leaves ragged webs rather than a honeycomb -- and the spine is
        // what carries bending between the two bars.
        in_bar = abs(x) > SPINE_X/2 - 4;
        if (!near_hip && in_bar) translate([x, y]) hex(HEX_A);
    }
}

// The room the shoulder's arm needs. A module, because the floor
// added at the end of chassis_plate() has to respect it too --
// it is union'd after the cuts, so it would otherwise sit in the
// arm's way (0.019 cm3, on the rear legs).
// The room the shoulder's arm needs.
module shoulder_clearance(sx, sy) {

    // THE SAME CUT AT ALL FOUR CORNERS, because the geometry is now the
    // same at all four corners. Measured from the axis, at both ends:
    // the shoulder reaches -40.0 to +33.5 and the roll servo -40.4 to
    // +7.0. So the arm is always on the +x side of its own axis and the
    // servo is always on the -x side, front and rear alike.
    //
    // This used to carry a scale([1, 1, sx]) that flipped the cut
    // fore/aft, from back when the shoulder mirrored and the arm really
    // did change sides. Nothing mirrors fore/aft any more, so all that
    // flip did was aim the rear cut at the one thing on that side: the
    // servo's own casing. It ate 6.8 cm3 of it per rear corner -- 12.0
    // left against the front's 18.8 -- while leaving the arm's real side
    // uncut. It got away with that only because the swept cylinder is a
    // conservative envelope and the arm, a rail out at ARM_Y0..ARM_Y1,
    // never actually reached the bar it was failing to cut.
    hip_at(sx, sy) {

        // Past the boss, the arm and everything it carries turns in
        // here, so nothing of the chassis may be inside the swept
        // radius. The servo lives at -40.4..+7, so a full cylinder
        // starting at +10 cannot touch it.
        translate([0, 0, ROLL_BOSS_H])
            cylinder(r = SHOULDER_SWEEP_R, h = 60);

        // Across the boss's own depth, the outboard half only. The arm
        // lives outboard of the roll axis -- ARM_Y0 to ARM_Y1 -- and
        // never swings inboard of it even rolled right over, and a full
        // cylinder here took the plate out on both sides and left a
        // servo mount hanging on 94 mm2 against the front's 1059.
        //
        // Not the collar boss, either way round. That boss is the whole
        // reason the leg's bending stays off the servo's output shaft,
        // and a plain cylinder deleted it outright. It used to be
        // spared only at the front, on the belief that the rear collar
        // faced backward; it does not, so it is spared at both.
        difference() {
            intersection() {
                cylinder(r = SHOULDER_SWEEP_R, h = ROLL_BOSS_H + 0.01);
                translate([-SHOULDER_SWEEP_R, 0, -1])
                    cube([2*SHOULDER_SWEEP_R, SHOULDER_SWEEP_R + 2,
                          ROLL_BOSS_H + 2]);
            }
            cylinder(d = COLLAR_D + 2*WALL, h = ROLL_BOSS_H);
        }
    }
}

module chassis_plate() {
    difference() {
        union() {
            linear_extrude(CHASSIS_T) outline();
            // Perimeter rail: a 3 mm plate is a spring in bending, and
            // bending stiffness goes with the CUBE of depth, so 10 mm of
            // rail buys far more than 10 mm of extra plate ever could.
            linear_extrude(CHASSIS_T + RAIL_H)
                difference() { outline(); offset(delta = -RAIL_W) outline(); }
            // Spine rails, carrying load corner to corner.
            for (sy = [-1, 1])
                translate([0, sy*(SPINE_Y/2 - RAIL_W/2)])
                    linear_extrude(CHASSIS_T + RAIL_H)
                        square([SPINE_X - 20, RAIL_W], center = true);
            // Servo housings at the four corners, each with a raised boss
            // the shoulder's collar runs in.
            //
            // THIS BOSS WAS MISSING. Every other joint on the robot has a
            // collar so the servo's output bearing only ever sees torque,
            // never bending -- and the roll joint, which carries the whole
            // leg, had none. The entire weight of the limb was going
            // through one servo's output shaft in the one direction it is
            // not built for.
            for (sx = [-1, 1], sy = [-1, 1]) {
                // Dims follow the servo in its OWN frame: length, WIDTH,
                // height. Using SERVO_H for the middle term described a
                // servo lying the other way and made the housing 46.4 mm
                // wide in Y instead of 26 -- which is what the shoulder
                // and the pitch servo were both running into.
                // Centred on the servo BODY, which sits at local
                // z = -SERVO_H..0 because the origin is its shaft face.
                // Centring on the origin instead put the housing 20 mm
                // out of place -- after the rotation that threw it 23 mm
                // outboard along X, into the shoulder's boss, while
                // leaving the servo itself half uncovered.
                roll_frame(sx, sy) ds3225_mount_solid(WALL, 1);
                // Short of ROLL_BOSS_H by a clearance. The shoulder's
                // own boss starts exactly where its collar ends, so it
                // cannot be moved out to make this gap -- do that and
                // the shoulder prints in two pieces, which it once did.
                // So the chassis gives way instead. Butted, the two
                // bosses press 333 mm2 of face together on a joint
                // that has to rotate, which is a friction brake with
                // the whole leg's weight behind it.
                hip_at(sx, sy)
                    cylinder(d = COLLAR_D + 2*WALL, h = ROLL_BOSS_H - CLEAR);
            }
            // Cable clips along the waist. They go in HERE, in the same
            // union as everything else, not beside the finished plate:
            // a solid placed next to a difference() result is a second
            // top-level object, and six of the ten printed as loose
            // pieces no matter how far into the rail they were buried.
            // Nothing cut below reaches them -- the hexes stop 7 mm
            // inside the outline and every other cut is inboard of the
            // waist -- so being inside the difference costs nothing.
            at_clips() wire_clip();
        }

        // Not under the roll pads: the pocket's floor IS the plate
        // there, and a hex in it opens the servo bay to the underside --
        // which is the thing the old floor skin was patching.
        translate([0, 0, -1]) linear_extrude(CHASSIS_T + 2)
            difference() {
                intersection() { hex_field();
                                 offset(delta = -(RAIL_W + 3)) outline(); }
                for (sx = [-1, 1], sy = [-1, 1])
                    offset(r = 3) roll_pad(sx, sy);
            }

        // Servo pockets and their mounting holes.
        for (sx = [-1, 1], sy = [-1, 1])
            roll_frame(sx, sy) {
                // Fitted from the BACK -- inboard, from under the end
                // bar -- so the front face is free to carry nothing but
                // the four pads and the shaft opening, which is all the
                // shoulder leaves room for. A filler goes in behind it,
                // same as the knee and pitch servos.
                ds3225_mount_cuts(SERVO_FIT, 1);
            }

        // Swept clearance for the shoulder. It turns about the roll axis
        // with structure out to SHOULDER_SWEEP_R, and the perimeter rail
        // stands 10 mm proud right where that sweep lands -- so the rail
        // is cut back within the swept cylinder, outboard of the collar
        // boss. The hip boss and servo housing carry the local stiffness
        // the rail would have.
        //
        // THE CUT FOLLOWS THE ARM, NOT THE SERVO. Both used to mirror
        // together, which was fine while the arm mirrored too. It does
        // not any more -- the arm is held forward of the thigh on all
        // four legs -- so on the rear pair the cut was being taken out
        // of the wrong side and the arm buried itself in the chassis
        // (2.9 cm3 a leg). scale() puts it back on the arm's side.
        for (sx = [-1, 1], sy = [-1, 1]) shoulder_clearance(sx, sy);

        // Collar recess in each boss, concentric with the roll axis.
        // Depth bounded to the boss. An earlier version ran 12 mm past it
        // and carved straight through the end bar behind.
        for (sx = [-1, 1], sy = [-1, 1])
            hip_at(sx, sy)
                translate([0, 0, -0.5])
                    cylinder(d = COLLAR_D + 2*CLEAR, h = RCOLLAR_RECESS_H + CLEAR + 0.5);

        // Battery straps through the spine.
        for (sx = [-1, 1])
            translate([sx*BATT_X/3, 0, -1]) linear_extrude(CHASSIS_T + 2)
                for (sy = [-1, 1])
                    translate([0, sy*(BATT_Y/2 + 5)])
                        offset(r = 2) square([14, 4], center = true);

        // Board mounting pads, both sides of the battery.
        for (sy = [-1, 1], gx = [-2 : 2], gy = [0 : 1])
            translate([gx*PAD, sy*(BATT_Y/2 + 7 + gy*PAD), -1])
                cylinder(d = M3_INSERT, h = CHASSIS_T + 6);

        // Cable routes, one per leg, through the bar inboard of each hip.
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx*(BODY_LEN/2 - 40), sy*(BODY_WID/2), -1])
                linear_extrude(CHASSIS_T + 2)
                    offset(r = 3) square([10, 16], center = true);

        // Slots for the roof's snap tabs.
        roof_catches();
    }

}

// ===================================================================
// THE ROOF. Printed separately, upside down -- every overhang on it is
// then either a wall or the barb's own ramp, so it needs no support.
// ===================================================================
module roof_plan() {
    union() {
        offset(r = 8) offset(delta = -8)
            square([SPINE_X - 4, SPINE_Y], center = true);
        // A pad per snap finger. The finger runs down the OUTSIDE of
        // the rim and has to stay clear of y = SPINE_Y/2 for its whole
        // length, because the rail above the catch groove is solid --
        // so it cannot reach back in to meet the skirt. The skirt is
        // brought out to meet IT instead, and the two then share real
        // material. Face contact is not a join: all four fingers came
        // off the plate as loose pieces.
        for (sy = [-1, 1], x = FING_X)
            translate([x, sy*(SPINE_Y/2 + FING_T/2)])
                square([FING_W, FING_T + 1], center = true);
    }
}

// The SKIRT's plan is the skin's, notched around the two front roll
// housings. They stand to z = 34 and reach 40 mm inboard along Y, so
// the skirt -- which runs from the rail at 18 up to the skin at 35 --
// goes straight through them; the skin itself starts at 35 and clears
// them, which is why only this one is notched. The skin spans the
// notches unsupported, which is 2 mm of PETG over 20 mm, carrying
// itself.
module roof_skirt_plan() {
    difference() {
        roof_plan();
        for (sx = [-1, 1], sy = [-1, 1])
            offset(r = 2)
                projection() roll_frame(sx, sy) ds3225_mount_solid(WALL, 1);
    }
}

module roof() {
    difference() {
        union() {
            // Skin.
            translate([0, 0, ROOF_UNDER]) linear_extrude(ROOF_T) roof_plan();
            // Skirt, down to the rail it stands on.
            translate([0, 0, RAIL_TOP])
                linear_extrude(ROOF_UNDER - RAIL_TOP)
                    difference() { roof_skirt_plan();
                                   offset(delta = -ROOF_T) roof_skirt_plan(); }
            // Snap fingers, continuing down the OUTSIDE of the rim.
            for (sy = [-1, 1], x = FING_X) {
                translate([x, sy*(SPINE_Y/2 + FING_T/2),
                           (FING_BOT + RAIL_TOP)/2])
                    cube([FING_W, FING_T, RAIL_TOP - FING_BOT],
                         center = true);
                // The barb. Narrow at the bottom and widening upward,
                // so pushing the lid down cams the finger outward and
                // it springs back when the barb's flat top clears the
                // groove's top edge -- which is then what holds it on.
                // Hulled from two thin slabs so the ramp is a printable
                // slope rather than a cliff.
                hull() {
                    translate([x, sy*(SPINE_Y/2 + FING_T/2), FING_BOT + 0.1])
                        cube([FING_W, FING_T, 0.2], center = true);
                    translate([x, sy*(SPINE_Y/2 + FING_T/2 - BARB/2),
                               FING_BOT + CATCH_H])
                        cube([FING_W, FING_T + BARB, 0.2], center = true);
                }
            }
        }
        // Same hexagons as the plate. The roof carries nothing but
        // itself, so there is no reason for it to be solid -- and it
        // lets the electronics breathe.
        translate([0, 0, ROOF_UNDER - 1]) linear_extrude(ROOF_T + 2)
            intersection() { hex_field();
                             offset(delta = -(ROOF_T + 4)) roof_plan(); }
    }
}

// ===================================================================
// SPLITTING IT, because 280 x 152 does not fit on a 235 mm bed.
//
// Rotating does not help: for a rectangle on a rectangular bed,
// axis-aligned is already the best fit -- turn a 280 x 152 plate and
// its bounding box only grows. So it has to come apart.
//
// A HALVED LAP, not a butt joint. The two halves overlap over
// LAP_LEN with the front taking the upper 4 mm and the rear the lower
// 4, and six M3 clamp them. Bending across the joint then goes through
// a 4 mm section rather than 8:
//
//   intact spine, worst case (all the weight forward of the rear
//   feet, about 2000 N.mm)   sigma 2.6 MPa, SF 11
//   across the lap           sigma 10.4 MPa, SF 2.9
//
// which still clears the SF 2 the rest of the robot is held to. The
// spine is hugely over-strength to begin with -- it is sized by the
// battery it has to straddle, not by load.
//
// The front half's rails run THROUGH the lap and the rear's stop at
// its edge, so the stiffest member is continuous across the joint on
// one side rather than broken on both.
// ===================================================================
SPLIT_X   = 0;
LAP_LEN   = 30;
LAP_Z     = CHASSIS_T/2;
// Clear of the battery straps at x +/-46.7 y +/-28, of the board pads
// at y +/-30, and of the hexes, which stay outside |x| = 74.
SPLIT_BOLT = [[-25, -14], [-25, 14], [-15, -14],
              [-15, 14], [-5, -14], [-5, 14]];

module split_bolts() {
    for (b = SPLIT_BOLT)
        translate([b[0], b[1], -1]) cylinder(d = M3_CLEAR, h = CHASSIS_T + 2);
}

// ---------------------------------------------------------------
// AND A PAIR OF LATCHES, so it clicks.
//
// They are for ASSEMBLY, not for strength. A halved lap wants both
// hands and six bolts at once otherwise; these hold the halves home
// and square while you get the bolts in, and tell you audibly that
// the lap is fully seated rather than sitting proud by a millimetre.
//
// They are NOT the joint. PETG creeps under a sustained load, and
// this one carries the spine's bending on every step -- a snap left
// to do that on its own relaxes and then lets go. The six M3 are the
// structure; if a latch ever breaks, nothing moves.
//
// Arrowhead type: two legs with a slit between them, so each bends
// inward as the barbs pass through the slot and springs back over the
// far face. 1.6 mm legs, which is four perimeters and no infill, and
// they only have to deflect 1.0 into a 1.8 mm slit.
//
// Placed at y = +/-28: outboard of the battery, which sits inside
// y = 23, and inboard of the waist rail at 32. The posts stand 16 mm
// up into the space under the roof, where nothing else goes.
// ---------------------------------------------------------------
LATCH_X     = -15;
LATCH_Y     = 28;
LATCH_W     = 12;                  // along the body
LATCH_T     = 5.0;                 // both legs plus the slit
LATCH_SLIT  = 1.8;
LATCH_TOP   = 16;
LATCH_BARB  = 1.0;
LATCH_CATCH = CHASSIS_T + 0.3;     // the barb's flat underside
LATCH_ROOT  = 2;                   // how far the post is buried in the lap

module at_latches() {
    for (sy = [-1, 1]) translate([LATCH_X, sy*LATCH_Y, 0]) children();
}

module latch_post() {
    lt = (LATCH_T - LATCH_SLIT)/2;
    z0 = LAP_Z - LATCH_ROOT;
    for (s = [-1, 1]) {
        translate([0, s*(LATCH_SLIT + lt)/2, (z0 + LATCH_TOP)/2])
            cube([LATCH_W, lt, LATCH_TOP - z0], center = true);
        // The barb. Flat underside at LATCH_CATCH -- that face is what
        // holds -- tapering back flush going UP, so the post cams its
        // own way through the slot as it rises.
        hull() {
            translate([0, s*((LATCH_SLIT + lt)/2 + LATCH_BARB/2),
                       LATCH_CATCH + 0.1])
                cube([LATCH_W, lt + LATCH_BARB, 0.2], center = true);
            translate([0, s*(LATCH_SLIT + lt)/2, LATCH_CATCH + 3])
                cube([LATCH_W, lt, 0.2], center = true);
        }
    }
}

module latch_slot() {
    translate([0, 0, (LAP_Z + CHASSIS_T)/2])
        cube([LATCH_W + 2*CLEAR, LATCH_T + 2*CLEAR,
              CHASSIS_T - LAP_Z + 2], center = true);
}

// Everything forward of the split, plus the UPPER half of the lap.
module chassis_front() {
    difference() {
        intersection() {
            chassis_plate();
            union() {
                translate([SPLIT_X + CLEAR/2, -200, -200])
                    cube([500, 400, 400]);
                translate([SPLIT_X - LAP_LEN + CLEAR/2, -200, LAP_Z + CLEAR/2])
                    cube([LAP_LEN, 400, 400]);
            }
        }
        split_bolts();
        at_latches() latch_slot();
    }
}

// Everything behind the lap, plus the LOWER half of it.
module chassis_rear() {
    difference() {
        // The posts go in the SAME union as the plate, not beside the
        // finished solid -- that is how six of the ten cable clips
        // came off the plate as loose pieces.
        union() {
            intersection() {
                chassis_plate();
                union() {
                    translate([SPLIT_X - LAP_LEN - 500 - CLEAR/2, -200, -200])
                        cube([500, 400, 400]);
                    translate([SPLIT_X - LAP_LEN - CLEAR/2, -200, -200])
                        cube([LAP_LEN, 400, 200 + LAP_Z - CLEAR/2]);
                }
            }
            at_latches() latch_post();
        }
        split_bolts();
    }
}

PART = "chassis";
if      (PART == "chassis")       chassis_plate();
else if (PART == "chassis_front") chassis_front();
else if (PART == "chassis_rear")  chassis_rear();
else if (PART == "roof")          roof();
