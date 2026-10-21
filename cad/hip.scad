include <params.scad>
use <servo.scad>

// ===================================================================
// THE HIP, AS A PAIR OF FRAMES.
//
// Three files place things on the roll axis -- chassis.scad builds the
// pocket, robot.scad draws the servo sitting in it, interference.scad
// has to agree with both. They each kept their own copy of the
// transform, and they drifted: the fore/aft mirror was in two of them
// and not the third, which is how the rear legs ended up driving their
// own arm into their own servo.
//
// So the transform lives here once, in two named versions, because two
// different jobs want two different axes pointing up.
//
//   hip_at()     x along the roll axis and OUTBOARD, y outboard across
//                the body, z up. The frame to say "the arm sweeps from
//                12 to 22 mm out" in, which is what ARM_Y0/ARM_Y1 mean
//                and what the chassis has to cut room for.
//
//   roll_frame() the same, plus a quarter turn about the shaft, which
//                is the frame the SERVO lives in.
//
// WHY THE QUARTER TURN. A DS3225 is 40 long, 20 wide and 40 tall with
// the shaft at the top. Putting the shaft on the roll axis fixes one
// axis and leaves a free choice about how the body hangs off it, and
// the unrotated version stood the 40 mm length straight up: a 57 mm
// tower at each corner, with the servo's bottom 4 mm buried in the
// plate.
//
// Two things fell out of that, and both looked like separate bugs:
//
//   the thigh    at full hip extension the thigh's upper edge sweeps
//                over that tower and grazed it at 26 degrees of roll.
//                Roll is the turning joint and the gait saturates it on
//                every turn, so losing it costs real yaw rate -- 9.6
//                deg/s at a 20 degree limit against 17.6 at 25.
//
//   the floor    with the servo sunk into the plate, its slide channel
//                and driver scallops broke out through the underside,
//                so from below you looked into the pocket and saw the
//                filler. That got patched with a 2 mm skin.
//
// The quarter turn lays the body along the body's own Y instead: 26 mm
// tall, sitting ON the plate, nothing below it. HIP_Z was derived from
// the servo lying flat all along -- CHASSIS_T + (SERVO_W + 2*WALL)/2 --
// so this is the geometry finally matching the number the kinematics
// were already using.
//
// It has to point INBOARD. The body reaches 40 mm from the shaft on the
// long side, so outboard would put it at 95 mm from the centreline and
// straight through the thigh's plane at 79.
// ===================================================================

module hip_at(sx, sy) {
    translate([sx*BODY_LEN/2, sy*BODY_WID/2, HIP_Z])
        mirror([0, sy > 0 ? 0 : 1, 0])
            rotate([0, 90, 0]) children();
}

module roll_frame(sx, sy) {
    hip_at(sx, sy) rotate([0, 0, 90]) children();
}

// The mount block's own plan footprint, by projection rather than by
// arithmetic. The block hangs off the bar by design and the plate has to
// reach under it; working the rectangle out by hand is how the rear
// mounts ended up attached by 94 mm2.
//
// CLIPPED OUTBOARD, though, and this is not optional. The block reaches
// 20 mm past the roll axis to wall in the outer mounting ear, and two
// different things live in that band:
//
//   the thigh    at -35 degrees of roll it sweeps DOWN through plate
//                level and came to y = 69.6, sharing 0.49 cm3 with the
//                8 mm skin.
//
//   the arm      its swept cylinder, radius SHOULDER_SWEEP_R about the
//                roll axis, cuts the plate's TOP out to y = 75 but its
//                UNDERSIDE only to 66.8 -- so a pad reaching past that
//                keeps a wedge at z = 0 with nothing above it. That is
//                exactly what it did: two 0.009 cm3 flakes, one per
//                rear corner, which print as swarf.
//
// The arm is the tighter of the two, so the pad stops where the sweep
// meets the plate's underside, less a millimetre so the edge has some
// thickness. The block's last 9 mm overhangs, and the outer ear is held
// by the block's own 26 mm walls rather than by plate beneath it.
// Whichever of the two binds. The arm's limit moves OUTWARD as the arm
// gets deeper -- a bigger sweep radius meets the underside further out
// -- so on its own it would walk the pad into the thigh, which comes to
// y = 69.6 at full roll and does not care how big the arm is.
PAD_Y_OUT = min(BODY_WID/2
                + sqrt(SHOULDER_SWEEP_R*SHOULDER_SWEEP_R - HIP_Z*HIP_Z) - 1,
                BODY_WID/2 + SERVO_W/2 + WALL);

module roll_pad(sx, sy) {
    intersection() {
        projection() roll_frame(sx, sy) ds3225_mount_solid(WALL, 1);
        translate([sx*BODY_LEN/2, sy*(PAD_Y_OUT - 250)])
            square([500, 500], center = true);
    }
}
