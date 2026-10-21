include <params.scad>
use <servo.scad>
use <leg.scad>
use <shoulder.scad>
use <chassis.scad>
use <hip.scad>

// Driven by check_assembly.py. Renders the bracket intersected with the
// volume a servo needs free -- either the channel it slides in through,
// or the slot its cable leaves by. Anything that comes out is material
// standing in the way.
//
// The servo placements here are the ones the real parts are built to, so
// if a bracket moves and this file is not updated the numbers go wrong
// in the obvious direction (the test starts failing), not the dangerous
// one.

$fn = 24;
ASSEMBLY_TEST  = "slide";    // "slide" | "wire" | "driver" | "pad" | "cap"
ASSEMBLY_SERVO = "knee";     // "knee" | "pitch" | "roll"
SRC = "leg.scad";
PART = "thigh";
ARM = 1;
SIDE = 1;      // mount handedness, -1 for the roll servo

module bracket() {
    if      (SRC == "leg.scad")      thigh();
    else if (SRC == "shoulder.scad") shoulder(ARM);
    else if (SRC == "chassis.scad")  chassis_plate();
}

// Where each servo sits, in its OWN frame, matching the part files.
module at_servo() {
    if (ASSEMBLY_SERVO == "knee")
        translate([0, 0, -CRANK_C]) translate([0, KNEE_FACE, 0])
            rotate([90,0,0]) children();
    else if (ASSEMBLY_SERVO == "pitch")
        scale([ARM, 1, 1]) translate([0, PITCH_FACE, 0])
            rotate([90,0,0]) children();
    else if (ASSEMBLY_SERVO == "roll")
        // chassis_plate() is drawn in the chassis frame, so the roll
        // servo has to be placed there too -- front-left corner. Via
        // hip.scad, which is the frame the chassis cuts the pocket
        // with. It was a local copy of the transform, and when the
        // servo was laid flat this file went on testing the old
        // orientation: 3.1 cm3 "blocking the slide", all of it the
        // test looking in the wrong direction.
        roll_frame(1, 1) children();
}

// The wanted-free volume, MINUS the servo itself: a bracket is allowed to
// touch where the servo finally sits (that is the pocket), just not
// anywhere along the way in.
module wanted_free() {
    difference() {
        if      (ASSEMBLY_TEST == "slide")  ds3225_slide(SERVO_FIT, [0,0,-SIDE*60]);
        else if (ASSEMBLY_TEST == "driver") ds3225_driver(side = SIDE);
        else                                ds3225_wire();
        ds3225(SERVO_FIT + 0.2);
    }
}

// The opposite question: is there material where the screws need it.
// A pilot hole in thin air holds nothing.
module wanted_solid() {
    difference() {
        ds3225_pads(side = SIDE);
        ds3225_inserts(side = SIDE);
    }
}

// The cap has to drop into its seat without fouling the bracket. If the
// seat was not cut, or was cut on the wrong side, this comes out solid.
module cap_in_place() { ds3225_filler(3.0, SIDE); }

if (ASSEMBLY_TEST == "cap")
    intersection() { bracket(); at_servo() cap_in_place(); }
else
    intersection() {
        bracket();
        at_servo() { if (ASSEMBLY_TEST == "pad") wanted_solid(); else wanted_free(); }
    }
