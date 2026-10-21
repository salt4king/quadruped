include <params.scad>

// DS3225 servo, modelled for fit checking and for subtracting pockets.
//
// ORIGIN IS THE OUTPUT SHAFT AXIS, at the top face.
//
// That choice matters. Every joint in the kinematic model is a line in
// space; putting the servo's origin on that line means a bracket places a
// servo by translating to the joint and rotating to the axis -- no offset
// arithmetic, nothing to get wrong by 10 mm. The body hangs off in -Z and
// -X from there, which is exactly how it sits on the real part.

module ds3225(clearance = 0) {
    c  = clearance;
    bx = -SERVO_SHAFT_OFF;

    translate([bx, 0, -SERVO_H/2])
        cube([SERVO_L + 2*c, SERVO_W + 2*c, SERVO_H + 2*c], center = true);

    translate([bx, 0, -(SERVO_H - SERVO_FLANGE_Z) - SERVO_FLANGE_TH/2])
        cube([SERVO_FLANGE_L + 2*c, SERVO_W + 2*c, SERVO_FLANGE_TH + 2*c],
             center = true);

    // Gearbox boss around the spline, and the spline itself.
    cylinder(d = 13 + 2*c, h = 3 + c);
    cylinder(d = SERVO_SPLINE_D + 2*c, h = 7 + c);
}

// -------------------------------------------------------------------
// HOLDING THE SERVO DOWN
//
// The four ears are the only places a standard servo can be fastened,
// and they sit 7 mm PROUD of the body on each side -- outside the
// 40 x 20 footprint the body needs to slide through. So the screws land
// beside the channel, not across it, and the two never fight.
//
// An M3 screw passes through the ear's own 4.2 mm hole and bites a
// heat-set insert in the bracket. Not a nut: three of the four mounts
// have no second side to reach.
// -------------------------------------------------------------------
PAD_D = M3_INSERT + 2*3.0;      // insert plus a wall
PAD_H = 8.0;                    // insert is ~5 mm; leave it something to sit in
EAR_Z = (SERVO_H - SERVO_FLANGE_Z) + SERVO_FLANGE_TH;   // ear underside, 12.9

// THE WHOLE MOUNT, as ONE BLOCK.
//
// Built as a solid and hollowed out by the standard cuts below, rather
// than assembled from a shell and four pads. Assembling it was the
// mistake: the shell stopped below the ears, the pads sat above them,
// and nothing joined the two -- four 1 cm3 discs floating in space that
// every other check in this project was happy with. A block minus the
// voids cannot come apart, because it never was apart.
//
// What survives the cuts: a frame above the ears carrying the four
// inserts, side rails down past the ear band where the ears are not,
// and a floor under the body. Open on the side the servo arrives from.
// The block grows with SERVO_FIT so the wall around the pocket stays
// at `wall`. Without this, widening the pocket just eats the housing:
// 3 mm of wall would have become 2.4.
module ds3225_mount_solid(wall = 3.0, side = 1) {
    // Either way round, the block wraps the WHOLE body. The side = -1
    // version used to stop at the pads and leave the far half of the
    // servo hanging out of its own pocket -- which matters most for the
    // roll servo, the one carrying a whole leg, and which is the one
    // that cannot have a filler because its open side is where the
    // shoulder goes.
    top = side > 0 ? -(EAR_Z - SERVO_FLANGE_TH) + PAD_H : wall;
    bot = side > 0 ? -(SERVO_H + wall) : -(SERVO_H + wall);
    translate([-SERVO_SHAFT_OFF, 0, (top + bot)/2])
        cube([SERVO_FLANGE_L + 2*SERVO_FIT + 2*wall,
              SERVO_W + 2*SERVO_FIT + 2*wall, top - bot],
             center = true);
}

// Everything that has to come out of it. One call, so a bracket cannot
// cut the pocket and forget the channel.
module ds3225_mount_cuts(clearance = SERVO_FIT, side = 1) {
    ds3225(clearance);
    ds3225_slide(clearance, [0, 0, -side * 60]);
    ds3225_driver(side = side);
    ds3225_wire();
    ds3225_inserts(side = side);
}

// A shallow counterbore at the bolt circle, so a STOCK screw length
// lands inside the horn's 2 mm rather than short of it or through it.
// The horn is the only thread in the joint and it is 2 mm thick, so
// the usable window for a screw is 2 mm wide -- miss it low and the
// screw spins, miss it high and it bottoms on the servo's own output
// boss before the head is down.
module ds3225_horn_access(depth, d = 6.4) {
    for (a = [0:90:359]) rotate([0, 0, a])
        translate([HORN_SCREW_R, 0, -0.01]) cylinder(d = d, h = depth + 0.01);
}

module ds3225_ear_at() {
    bx = -SERVO_SHAFT_OFF;
    for (sx = [-1, 1], sy = [-1, 1])
        translate([bx + sx*SERVO_HOLE_DX/2, sy*SERVO_HOLE_DY/2,
                   -(SERVO_H - SERVO_FLANGE_Z) - SERVO_FLANGE_TH/2])
            children();
}

// Clearance holes, for cutting straight through a wall the ear sits on.
module ds3225_holes(depth = 40) {
    ds3225_ear_at() cylinder(d = SERVO_HOLE_D, h = depth, center = true);
}

// Material for the insert, on the SHAFT side of the ear -- so the servo
// hangs from the pads and the screw head is on the far side, out in the
// open where a driver can reach it.
//
// Getting this backwards is easy and it looks fine: the pads are solid,
// the pilot holes are there, the screws have something to bite. They are
// simply unreachable, because the driver would have to come up through
// the robot. There are 10.4 mm between the ear and the shaft face and
// the pad lives in those.
// `side` picks which face of the ear the pads sit on, and with it the
// whole handedness of the mount: the servo always arrives from the
// opposite side and the driver always works from there too.
//
//   +1  pads on the SHAFT side -- servo comes in from the body side
//   -1  pads on the BODY side  -- servo comes in over the shaft
//
// It is per mount, not a constant, because the three servos face
// differently. The roll servo's body points into the chassis: give it
// +1 like the others and the only way to fit it is from inside the
// robot, and the 60 mm channel that implies cuts its own mount off the
// plate -- 7 cm3 of loose plastic a side, which is how this was found.
module ds3225_pads(h = PAD_H, side = 1) {
    ds3225_ear_at()
        translate([0, 0, side * (SERVO_FLANGE_TH/2 + h/2)])
            cylinder(d = PAD_D, h = h, center = true);
}

// The insert pilots. Subtract after the pads.
module ds3225_inserts(h = PAD_H, side = 1) {
    ds3225_ear_at()
        translate([0, 0, side * (SERVO_FLANGE_TH/2 + h/2 - 0.01)])
            cylinder(d = M3_INSERT, h = h, center = true);
}

// -------------------------------------------------------------------
// THE FILLER
//
// The slide channel has to be there -- without it the servo cannot get
// into its pocket at all -- but once the servo IS in, most of that
// channel is empty air around it, and the mount is left looking like an
// open slot with a servo loose in the middle of it.
//
// So this is the channel's void, minus the servo: exactly the gaps, as
// one piece. It goes in behind the servo along the same channel, wraps
// its sides and its far end, and the four mounting screws pass straight
// through it on their way into the inserts -- so it is held by the same
// screws rather than needing its own.
//
// Computed from the channel rather than drawn, so it cannot drift: move
// the servo or resize the pocket and the filler follows.
// -------------------------------------------------------------------
// `fit` must match what the BRACKET cut its pocket with, or the filler
// is the wrong size for the void it is meant to close -- it defaulted to
// 0.3 while the pockets moved to SERVO_FIT, which left it 0.3 undersize
// all round. `slide` is its own sliding clearance into that pocket, and
// it is separate: the inner face wants to sit exactly where the servo
// void is, the outer face wants to be a little smaller than the channel.
module ds3225_filler(wall = 3.0, side = 1, fit = SERVO_FIT, slide = 0.3) {
    difference() {
        intersection() {
            ds3225_mount_solid(wall, side);
            // fit - slide, so the gap to the pocket wall IS `slide`.
            // fit - 2*slide leaves 0.6 a side, which is what the old
            // hardcoded version did and why the filler rattled.
            scale([1, 1, side]) ds3225_slide(fit - slide);
        }
        ds3225(fit);                            // the servo sits here
        ds3225_wire(grow = 3);                  // its cable leaves here,
                                                // with room to bend
        ds3225_driver(side = side);             // a driver comes in here
        // and the screws pass through on the way to the inserts
        ds3225_ear_at() translate([0, 0, -side * 40])
            cylinder(d = M3_CLEAR, h = 80);
    }
}

// Room to get a driver onto each screw: down the screw axis from the
// ear, on the side the head sits. A mount you cannot reach is a mount
// you cannot do up.
//
// This gets SUBTRACTED from every bracket, as four scallops in the side
// of the housing. It is not optional and it is not avoidable by making
// the housing neater: the ear holes are 49.5 mm apart because the servo
// says so, and a shell round a 40 mm body with any printable wall
// reaches past 21.25 mm. So the shell is relieved where the screws are.
//
// Keeping the check afterwards is still worth it -- it is what notices
// when some later rib or boss grows across the same line.
module ds3225_driver(h = 45, d = 7, side = 1) {
    ds3225_ear_at() translate([0, 0, -side * h/2])
        cylinder(d = d, h = h, center = true);
}

// Clearance envelope: the servo plus room for its wires, used to check that
// a bracket is not about to be printed through the cable exit.
module ds3225_envelope() {
    ds3225(CLEAR);
    ds3225_wire();
}

// THE CABLE. It leaves the far end of the body from the bottom corner,
// so every bracket needs a slot there -- a housing that is a closed box
// traps the lead and you find out with the servo already bolted in.
WIRE_W = 10; WIRE_H = 10;
// THE CABLE LEAVES ON THE SHAFT'S END, not the far one.
//
// This was drawn at -SERVO_SHAFT_OFF - SERVO_L/2, which is the end
// AWAY from the output shaft -- the intuitive place, and wrong. On a
// real DS3225MG the lead comes out of the end the splined output sits
// on. Measured on the printed part with a servo in it: the slot was
// 40 mm from the cable, so the lead had nowhere to go and the filler
// behind it was solid where the wire wanted to be.
//
// The ears are at z = -11.65 and this sits at -39.4..-29.4, so moving
// it to this end does not touch the mounting pads or their inserts --
// they are 18 mm higher up.
// `grow` opens it out, which only the FILLER asks for. The bracket's
// slot just has to let the lead through; the filler is the thing the
// lead has to bend around on its way out, and it carries no load, so
// it can give up the room for free.
module ds3225_wire(run = 25, grow = 0) {
    translate([-SERVO_SHAFT_OFF + SERVO_L/2 + run/2, 0, -SERVO_H + WIRE_H/2 + 1])
        cube([run, WIRE_W + 2*grow, WIRE_H + 2*grow], center = true);
}

// THE SWEPT VOLUME the servo travels through on its way in.
//
// A pocket that is merely servo-shaped cannot be assembled. The part has
// to ARRIVE from somewhere, and a 3 mm wall on every face of a closed box
// means the only way in is through the plastic. Subtract this as well as
// the servo itself and the bracket keeps an open channel.
//
// Swept per piece rather than as one hull: the flange is wider than the
// body, and hulling the two together would cut a 54 mm slot the whole
// length instead of a 40 mm one.
module ds3225_slide(clearance = 0, d = [0, 0, -60]) {
    c  = clearance;
    bx = -SERVO_SHAFT_OFF;
    hull() for (k = [[0,0,0], d]) translate(k)
        translate([bx, 0, -SERVO_H/2])
            cube([SERVO_L + 2*c, SERVO_W + 2*c, SERVO_H + 2*c], center = true);
    hull() for (k = [[0,0,0], d]) translate(k) cylinder(d = 13 + 2*c, h = 3 + c);
    // The FLANGE sweeps too. The ears are part of the servo and they
    // travel in with it, so a channel sized to the body alone is a
    // channel the servo cannot actually enter -- which is the whole
    // failure this module exists to prevent. They are only clear of the
    // mounting pads because the pads sit on the OTHER side of the ear.
    hull() for (k = [[0,0,0], d]) translate(k)
        translate([bx, 0, -(SERVO_H - SERVO_FLANGE_Z) - SERVO_FLANGE_TH/2])
            cube([SERVO_FLANGE_L + 2*c, SERVO_W + 2*c, SERVO_FLANGE_TH + 2*c],
                 center = true);
}
// -------------------------------------------------------------------
// EVERYTHING A DRIVEN PART NEEDS AT THE HORN, in one place.
//
// Three things have to clear, and they are three different diameters
// at three different depths:
//
//   horn       22 mm disc, 2 mm deep, sits in a pocket
//   gearbox    13 mm boss, 3 mm proud of the shaft face
//   spline     5.9 mm, 7 mm proud
//
// Boring one 16 mm hole for the lot -- which is what the thigh did --
// also bores away the four screw holes, because they sit on a 15 mm
// circle and a 16 mm hole has a 16 mm mouth. The result looks like a
// big hole in the middle of the part and leaves the horn with nothing
// to bolt to. Stepping it keeps the screws in solid material.
//
// Cut this into the face of whatever the horn drives, with the servo's
// own frame: translate to the shaft, rotate to the axis, call it.
// -------------------------------------------------------------------
HORN_SCREW_R = 7.5;     // the horn's own bolt circle, radius

// All of it cut from the shaft face INTO the part, which is the +Z
// side -- ds3225() puts the body at -Z, so the thing being driven is
// always on the other side. Cutting the other way puts the holes in
// the air beside the servo, which is what the thigh had.
module ds3225_horn_cuts(clearance = 0.3, depth = 12) {
    c = clearance;
    // the horn's disc, recessed into the part's face
    translate([0, 0, -0.01]) cylinder(d = SERVO_HORN_D + 2*c, h = SERVO_HORN_TH + c);
    // the gearbox boss behind it
    translate([0, 0, -0.01]) cylinder(d = 13 + 2*c, h = 3 + c);
    // the spline, which stands furthest proud and needs least width
    translate([0, 0, -0.01]) cylinder(d = SERVO_SPLINE_D + 3, h = 7 + c);
    // and the four screws into the horn, right through the part
    for (a = [0:90:359]) rotate([0, 0, a])
        translate([HORN_SCREW_R, 0, -0.01]) cylinder(d = M3_CLEAR, h = depth);
}

// A round horn, for checking the link's horn pocket.
module servo_horn() {
    cylinder(d = SERVO_HORN_D, h = SERVO_HORN_TH);
    cylinder(d = 8, h = SERVO_HORN_TH + 2);
}

// The idler bearing that supports the far side of every joint.
module bearing_624() {
    difference() {
        cylinder(d = BEARING_OD, h = BEARING_TH);
        translate([0, 0, -1]) cylinder(d = BEARING_ID, h = BEARING_TH + 2);
    }
}

// Pocket to press a 624 into. Slightly undersized -- press fits want
// interference, not clearance.
module bearing_pocket(extra = 0) {
    cylinder(d = BEARING_OD - 2*BEARING_FIT, h = BEARING_TH + extra);
}

SHOW = 1;
if (SHOW) {
    ds3225();
    color("red") ds3225_holes(60);
    translate([0, 0, 7]) color("silver") servo_horn();
}
