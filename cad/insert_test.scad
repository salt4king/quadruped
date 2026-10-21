// A coupon for finding the right pilot hole for YOUR inserts.
//
// Heat-set inserts grip because the hole is UNDER their outside
// diameter and the brass displaces plastic into the knurl. Too big and
// it spins; too small and it will not seat square. The right number
// depends on the insert, the plastic and how hot you run the iron, so
// it is measured, not looked up.
//
// Print this, melt one insert into each hole, then try to turn each one
// with a screw done up hand-tight. Use the largest hole that does NOT
// spin -- that is your pilot. Tell it to params.scad as M3_INSERT.
//
// 8 mm thick, the same as the real pads, so the melt depth matches.

PILOTS = [3.5, 3.6, 3.7, 3.8, 3.9, 4.0];
PITCH  = 13;
T      = 8;
W      = 22;
$fn = 48;

L = len(PILOTS) * PITCH + 8;

difference() {
    translate([-4, -W/2, 0]) cube([L, W, T]);
    for (i = [0 : len(PILOTS) - 1])
        translate([i*PITCH + 4, 2, -1])
            cylinder(d = PILOTS[i], h = T + 2);
    // the size, sunk into the face beside each hole
    for (i = [0 : len(PILOTS) - 1])
        translate([i*PITCH + 4, -8.5, T - 0.6])
            linear_extrude(1)
                text(str(PILOTS[i]), size = 3.4, halign = "center",
                     font = "Liberation Sans");
}
