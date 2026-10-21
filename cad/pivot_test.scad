// Driven by check_screws.py. Intersects a linkage pivot's hole with
// the part AS EXPORTED, which is the only way to catch a hole that was
// cut and then unioned shut again.
//
// Which is what happened to the shank's rocker. arm() cuts its own
// hole, and the hole was fine -- then the beam's knee disc, in the same
// plane and reaching 0.5 mm past the hole's centre, was unioned on top
// and filled 77 of its 100 cubic millimetres back in. It printed as a
// crescent with nothing to screw into, and nothing here noticed,
// because every other check looks at material, not at the absence of it.
//
// The probe is 0.4 mm under the nominal hole, so a facetted bore does
// not read as a blockage.
include <params.scad>

STL   = "";
PHI   = 0;    // the rocker's fixed angle; 0 where the arm runs straight
MIR   = 0;    // 1 for the right-hand print
PLANE = 0;    // inboard face of the arm's plate
THICK = 0;    // plate thickness
L     = 0;    // hole centre, out from the arm's own axis
D     = 0;    // nominal hole

intersection() {
    import(STL);
    mirror([0, MIR, 0]) rotate([0, PHI, 0])
        translate([0, PLANE, 0]) rotate([-90, 0, 0]) rotate([0, 0, 90])
            translate([L, 0, -1])
                cylinder(d = D - 0.4, h = THICK + 2, $fn = 64);
}
