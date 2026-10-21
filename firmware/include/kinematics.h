#pragma once

// Leg kinematics: a foot position in the body frame -> three joint angles,
// and back the other way.
//
// Closed form, no iteration -- solve the roll (which plane the leg swings in)
// first, then a two-link arm in that plane. The derivation is written out in
// reference/quadruped/leg.py, which is the same maths in Python and already
// has tests against it.
//
// Watch for: the knee points FORWARD on this machine, so knee angles are
// negative; and a foot position can be unreachable, which has to fail
// loudly rather than return nonsense.
