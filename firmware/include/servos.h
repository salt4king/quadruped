#pragma once

// The twelve servos: angles in, PWM out.
//
// Owns the pin map, the per-servo calibration, the joint limit clamp, and
// the slew limit that stops a bad command from snapping a leg across its
// travel. Nothing above this file should know about microseconds.
