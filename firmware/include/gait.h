#pragma once

// Gait scheduler: a velocity command -> where each foot should be, now.
//
// Decides which feet are swinging and which are planted, where a swinging
// foot is along its arc, and where the body should be so the thing does not
// tip over. Output is four foot targets plus a body pose, which go to the
// kinematics.
//
// reference/quadruped/gait.py is the worked version.
