#pragma once

// Geometry, limits and rates, in one place.
//
// The numbers below are the ones the CAD was built from. They come from
// reference/quadruped/robot.py and reference/quadruped/leg.py -- if you
// change a link length here and not there, the model and the machine stop
// agreeing. Millimetres and radians.

#include <math.h>
#include <stdint.h>

// ---- links ---------------------------------------------------------------
constexpr float BODY_LENGTH = 200.0f;   // hip to hip, front to back
constexpr float BODY_WIDTH  = 110.0f;   // hip to hip, left to right
constexpr float HIP_OFFSET  = 46.0f;    // roll axis out to the thigh plane (l0)
constexpr float THIGH       = 110.0f;   // l1
constexpr float SHANK       = 130.0f;   // l2
constexpr float STAND_HEIGHT = 186.5f;  // body height when standing

// ---- joint travel --------------------------------------------------------
// Outside these a servo either stalls or drives a link into the chassis.
constexpr float ROLL_MIN = -35.0f * (float)M_PI / 180.0f;
constexpr float ROLL_MAX =  35.0f * (float)M_PI / 180.0f;
constexpr float HIP_MIN  = -90.0f * (float)M_PI / 180.0f;
constexpr float HIP_MAX  =  90.0f * (float)M_PI / 180.0f;
constexpr float KNEE_MIN = -150.0f * (float)M_PI / 180.0f;
constexpr float KNEE_MAX =  -15.0f * (float)M_PI / 180.0f;

// ---- rates ---------------------------------------------------------------
constexpr uint32_t SERVO_HZ   = 333;    // DS3225MG takes 50-330 Hz; 50 shows
                                        // its quantisation in the gait
constexpr uint32_t LOOP_HZ    = 200;    // control loop
constexpr uint32_t COMMAND_TIMEOUT_MS = 200;  // deadman: stop if the host
                                              // goes quiet for this long

// ---- gait ----------------------------------------------------------------
// Starting values from the reference gait; tune on the real machine.
constexpr float CYCLE_TIME      = 1.6f;   // seconds for one full cycle
constexpr float STEP_HEIGHT     = 40.0f;  // swing foot lift
constexpr float MAX_STEP_LENGTH = 90.0f;  // stride clamp

// ---- servos --------------------------------------------------------------
constexpr uint8_t LEG_COUNT   = 4;
constexpr uint8_t JOINTS_PER_LEG = 3;    // roll, hip, knee
constexpr uint8_t SERVO_COUNT = LEG_COUNT * JOINTS_PER_LEG;

// TODO: pin map. Twelve PWM pins on the Teensy 4.1.
//
// The catch: FlexPWM pins that share a timer module also share a frequency,
// so pick twelve that can all be set to SERVO_HZ. Check the groupings on the
// PJRC pinout card for your board before you solder -- they are not worth
// guessing at.
//
// constexpr uint8_t SERVO_PIN[SERVO_COUNT] = { ... };

// TODO: per-servo calibration. Pulse width at zero and the microseconds per
// radian, measured, not from the datasheet -- every DS3225 is slightly
// different and the horn only fits at 1/24 turn increments. Also the sign of
// each joint, which depends on which way round you bolted the horn.
