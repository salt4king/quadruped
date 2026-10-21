// Quadruped firmware -- entry point.
//
// Nothing is implemented yet. The order below is the one the design assumes;
// see docs/CONTROL.md for why the gait lives here and not on the host.

#include <Arduino.h>

#include "config.h"
#include "gait.h"
#include "imu.h"
#include "kinematics.h"
#include "protocol.h"
#include "servos.h"

void setup() {
    Serial.begin(115200);

    // servos.begin()    -- pins to SERVO_HZ, then hold the stand pose
    // imu.begin()
    // protocol.begin()
}

void loop() {
    // Once every 1000/LOOP_HZ ms:
    //
    //   1. protocol  -- read any new command; stop on the deadman
    //   2. imu       -- current body attitude
    //   3. gait      -- command + attitude -> four foot targets, one body pose
    //   4. kinematics-- foot targets -> twelve joint angles
    //   5. servos    -- clamp, slew-limit, write
    //
    // Step 5 must happen every period whether or not steps 1-4 did anything.
}
