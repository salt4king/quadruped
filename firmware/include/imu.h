#pragma once

// BNO055 over I2C: body attitude for the balance loop.
//
// Only the orientation is needed at first. Worth filtering before it reaches
// the gait -- a jumpy pitch estimate turns into twelve jumpy servos.
