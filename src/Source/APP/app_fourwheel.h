#ifndef __APP_FOURWHEEL_H__
#define __APP_FOURWHEEL_H__

#include "stdint.h"


// Half the track width -- the ONLY constant that scales the reported Vz (angular
// velocity from wheel odometry): Vz = (V_right - V_left) / (2 * FOURWHEEL_APB).
// MecaMate final robot: track measured centre-to-centre = 430 mm (15/07/2026),
// so APB = 215 mm. Was 164.555 (the stock value, assumed a 329 mm track).
// This robot never commands v_z (it steers with 4 servos), so this ONLY affects the
// Vz telemetry, not how the robot drives. NOTE: Vz is also scaled by the wheel
// circumference (FOURWHEEL_CIRCLE_MM); it is fully correct only once that is also
// calibrated on the ground.
#define FOURWHEEL_APB                  (215.0f)


// Displacement of one full wheel revolution of the four-wheel car, unit: MM.
// MecaMate final robot: 120 mm wheels -> pi * 120 = 376.99 mm.
// This is the ONLY constant that turns encoder pulses into mm/s for the four-wheel
// (differential) chassis, so it must match the real wheel or every reported speed and
// every closed-loop distance is off by the same factor. The PID cannot catch this:
// it closes on the encoder, so it converges perfectly in pulses while the robot runs
// at the wrong ground speed.
// NOTE: 376.99 is the FREE rolling circumference. Under the ~7 kg load the tyre
// compresses a few %, so the definitive value is measured by rolling the robot over a
// known ground distance and comparing with the encoder. Motors are unchanged, so the
// encoder resolution (ENCODER_CIRCLE_330 = 1320 pulses/rev) stays correct.
#define FOURWHEEL_CIRCLE_MM            (376.99f)


void Fourwheel_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);
void Fourwheel_State(uint8_t state, uint16_t speed, uint8_t adjust);
void Fourwheel_Yaw_Calc(float yaw);

#endif /*__APP_FOURWHEEL_H__*/
