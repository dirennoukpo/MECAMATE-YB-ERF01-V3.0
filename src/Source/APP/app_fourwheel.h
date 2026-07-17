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
// Wheel circumference: this and ENCODER_CIRCLE_330 together turn encoder pulses into
// mm/s and distance, so if either is wrong every reported speed and every closed-loop
// distance is off by the same factor. The PID cannot catch it: it closes on the encoder,
// converging perfectly in pulses while the robot runs at the wrong ground speed.
// MEASURED 17/07/2026: the wheel is RIGID (120 mm, no load compression), so pi*120 =
// 376.99 mm is correct and stays. The 2.3 % odometry error found by rolling ~2 m and
// comparing to a tape was NOT the wheel -- it was the encoder count: the "30:1" gearbox
// is really ~30.68:1, so ENCODER_CIRCLE_330 went 1320 -> 1350 (see app_motion.h).
// After that fix, get_motor_speed_raw/lpf integrate to the encoder distance within 0.1 %.
#define FOURWHEEL_CIRCLE_MM            (376.99f)


void Fourwheel_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);
void Fourwheel_State(uint8_t state, uint16_t speed, uint8_t adjust);
void Fourwheel_Yaw_Calc(float yaw);

#endif /*__APP_FOURWHEEL_H__*/
