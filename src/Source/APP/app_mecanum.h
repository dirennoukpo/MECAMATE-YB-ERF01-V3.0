#ifndef __APP_MECANUM_H__
#define __APP_MECANUM_H__

#include "stdint.h"

// Small mecanum wheel car chassis spacing
#define ROBOT_WIDTH                  (169.0f)    // millimeters
#define ROBOT_LENGTH                 (160.11f)   // millimeters

// Half of the sum of the motor spacings of the small mecanum wheel chassis
// #define MECANUM_APB               ((ROBOT_WIDTH + ROBOT_LENGTH)/2.0f)
#define MECANUM_APB                  (164.555f)

// Displacement of one full turn of the small mecanum wheel, unit is mm
#define MECANUM_CIRCLE_MM            (204.203f)
#define MECANUM_MINI_CIRCLE_MM       (204.203f)


// Half of the sum of the motor spacings of the large mecanum wheel chassis
#define MECANUM_MAX_APB              (214.1f)
// Displacement of one full turn of the large mecanum wheel, unit is MM
#define MECANUM_MAX_CIRCLE_MM        (251.327f)

// Half of the sum of the motor spacings of the small mecanum wheel large chassis
#define MECANUM_MINI_APB             (174.5f)

// Half of the sum of the motor spacings of the Sunrise Pi car chassis
#define MECANUM_SUNRISE_APB          (143.68f)

// X3 PLUS car speed limit
#define CAR_X3_PLUS_MAX_SPEED        (700)




void Mecanum_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);
void Mecanum_State(uint8_t state, uint16_t speed, uint8_t adjust);

void Mecanum_Yaw_Calc(float yaw);

#endif /* __APP_MECANUM_H__ */
