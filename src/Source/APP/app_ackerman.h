#ifndef __APP_ACKERMAN_H__
#define __APP_ACKERMAN_H__

#include "stdint.h"

// Ackerman car chassis spacing
#define AKM_WIDTH                      (191.10f)    // millimeters
#define AKM_LENGTH                     (235.37f)    // millimeters

// Displacement of one full wheel revolution of the Ackerman car, unit is MM
#define AKM_CIRCLE_MM                  (215.2f)

// Servo ID and initialization angle
#define AKM_ANGLE_ID                   (PWMServo_ID_S1)
#define AKM_ANGLE_INIT                 (90)
#define AKM_ANGLE_LIMIT                (45)


// Maximum motor speed of the Ackerman car
#define AKM_MOTOR_MAX_SPEED            (2000)


int16_t Ackerman_Get_Steer_Angle(void);
void Ackerman_Steering(int16_t angle);
void Ackerman_Steering_with_car(int16_t angle);
void Ackerman_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);
void Ackerman_State(uint8_t state, uint16_t speed, uint8_t adjust);
uint16_t Ackerman_Get_Default_Angle(void);
void Ackerman_Set_Default_Angle(uint16_t angle, uint8_t forever);

void Ackerman_Send_Default_Angle(void);

void Ackerman_Yaw_Calc(float yaw);

#endif /* __APP_ACKERMAN_H__ */
