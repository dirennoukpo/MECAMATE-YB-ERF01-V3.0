#ifndef __APP_MOTION_H__
#define __APP_MOTION_H__

#include "stdint.h"


// 205RPM motor, for one full wheel revolution, encoder pulse count = reduction ratio * encoder disc lines * encoder pulses (56*11*4)
#define ENCODER_CIRCLE_205           (2464.0f)

// 330RPM motor, for one full wheel revolution, encoder pulse count = reduction ratio * encoder disc lines * encoder pulses (30*11*4)
#define ENCODER_CIRCLE_330           (1320.0f)

// 450RPM motor, for one full wheel revolution, encoder pulse count = reduction ratio * encoder disc lines * encoder pulses (20*13*4)
#define ENCODER_CIRCLE_450           (1040.0f)

// 550RPM motor, for one full wheel revolution, encoder pulse count = reduction ratio * encoder disc lines * encoder pulses (19*11*4)
#define ENCODER_CIRCLE_550           (836.0f)

// Distance travelled for one full wheel revolution, in meters
#define DISTANCE_CIRCLE      (0.204203)


// First-order low-pass filter coefficient for the motor speeds.
// alpha = Te / (tau + Te), where Te = 10 ms is the call period of Motion_Handle.
// 0.2f gives a time constant tau = 40 ms, i.e. a cutoff around 4 Hz. The smaller
// the value, the stronger the filtering and the slower the response.
#define MOTOR_SPEED_LPF_ALPHA        (0.2f)


// Stop mode: STOP_FREE means free stop (coast), STOP_BRAKE means brake.
typedef enum _stop_mode {
    STOP_FREE = 0,
    STOP_BRAKE
} stop_mode_t;


typedef enum _motion_state {
    MOTION_STOP = 0,
    MOTION_RUN,
    MOTION_BACK,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_SPIN_LEFT,
    MOTION_SPIN_RIGHT,
    MOTION_BRAKE,

    MOTION_MAX_STATE
} motion_state_t;


typedef struct _car_data
{
    int16_t Vx;
    int16_t Vy;
    int16_t Vz;
} car_data_t;


typedef enum _car_type
{
    CAR_TYPE_NONE = 0x00,       // Reserved
    CAR_MECANUM = 0x01,         // Small chassis, small mecanum wheels X3
    CAR_MECANUM_MAX = 0x02,     // Large chassis, large mecanum wheels X3 PLUS
    CAR_MECANUM_MINI = 0x03,    // Large chassis, small mecanum wheels, none
    CAR_FOURWHEEL = 0x04,       // Four-wheel standard car X1
    CAR_ACKERMAN = 0x05,        // Ackermann car    R2
    CAR_SUNRISE = 0x06,         // Sunrise Pi car

    CAR_TYPE_MAX                // Last car type, used only for range checking
} car_type_t;


void Motion_Stop(uint8_t brake);
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4);
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);
void Motion_Ctrl_State(uint8_t state, uint16_t speed, uint8_t adjust);


void Motion_Get_Encoder(void);
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4);


void Motion_Handle(void);

void Motion_Get_Speed(car_data_t* car);
void Motion_Yaw_Calc(float yaw);

void Motion_Set_Yaw_Adjust(uint8_t adjust);
uint8_t Motion_Get_Yaw_Adjust(void);

uint8_t Motion_Get_Car_Type(void);
void Motion_Set_Car_Type(car_type_t car_type);
float Motion_Get_Circle_MM(void);
float Motion_Get_APB(void);


void Motion_Get_Motor_Speed(float* speed);
void Motion_Get_Motor_Speed_LPF(float* speed);
void Motion_Clear_Speed_Data(void);


void Motion_Send_Data(void);
void Motion_Send_Car_Type(void);
void Motion_Send_Motor_Speed_Raw(void);
void Motion_Send_Motor_Speed_LPF(void);


#endif
