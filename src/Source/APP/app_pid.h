#ifndef __APP_PID_H__
#define __APP_PID_H__

#include "stdint.h"

#define PID_SUNRISE_KP  (1.5f)
#define PID_SUNRISE_KI  (0.08f)
#define PID_SUNRISE_KD  (0.5f)

#define PID_DEF_KP      (0.8f)
#define PID_DEF_KI      (0.06f)
#define PID_DEF_KD      (0.5f)

#define PID_YAW_DEF_KP  (0.4)
#define PID_YAW_DEF_KI  (0.0)
#define PID_YAW_DEF_KD  (0.1)


typedef struct _pid
{
    float target_val;               //Target value
    float output_val;               //Output value
    float pwm_output;        		//PWM output value
    float Kp,Ki,Kd;          		//Proportional, integral and derivative coefficients
    float err;             			//Error value
    float err_last;          		//Previous error value

    float err_next;                 //Next error value, incremental form
    float integral;          		//Integral value, positional form
} pid_t;

typedef struct _motor_data_t
{
    float speed_mm_s[4];        // Input value, speed computed by the encoder
    float speed_pwm[4];         // Output value, PWM value computed by the PID
    int16_t speed_set[4];       // Speed setpoint value
} motor_data_t;


typedef struct
{
    float SetPoint;   // Setpoint, Desired value
    float Proportion; // Proportional constant, Proportional Const
    float Integral;   // Integral constant, Integral Const
    float Derivative; // Derivative constant, Derivative Const
    float LastError;  // Error[-1]
    float PrevError;  // Error[-2]
    float SumError;   // Sums of Errors
} PID;



void PID_Param_Init(void);

float PID_Location_Calc(pid_t *pid, float actual_val);
void PID_Calc_Motor(motor_data_t* motor);
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd);
void PID_Send_Parm_Active(uint8_t index);



void PID_Yaw_Reset(float yaw);
float PID_Yaw_Calc(float NextPoint);
void PID_Yaw_Set_Parm(float kp, float ki, float kd);

#endif /* __APP_PID_H__ */
