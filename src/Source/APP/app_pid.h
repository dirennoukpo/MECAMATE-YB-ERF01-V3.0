#ifndef __APP_PID_H__
#define __APP_PID_H__

#include "stdint.h"

#define PID_SUNRISE_KP  (1.5f)
#define PID_SUNRISE_KI  (0.08f)
#define PID_SUNRISE_KD  (0.5f)

/* Velocity PID defaults, TUNED under load (17/07/2026, final robot, 7 kg on the ground).
 * Was 0.8 / 0.06 / 0.5 (Yahboom stock, only ever validated wheels-free). Under load the
 * stock Ki=0.06 wound up during the slower loaded rise and overshot 15 % on a 0.5 m/s step.
 * Lowering Ki to 0.045 halves the overshoot to ~8 % with a clean ~1 % steady-state error;
 * Kp 1.0 / Kd 0.6 keep the rise near ~540 ms. Smoothness over speed, which suits the 15 kg
 * payload to come. These are also written to the card's data flash, so a running board keeps
 * them across power cycles; this default only takes over after a FULL flash reset. See TUNING.md. */
#define PID_DEF_KP      (1.0f)
#define PID_DEF_KI      (0.045f)
#define PID_DEF_KD      (0.6f)

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
