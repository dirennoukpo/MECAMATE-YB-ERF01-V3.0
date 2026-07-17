#include "app_pid.h"
#include "app_motion.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_usart.h"

#include "app.h"

#define PI      (3.1415926f)


pid_t pid_motor[4];

// YAW yaw angle
PID pid_Yaw = {0, 0.4, 0, 0.1, 0, 0, 0};


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

// Initialize PID parameters
void PID_Param_Init(void)
{
    /* Speed-related initialization parameters */
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        pid_motor[i].target_val = 0.0;
        pid_motor[i].pwm_output = 0.0;
        pid_motor[i].err = 0.0;
        pid_motor[i].err_last = 0.0;
        pid_motor[i].err_next = 0.0;
        pid_motor[i].integral = 0.0;

        pid_motor[i].Kp = PID_DEF_KP;
        pid_motor[i].Ki = PID_DEF_KI;
        pid_motor[i].Kd = PID_DEF_KD;
    }

    pid_Yaw.Proportion = PID_YAW_DEF_KP;
    pid_Yaw.Integral = PID_YAW_DEF_KI;
    pid_Yaw.Derivative = PID_YAW_DEF_KD;
    
    #if PID_ASSISTANT_EN
    float pid_temp[3] = {pid_motor[0].Kp, pid_motor[0].Ki, pid_motor[0].Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // Send the P I D values to channel 1
    #endif
}

// Set PID parameters
void PID_Set_Parm(pid_t *pid, float p, float i, float d)
{
    pid->Kp = p; // Set proportional coefficient P
    pid->Ki = i; // Set integral coefficient I
    pid->Kd = d; // Set derivative coefficient D
}

// Set the PID target value
void PID_Set_Target(pid_t *pid, float temp_val)
{
    pid->target_val = temp_val; // Set the current target value
}

// Get the PID target value
float PID_Get_Target(pid_t *pid)
{
    return pid->target_val; // Set the current target value
}

// Incremental PID calculation formula.
// motor_id selects which axis's dead-zone offset this output has to leave room for --
// the offset is per-axis, so the saturation must be too, or the invariant below breaks.
float PID_Incre_Calc(pid_t *pid, uint8_t motor_id, float actual_val)
{
    /*Compute the error between target and actual value*/
    pid->err = pid->target_val - actual_val;
    /*PID algorithm implementation*/
    pid->pwm_output += pid->Kp * (pid->err - pid->err_next) 
                    + pid->Ki * pid->err 
                    + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);
    /*Propagate the errors*/
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    
    /*Return the PWM output value*/
    // The PID output is the pulse BEFORE the dead-zone offset is added by
    // Motor_Ignore_Dead_Zone(). Its saturation must therefore be exactly the room
    // left once that offset is taken out, otherwise the top of the range is either
    // wasted (output clipped later by MOTOR_MAX_PULSE) or unreachable.
    // Now that the offset follows the battery voltage AND the axis, this limit has to
    // follow both -- it used to be the hard-coded MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE.
    // Motor_Get_Ignore_Pulse() already handles the CAR_SUNRISE case internally.
    //
    // Both sides read the SAME function, which is what makes
    //     ignore + PID_sat = MOTOR_MAX_PULSE
    // true at every voltage and on every axis. That invariant is why raising
    // MOTOR_IGNORE_MAX to full scale needs no prerequisite: there is no value of the
    // offset for which the two can disagree.
    float limit = (float)(MOTOR_MAX_PULSE - Motor_Get_Ignore_Pulse(motor_id));

    if (pid->pwm_output > limit)  pid->pwm_output = limit;
    if (pid->pwm_output < -limit) pid->pwm_output = -limit;

    return pid->pwm_output;
}

// Positional PID calculation method
float PID_Location_Calc(pid_t *pid, float actual_val)
{
	/*Compute the error between target and actual value*/
    pid->err = pid->target_val - actual_val;

    /* Integral separation: drop the integral action when the error is large */
    if (pid->err > -1500 && pid->err < 1500)
    {
        pid->integral += pid->err;    // Error accumulation

        /* Limit the integral range to prevent integral saturation */
        if (pid->integral > 4000)
            pid->integral = 4000;
        else if (pid->integral < -4000)
            pid->integral = -4000;
    }

	/*PID algorithm implementation*/
    pid->output_val = pid->Kp * pid->err + 
                      pid->Ki * pid->integral + 
                      pid->Kd * (pid->err - pid->err_last);

	/*Error propagation*/
    pid->err_last = pid->err;
    
	/*Return the current actual value*/
    return pid->output_val;
}


// PID computation of the output values
void PID_Calc_Motor(motor_data_t* motor)
{
    int i;
    // float pid_out[4] = {0};
    // for (i = 0; i < MAX_MOTOR; i++)
    // {
    //     pid_out[i] = PID_Location_Calc(&pid_motor[i], 0);
    //     PID_Set_Motor_Target(i, pid_out[i]);
    // }
    
    for (i = 0; i < MAX_MOTOR; i++)
    {
        motor->speed_pwm[i] = PID_Incre_Calc(&pid_motor[i], i, motor->speed_mm_s[i]);
    }
}

// PID computation for a single channel
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed)
{
    if (motor_id >= MAX_MOTOR) return 0; 
    return PID_Incre_Calc(&pid_motor[motor_id], motor_id, now_speed);
}

// Set PID parameters, motor_id=4 sets all, =0123 sets the PID parameters of the corresponding motor.
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd)
{
    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].Kp = kp;
            pid_motor[i].Ki = ki;
            pid_motor[i].Kd = kd;
        }
        DEBUG("PID Set:%.3f, %.3f, %.3f\n", kp, ki, kd);
    }
    else
    {
        pid_motor[motor_id].Kp = kp;
        pid_motor[motor_id].Ki = ki;
        pid_motor[motor_id].Kd = kd;
        DEBUG("PID Set M%d:%.3f, %.3f, %.3f\n", motor_id+1, kp, ki, kd);
    }
}

// Clear the PID data
void PID_Clear_Motor(uint8_t motor_id)
{
    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].pwm_output = 0.0;
            pid_motor[i].err = 0.0;
            pid_motor[i].err_last = 0.0;
            pid_motor[i].err_next = 0.0;
            pid_motor[i].integral = 0.0;
        }
    }
    else
    {
        pid_motor[motor_id].pwm_output = 0.0;
        pid_motor[motor_id].err = 0.0;
        pid_motor[motor_id].err_last = 0.0;
        pid_motor[motor_id].err_next = 0.0;
        pid_motor[motor_id].integral = 0.0;
    }
}

// Set the PID target speed, unit: mm/s
void PID_Set_Motor_Target(uint8_t motor_id, float target)
{
    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].target_val = target;
        }
    }
    else
    {
        pid_motor[motor_id].target_val = target;
    }
}

// Return the PID structure array
pid_t* Pid_Get_Motor(void)
{
    return pid_motor;
}


/* Send the PID data to the host over the serial port, index=1~5, 1~4 are the four motors, 5 is YAW */
void PID_Send_Parm_Active(uint8_t index)
{
    #define LEN        12
	uint8_t data_buffer[LEN] = {0};
	uint8_t i, checknum = 0;
    if (index > 0 && index < 6)
    {
        data_buffer[0] = PTO_HEAD;
        data_buffer[1] = PTO_DEVICE_ID-1;
        data_buffer[2] = LEN-2; // Count
        data_buffer[3] = FUNC_SET_MOTOR_PID; // Function byte
        data_buffer[4] = index;
        if (index == 5)
        {
            data_buffer[3] = FUNC_SET_YAW_PID; // Function byte
            data_buffer[5] = (int32_t)(pid_Yaw.Proportion * 1000) & 0xff;
            data_buffer[6] = ((int32_t)(pid_Yaw.Proportion * 1000) >> 8) & 0xff;
            data_buffer[7] = (int32_t)(pid_Yaw.Integral * 1000) & 0xff;
            data_buffer[8] = ((int32_t)(pid_Yaw.Integral * 1000) >> 8) & 0xff;
            data_buffer[9] = (int32_t)(pid_Yaw.Derivative * 1000) & 0xff;
            data_buffer[10] = ((int32_t)(pid_Yaw.Derivative * 1000) >> 8) & 0xff;
        }
        else
        {
            data_buffer[5] = (int32_t)(pid_motor[index-1].Kp * 1000) & 0xff;
            data_buffer[6] = ((int32_t)(pid_motor[index-1].Kp * 1000) >> 8) & 0xff;
            data_buffer[7] = (int32_t)(pid_motor[index-1].Ki * 1000) & 0xff;
            data_buffer[8] = ((int32_t)(pid_motor[index-1].Ki * 1000) >> 8) & 0xff;
            data_buffer[9] = (int32_t)(pid_motor[index-1].Kd * 1000) & 0xff;
            data_buffer[10] = ((int32_t)(pid_motor[index-1].Kd * 1000) >> 8) & 0xff;
        }
        for (i = 2; i < LEN-1; i++)
        {
            checknum += data_buffer[i];
        }
        data_buffer[LEN-1] = checknum;
        USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
    }
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////



// Reset the yaw target value
void PID_Yaw_Reset(float yaw)
{
	pid_Yaw.SetPoint = yaw;
    pid_Yaw.SumError = 0;
    pid_Yaw.LastError = 0;
    pid_Yaw.PrevError = 0;
}

// Compute the yaw output value
float PID_Yaw_Calc(float NextPoint)
{
	float dError, Error;
	Error = pid_Yaw.SetPoint - NextPoint;			// Error
	pid_Yaw.SumError += Error;						// Integral
	dError = pid_Yaw.LastError - pid_Yaw.PrevError; // Current derivative
	pid_Yaw.PrevError = pid_Yaw.LastError;
	pid_Yaw.LastError = Error;

	double omega_rad = pid_Yaw.Proportion * Error			 // Proportional term
					   + pid_Yaw.Integral * pid_Yaw.SumError // Integral term
					   + pid_Yaw.Derivative * dError;		 // Derivative term

	if (omega_rad > PI / 6)
		omega_rad = PI / 6;
	if (omega_rad < -PI / 6)
		omega_rad = -PI / 6;
	return omega_rad;
}

// Set the yaw PID parameters
void PID_Yaw_Set_Parm(float kp, float ki, float kd)
{
    pid_Yaw.Proportion = kp;
    pid_Yaw.Integral = ki;
    pid_Yaw.Derivative = kd;
}

