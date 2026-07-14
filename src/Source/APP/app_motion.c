#include "app_motion.h"
#include "app.h"
#include "app_pid.h"
#include "app_bat.h"
#include "app_mecanum.h"
#include "app_ackerman.h"
#include "app_fourwheel.h"
#include "app_flash.h"

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_usart.h"
#include "icm20948.h"
#include "bsp_mpu9250.h"


// Encoder data before/after 10 ms
int g_Encoder_All_Now[MAX_MOTOR] = {0};
int g_Encoder_All_Last[MAX_MOTOR] = {0};

int g_Encoder_All_Offset[MAX_MOTOR] = {0};

uint8_t g_start_ctrl = 0;

car_data_t car_data;
motor_data_t motor_data;

uint8_t g_yaw_adjust = 0;
car_type_t g_car_type = CAR_MECANUM;



static float Motion_Get_Circle_Pulse(void)
{
    float temp = 0;
    switch (g_car_type)
    {
    case CAR_MECANUM:
        temp = ENCODER_CIRCLE_330;
        break;
    case CAR_MECANUM_MAX:
        temp = ENCODER_CIRCLE_205;
        break;
    case CAR_FOURWHEEL:
        temp = ENCODER_CIRCLE_330;
        break;
    case CAR_ACKERMAN:
        temp = ENCODER_CIRCLE_550;
        break;
    case CAR_SUNRISE:
        temp = ENCODER_CIRCLE_450;
        break;
    default:
        temp = ENCODER_CIRCLE_330;
        break;
    }
    return temp;
}


#if ENABLE_REAL_WHEEL
// Actual wheel revolutions, unit: XX rev/min
int real_circle[MAX_MOTOR] = {0};

// Actual wheel speed, unit: m/s
float real_circle_speed[MAX_MOTOR] = {0};

// Actual car angular velocity
float real_motion_angular = 0.0;

void* Motion_Real_Circle_Speed(uint8_t index)
{
    if (index == 1) return (int*) real_circle;
    if (index == 2) return (float*) real_circle_speed;
    return NULL;
}
#endif


// Only used to display data added to the debug output.
void* Motion_Get_Data(uint8_t index)
{
    if (index == 1) return (int*)g_Encoder_All_Now;
    if (index == 2) return (int*)g_Encoder_All_Last;
    if (index == 3) return (int*)g_Encoder_All_Offset;
    return NULL;
}

// Get the motor speed
void Motion_Get_Motor_Speed(float* speed)
{
    for (int i = 0; i < 4; i++)
    {
        speed[i] = motor_data.speed_mm_s[i];
        
    }
}


// Set the yaw adjust state, if enabled refresh the target angle.
void Motion_Set_Yaw_Adjust(uint8_t adjust)
{
    if (adjust == 0)
    {
        g_yaw_adjust = 0;
    }
    else
    {
        g_yaw_adjust = 1;
    }
    if (g_yaw_adjust)
    {
        if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
        {
            PID_Yaw_Reset(ICM20948_Get_Yaw_Now());
        }
        else if (Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
        {
            PID_Yaw_Reset(MPU_Get_Yaw_Now());
        }
    }
}

// Return the yaw adjust state.
uint8_t Motion_Get_Yaw_Adjust(void)
{
    return g_yaw_adjust;
}


// Control the car motion, Motor_X=[-3600, 3600], out of range is invalid.
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    if (Motor_1 >= -MOTOR_MAX_PULSE && Motor_1 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(MOTOR_ID_M1, Motor_1);
    }
    if (Motor_2 >= -MOTOR_MAX_PULSE && Motor_2 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(MOTOR_ID_M2, Motor_2);
    }
    if (Motor_3 >= -MOTOR_MAX_PULSE && Motor_3 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(MOTOR_ID_M3, Motor_3);
    }
    if (Motor_4 >= -MOTOR_MAX_PULSE && Motor_4 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(MOTOR_ID_M4, Motor_4);
    }
}

// Stop the car
void Motion_Stop(uint8_t brake)
{
    Motion_Set_Speed(0, 0, 0, 0);
    PID_Clear_Motor(MAX_MOTOR);
    g_start_ctrl = 0;
    g_yaw_adjust = 0;
    Motor_Stop(brake);
}


// speed_mX=[-1000, 1000], unit: mm/s
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4)
{
    g_start_ctrl = 1;
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    motor_data.speed_set[2] = speed_m3;
    motor_data.speed_set[3] = speed_m4;
    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        PID_Set_Motor_Target(i, motor_data.speed_set[i]*1.0);
    }
}

// Add yaw correction to the car motion direction
void Motion_Yaw_Calc(float yaw)
{
    switch (g_car_type)
    {
    case CAR_MECANUM:
    {
        Mecanum_Yaw_Calc(yaw);
        break;
    }
    case CAR_MECANUM_MAX:
    {
        Mecanum_Yaw_Calc(yaw);
        break;
    }
    case CAR_FOURWHEEL:
    {
        Fourwheel_Yaw_Calc(yaw);
        break;
    }
    // case CAR_ACKERMAN:
    // {
    //     Ackerman_Yaw_Calc(yaw);
    //     break;
    // }    
    // case CAR_SUNRISE:
    // {
    //     Mecanum_Yaw_Calc(yaw);
    //     break;
    // }
    default:
        break;
    }
}


// Read the current speed of each wheel from the encoder, unit mm/s
void Motion_Get_Speed(car_data_t* car)
{
    int i = 0;
    float speed_mm[MAX_MOTOR] = {0};
    float circle_mm = Motion_Get_Circle_MM();
    float circle_pulse = Motion_Get_Circle_Pulse();
    float robot_APB = Motion_Get_APB();

    Motion_Get_Encoder();

    // Compute the wheel speed, unit mm/s.
    for (i = 0; i < 4; i++)
    {
        speed_mm[i] = (g_Encoder_All_Offset[i]) * 100 * circle_mm / circle_pulse;
    }
    switch (g_car_type)
    {
    case CAR_MECANUM:
    {
        car->Vx = (speed_mm[0] + speed_mm[1] + speed_mm[2] + speed_mm[3]) / 4;
        car->Vy = -(speed_mm[0] - speed_mm[1] - speed_mm[2] + speed_mm[3]) / 4;
        car->Vz = -(speed_mm[0] + speed_mm[1] - speed_mm[2] - speed_mm[3]) / 4.0f / robot_APB * 1000;
        break;
    }
    case CAR_MECANUM_MAX:
    {
        car->Vx = (speed_mm[0] + speed_mm[1] + speed_mm[2] + speed_mm[3]) / 4;
        car->Vy = -(speed_mm[0] - speed_mm[1] - speed_mm[2] + speed_mm[3]) / 4;
        car->Vz = -(speed_mm[0] + speed_mm[1] - speed_mm[2] - speed_mm[3]) / 4.0f / robot_APB * 1000;
        break;
    }
    case CAR_FOURWHEEL:
    {
        car->Vx = (speed_mm[0] + speed_mm[1] + speed_mm[2] + speed_mm[3]) / 4;
        car->Vy = 0;
        car->Vz = -(speed_mm[0] + speed_mm[1] - speed_mm[2] - speed_mm[3]) / 4.0f / robot_APB * 1000;
        break;
    }
    case CAR_ACKERMAN:
    {
        car->Vx = (speed_mm[1] + speed_mm[3]) / 2;
        car->Vy = Ackerman_Get_Steer_Angle();
        car->Vz = -(speed_mm[1] - speed_mm[3]) * 1000 / robot_APB;
        break;
    }    
    case CAR_SUNRISE:
    {
        car->Vx = (speed_mm[0] + speed_mm[1] + speed_mm[2] + speed_mm[3]) / 4;
        car->Vy = -(speed_mm[0] - speed_mm[1] - speed_mm[2] + speed_mm[3]) / 4;
        car->Vz = -(speed_mm[0] + speed_mm[1] - speed_mm[2] - speed_mm[3]) / 4.0f / robot_APB * 1000;
        break;
    }
    default:
        break;
    }

    if (g_start_ctrl)
    {
        for (i = 0; i < MAX_MOTOR; i++)
        {
            motor_data.speed_mm_s[i] = speed_mm[i];
        }
        
        #if ENABLE_YAW_ADJUST
        if (g_yaw_adjust)
        {
            if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
            {
                Motion_Yaw_Calc(ICM20948_Get_Yaw_Now());
            }
            else if (Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
            {
                Motion_Yaw_Calc(MPU_Get_Yaw_Now());
            }
        }
        #endif
        PID_Calc_Motor(&motor_data);

        #if PID_ASSISTANT_EN
        if (start_tool())
        {
            int32_t speed_send = car->Vx;
            // int32_t speed_send = (int32_t)speed_m1;
            set_computer_value(SEND_FACT_CMD, CURVES_CH1, &speed_send, 1);
        }
        #endif
    }
}

// Return half of the sum of the current car wheel axle distances
float Motion_Get_APB(void)
{
    if (g_car_type == CAR_MECANUM) return MECANUM_APB;
    if (g_car_type == CAR_MECANUM_MAX) return MECANUM_MAX_APB;
    if (g_car_type == CAR_MECANUM_MINI) return MECANUM_MINI_APB;
    if (g_car_type == CAR_ACKERMAN) return AKM_WIDTH;
    if (g_car_type == CAR_FOURWHEEL) return FOURWHEEL_APB;
    if (g_car_type == CAR_SUNRISE) return MECANUM_SUNRISE_APB;
    return MECANUM_APB;
}

// Return how many millimeters one wheel revolution of the current car covers
float Motion_Get_Circle_MM(void)
{
    if (g_car_type == CAR_MECANUM) return MECANUM_CIRCLE_MM;
    if (g_car_type == CAR_MECANUM_MAX) return MECANUM_MAX_CIRCLE_MM;
    if (g_car_type == CAR_MECANUM_MINI) return MECANUM_MINI_CIRCLE_MM;
    if (g_car_type == CAR_ACKERMAN) return AKM_CIRCLE_MM;
    if (g_car_type == CAR_FOURWHEEL) return FOURWHEEL_CIRCLE_MM;
    if (g_car_type == CAR_SUNRISE) return MECANUM_CIRCLE_MM;
    return MECANUM_CIRCLE_MM;
}

// Set the car type currently controlled.
void Motion_Set_Car_Type(car_type_t car_type)
{
    #if ENABLE_CAR_SUNRISE_ONLY
    g_car_type = CAR_SUNRISE;
    #else
    if (car_type >= CAR_TYPE_MAX) return;
    if (g_car_type == car_type) return;
    g_car_type = car_type;
    if (g_car_type == CAR_ACKERMAN)
    {
        PwmServo_Set_Angle(PWMServo_ID_S1, Ackerman_Get_Default_Angle());
    }
    #endif
}

// Return the car type currently controlled.
uint8_t Motion_Get_Car_Type(void)
{
    return (uint8_t)g_car_type;
}

// Get the encoder data and compute the pulse count difference
void Motion_Get_Encoder(void)
{
    Encoder_Get_ALL(g_Encoder_All_Now);

    for(uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        // Record the pulse count between two sampling times
        g_Encoder_All_Offset[i] = g_Encoder_All_Now[i] - g_Encoder_All_Last[i];
	    // Record the previous encoder data
	    g_Encoder_All_Last[i] = g_Encoder_All_Now[i];
    
    #if ENABLE_REAL_WHEEL
        // Compute revolutions per minute: pulses measured in 10 ms *100 for seconds *60 for minutes / pulses per revolution
        real_circle[i] = g_Encoder_All_Offset[i] * 60 * 100 / Motion_Get_Circle_Pulse();
        // Compute the wheel speed, unit m/s. Revolutions per minute * distance per revolution / 60 gives meters per second
        real_circle_speed[i] = real_circle[i] * (Motion_Get_Circle_MM() / 1000.0) / 60.0;
    #endif
    }

    #if ENABEL_DEBUG_ENCODER
    DEBUG("Encoder:%ld, %ld, %ld, %ld\n", g_Encoder_All_Now[0], g_Encoder_All_Now[1], g_Encoder_All_Now[2], g_Encoder_All_Now[3]);
    #endif
}

// Control the car motion
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust)
{
    switch (g_car_type)
    {
    case CAR_MECANUM:
    {
        Mecanum_Ctrl(V_x, V_y, V_z, adjust);
        break;
    }
    case CAR_MECANUM_MAX:
    {
        if (V_x > CAR_X3_PLUS_MAX_SPEED)  V_x = CAR_X3_PLUS_MAX_SPEED;
        if (V_x < -CAR_X3_PLUS_MAX_SPEED) V_x = -CAR_X3_PLUS_MAX_SPEED;
        if (V_y > CAR_X3_PLUS_MAX_SPEED)  V_y = CAR_X3_PLUS_MAX_SPEED;
        if (V_y < -CAR_X3_PLUS_MAX_SPEED) V_y = -CAR_X3_PLUS_MAX_SPEED;
        Mecanum_Ctrl(V_x, V_y, V_z, adjust);
        break;
    }
    case CAR_FOURWHEEL:
    {
        Fourwheel_Ctrl(V_x, V_y, V_z, adjust);
        break;
    }
    case CAR_ACKERMAN:
    {
        Ackerman_Ctrl(V_x, V_y, V_z, adjust);
        break;
    }
    case CAR_SUNRISE:
    {
        Mecanum_Ctrl(V_x, V_y, V_z, adjust);
        break;
    }
    default:
        break;
    }
}


// Control the motion state of the car
void Motion_Ctrl_State(uint8_t state, uint16_t speed, uint8_t adjust)
{
    uint16_t input_speed = speed * 10;
    switch (g_car_type)
    {
    case CAR_MECANUM:
    {
        Mecanum_State(state, input_speed, adjust);
        break;
    }
    case CAR_MECANUM_MAX:
    {
        if (input_speed > CAR_X3_PLUS_MAX_SPEED) input_speed = CAR_X3_PLUS_MAX_SPEED; 
        Mecanum_State(state, input_speed, adjust);
        break;
    }
    case CAR_FOURWHEEL:
    {
        Fourwheel_State(state, input_speed, adjust);
        break;
    }
    case CAR_ACKERMAN:
    {
        Ackerman_State(state, input_speed, adjust);
        break;
    }
    case CAR_SUNRISE:
    {
        Mecanum_State(state, input_speed, adjust);
        break;
    }
    default:
        break;
    }
}


// Send the car type to the host controller
void Motion_Send_Car_Type(void)
{
    #define LEN_BUF        7
	uint8_t data_buffer[LEN_BUF] = {0};
	uint8_t i, checknum = 0;
	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LEN_BUF-2; // Length
	data_buffer[3] = FUNC_CAR_TYPE; // Function byte
	data_buffer[4] = g_car_type & 0xff;
	data_buffer[5] = 0;
	for (i = 2; i < LEN_BUF-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LEN_BUF-1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}


// Send the car data to the host controller
void Motion_Send_Data(void)
{
    #define LEN        12
	uint8_t data_buffer[LEN] = {0};
	uint8_t i, checknum = 0;
	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LEN-2; // Length
	data_buffer[3] = FUNC_REPORT_SPEED; // Function byte
	data_buffer[4] = car_data.Vx & 0xff;
	data_buffer[5] = (car_data.Vx >> 8) & 0xff;
	data_buffer[6] = car_data.Vy & 0xff;
	data_buffer[7] = (car_data.Vy >> 8) & 0xff;
    data_buffer[8] = car_data.Vz & 0xff;
	data_buffer[9] = (car_data.Vz >> 8) & 0xff;
	data_buffer[10] = (uint8_t)(Bat_Voltage_Z10());  // Depends on the system voltage detection;

	for (i = 2; i < LEN-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LEN-1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}


// Motion control handler, called every 10 ms, mainly processes speed-related data
void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);

    if (g_start_ctrl)
    {
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1], motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}

