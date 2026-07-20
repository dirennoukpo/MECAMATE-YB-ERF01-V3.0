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

#include "FreeRTOS.h"
#include "task.h"


// Encoder data before/after 10 ms
int g_Encoder_All_Now[MAX_MOTOR] = {0};
int g_Encoder_All_Last[MAX_MOTOR] = {0};

int g_Encoder_All_Offset[MAX_MOTOR] = {0};

uint8_t g_start_ctrl = 0;

car_data_t car_data;
motor_data_t motor_data;

// Speed of the four motors after the first-order low-pass filter, in mm/s.
static float g_motor_speed_lpf[MAX_MOTOR] = {0};

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

// Get the low-pass filtered motor speed
void Motion_Get_Motor_Speed_LPF(float* speed)
{
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        speed[i] = g_motor_speed_lpf[i];
    }
}

// Clear every speed measurement.
// The speed data is only refreshed inside vTask_Speed; once that task stops
// running (on a low-battery shutdown, for instance) the host would keep reading
// the stale pre-shutdown values, so we must actively zero them here.
// Do NOT move this into Motion_Stop(): on a normal brake the wheels are still
// spinning, and zeroing there would report a speed that is simply false.
//
// LOCKING: this function must NOT use taskENTER_CRITICAL(). It is reachable from
// interrupt context: Motion_Set_Car_Type() calls it, and Motion_Set_Car_Type() is
// reached from Upper_CAN_Execute_Command(), which runs inside the CAN RX ISR
// (USB_LP_CAN1_RX0_IRQHandler). taskENTER_CRITICAL() is a FreeRTOS API call and
// is forbidden from an ISR whose priority sits above
// configMAX_SYSCALL_INTERRUPT_PRIORITY -- and the CAN ISR does (NVIC 0x90 vs the
// 0xBF threshold). This is the same class of bug already fixed in __malloc_lock
// (src/port_gcc/syscalls.c).
// portSET/CLEAR_INTERRUPT_MASK_FROM_ISR save and restore the previous BASEPRI:
// they are valid in task context, in ISR context, and before the scheduler starts.
void Motion_Clear_Speed_Data(void)
{
    uint32_t saved_basepri = portSET_INTERRUPT_MASK_FROM_ISR();

    for (int i = 0; i < MAX_MOTOR; i++)
    {
        motor_data.speed_mm_s[i] = 0;
        g_motor_speed_lpf[i] = 0;
    }
    car_data.Vx = 0;
    car_data.Vy = 0;
    car_data.Vz = 0;

    portCLEAR_INTERRUPT_MASK_FROM_ISR(saved_basepri);
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
// Open-loop PWM control: write the four PWM values AND hand control back to the host.
//
// This is what FUNC_MOTOR (the host's set_motor()) must call, never Motion_Set_Pwm()
// directly. Reason: g_start_ctrl stays at 1 after ANY set_car_motion() / set_car_run()
// (or SBUS, or CAN) command, and while it is 1, Motion_Handle() rewrites all four PWM
// registers every 10 ms with the on-board PID's output. An open-loop command written
// underneath it therefore survives for at most 10 ms before being silently overwritten.
//
// Measured on the bench, before this function existed:
//     reset_car_state() + set_motor(100 %)          -> 795 mm/s   (the real maximum)
//     set_car_motion(0.1,0,0) then set_motor(100 %) -> 100 mm/s   (the PID won)
//
// The PID is also cleared on the way out, so that a later set_car_motion() restarts
// from a zeroed integrator instead of resuming with a stale pwm_output.
//
// NOTE: this is a deliberate deviation from the stock Yahboom firmware. It matches
// what the API always claimed to do -- the Python docstring for set_motor() reads
// "Control PWM pulse of motor to control speed (speed measurement without encoder)",
// i.e. open loop. The stock behaviour contradicted its own documentation.
void Motion_Set_Pwm_Open_Loop(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    if (g_start_ctrl)
    {
        g_start_ctrl = 0;
        g_yaw_adjust = 0;
        PID_Clear_Motor(MAX_MOTOR);
    }
    Motion_Set_Pwm(Motor_1, Motor_2, Motor_3, Motor_4);
}

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
        // WHEEL ORDER IS NOT WHAT bsp_motor.h's "L1 L2 R1 R2" comment claims. Measured on
        // the final robot 19/07/2026, each motor driven alone:
        //     M1 = rear-RIGHT   M2 = front-RIGHT   M3 = rear-LEFT   M4 = front-LEFT
        // so speed_mm[0],[1] are the RIGHT side and [2],[3] the LEFT -- the mirror of what
        // this file assumed. With the stock grouping the reported yaw rate came out with the
        // WRONG SIGN, which would fight the IMU gyro inside the host EKF (both feed the same
        // yaw state, so opposite signs make the filter diverge rather than average).
        // REP-103: z up, yaw positive counter-clockwise, i.e. omega = (v_right - v_left)/track.
        car->Vz = -(speed_mm[2] + speed_mm[3] - speed_mm[0] - speed_mm[1]) / 4.0f / robot_APB * 1000;
        //          └──── LEFT (M3,M4) ────┘   └──── RIGHT (M1,M2) ────┘
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

    // Refresh the speed data whether or not the car is under control, otherwise
    // the host reads stale values as soon as the robot stops or is pushed by hand.
    for (i = 0; i < MAX_MOTOR; i++)
    {
        motor_data.speed_mm_s[i] = speed_mm[i];
        // First-order low-pass filter: y[n] = y[n-1] + alpha * (x[n] - y[n-1])
        g_motor_speed_lpf[i] += MOTOR_SPEED_LPF_ALPHA * (speed_mm[i] - g_motor_speed_lpf[i]);
    }

    if (g_start_ctrl)
    {
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
    // Changing the car type changes both the encoder pulse coefficient
    // (Motion_Get_Circle_Pulse) and the accumulation sign (Encoder_Update_Count).
    // The history accumulated in the filter is expressed on the old scale, and may
    // even carry the opposite sign, so it must be cleared and restarted.
    Motion_Clear_Speed_Data();
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


// Saturate the speed to the int16 range, so that an aberrant encoder jump cannot
// wrap the value around.
static int16_t Motion_Speed_Limit_Int16(float speed)
{
	if (speed >= 32767.0f) return 32767;
	if (speed <= -32768.0f) return -32768;
	return (int16_t)speed;
}

// Send the speed of the four motors to the host, in mm/s. `func` selects between
// the raw and the filtered speed.
// The sign convention is the same as for Vx/Vy/Vz: all four wheels are positive
// when the robot moves forward.
static void Motion_Send_Motor_Speed(uint8_t func, const float* speed)
{
    #define LEN_MOTOR_SPEED        13
	uint8_t data_buffer[LEN_MOTOR_SPEED] = {0};
	uint8_t i, checknum = 0;
	float speed_now[MAX_MOTOR] = {0};

	// This function runs in vTask_Control, while vTask_Speed rewrites the speed
	// arrays every 10 ms and can preempt it at any time -- the two tasks share the
	// same effective priority (the 10/9/8/7 values passed to xTaskCreate are
	// clamped to 4 by configMAX_PRIORITIES = 5), so they time-slice against each
	// other on the tick. Take a snapshot under a critical section first, otherwise
	// a single frame could mix wheels 0-1 from one sampling period with wheels 2-3
	// from the next -- with a valid checksum, hence undetectable by the host.
	//
	// taskENTER_CRITICAL() is legitimate HERE (unlike in Motion_Clear_Speed_Data):
	// this function is only ever called from Send_Request_Data(), which is only
	// called from vTask_Control -- task context, never an ISR.
	taskENTER_CRITICAL();
	for (i = 0; i < MAX_MOTOR; i++)
	{
		speed_now[i] = speed[i];
	}
	taskEXIT_CRITICAL();

	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LEN_MOTOR_SPEED-2; // Length
	data_buffer[3] = func;              // Function byte
	for (i = 0; i < MAX_MOTOR; i++)
	{
		int16_t value = Motion_Speed_Limit_Int16(speed_now[i]);
		data_buffer[4 + i*2] = value & 0xff;
		data_buffer[5 + i*2] = (value >> 8) & 0xff;
	}

	for (i = 2; i < LEN_MOTOR_SPEED-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LEN_MOTOR_SPEED-1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}

// Send the raw speed of the four motors to the host controller
void Motion_Send_Motor_Speed_Raw(void)
{
	Motion_Send_Motor_Speed(FUNC_REPORT_MOTOR_RAW, motor_data.speed_mm_s);
}

// Send the low-pass filtered speed of the four motors to the host controller
void Motion_Send_Motor_Speed_LPF(void)
{
	Motion_Send_Motor_Speed(FUNC_REPORT_MOTOR_LPF, g_motor_speed_lpf);
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

