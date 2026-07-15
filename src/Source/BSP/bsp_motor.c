#include "bsp_motor.h"
#include "bsp.h"
#include "app_motion.h"
#include "app_bat.h"   /* Bat_Voltage_Z10(), for the voltage-scaled dead zone */


uint8_t motor_enable = 0;


// Dead-zone offset for the CURRENT battery voltage (see the block comment in
// bsp_motor.h). Returns a pulse count, to be added to the commanded pulse.
//
// This is called from vTask_Speed every 10 ms, four times per cycle. The Cortex-M3
// has a hardware divider, so the cost is a handful of cycles -- not worth caching.
int16_t Motor_Get_Ignore_Pulse(void)
{
    int32_t v10;
    int32_t ignore;

    // The Sunrise chassis keeps its original fixed value: different motors, and it
    // is a chassis we cannot test here.
    if (Motion_Get_Car_Type() == CAR_SUNRISE)
        return MOTOR_SUNRISE_IGNORE_PULSE;

    v10 = (int32_t)Bat_Voltage_Z10();   // battery voltage x10, e.g. 83 = 8.3 V

    // Bat_Voltage_Z10() is 0 until the first ADC conversion completes, and the motors
    // can already be driven before that. Dividing by it would fault. Anything outside
    // 5..20 V is not a plausible pack either -- fall back to the stock fixed offset.
    if (v10 < 50 || v10 > 200)
        return MOTOR_IGNORE_PULSE;

    // ignore = V_breakaway / V_battery * MOTOR_MAX_PULSE, in integer maths.
    // Worst case 4100 * 3600 = 14 760 000, which needs the 32-bit intermediate.
    ignore = ((int32_t)MOTOR_DEAD_ZONE_MV * MOTOR_MAX_PULSE) / (v10 * 100);

    if (ignore < MOTOR_IGNORE_MIN) ignore = MOTOR_IGNORE_MIN;
    if (ignore > MOTOR_IGNORE_MAX) ignore = MOTOR_IGNORE_MAX;

    return (int16_t)ignore;
}

static int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    int16_t ignore = Motor_Get_Ignore_Pulse();

    if (pulse > 0) return pulse + ignore;
    if (pulse < 0) return pulse - ignore;
    return 0;
}

// Motor PWM port init, arr: auto-reload value  psc: clock prescaler
void Motor_PWM_Init(uint16_t arr, uint16_t psc)
{
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
    //Reset the Timer to its default values
    TIM_DeInit(TIM8);
    TIM_DeInit(TIM1);
    //Prescaler factor 0 means no prescaling, the TIMER frequency is then 72MHz re.TIM_Prescaler =0;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    //Set the counter overflow value, an update event occurs every xxx counts
    TIM_TimeBaseStructure.TIM_Period = arr - 1;
    //Set the clock division
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //Set the counter mode to up-counting mode
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    //Apply the configuration to TIM8
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //Set the default values
    TIM_OCStructInit(&TIM_OCInitStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //Select PWM mode or compare mode
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Compare output enable, enable PWM output to the pin
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //Set polarity: high or low
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	  //Set polarity: high or low
    //Set the duty cycle, duty cycle=(CCRx/ARR)*100% or (TIM_Pulse/TIM_Period)*100%
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);                      //TIM8 CHx output
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);                      //TIM8 CHx output
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);                      //TIM8 CHx output
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);                      //TIM8 CHx output
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);                      //TIM1 CHx output
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);                      //TIM1 CHx output
    

    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //Select PWM mode or compare mode
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; //Compare output disable
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //Complementary compare output enable, enable PWM output to the pin
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //Set polarity: high or low
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	  //Set polarity: high or low
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);                      //TIM1 CHxN output
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);                      //TIM1 CHxN output

    //Enable the PWM outputs
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

// Stop all motors
void Motor_Stop(uint8_t brake)
{
    if (brake != 0) brake = 1;
    TIM_SetCompare1(TIM8, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare2(TIM8, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare3(TIM8, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare4(TIM8, brake * MOTOR_MAX_PULSE);

    TIM_SetCompare1(TIM1, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare2(TIM1, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare3(TIM1, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare4(TIM1, brake * MOTOR_MAX_PULSE);
}

// Set motor speed, speed:±3600, 0 means stop
void Motor_Set_Pwm(uint8_t id, int16_t speed)
{
    int16_t pulse = Motor_Ignore_Dead_Zone(speed);
    // Limit the input
    if (pulse >= MOTOR_MAX_PULSE)
        pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE)
        pulse = -MOTOR_MAX_PULSE;

    switch (id)
    {
    case MOTOR_ID_M1:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M1_A = pulse;
            PWM_M1_B = 0;
        }
        else
        {
            PWM_M1_A = 0;
            PWM_M1_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M2:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M2_A = pulse;
            PWM_M2_B = 0;
        }
        else
        {
            PWM_M2_A = 0;
            PWM_M2_B = -pulse;
        }
        break;
    }

    case MOTOR_ID_M3:
    {
        if (pulse >= 0)
        {
            PWM_M3_A = pulse;
            PWM_M3_B = 0;
        }
        else
        {
            PWM_M3_A = 0;
            PWM_M3_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M4:
    {
        if (pulse >= 0)
        {
            PWM_M4_A = pulse;
            PWM_M4_B = 0;
        }
        else
        {
            PWM_M4_A = 0;
            PWM_M4_B = -pulse;
        }
        break;
    }

    default:
        break;
    }
}

// Initialize the motor pins
void MOTOR_GPIO_Init(void)
{
    /*Define a structure of type GPIO_InitTypeDef*/
    GPIO_InitTypeDef GPIO_InitStructure;
    /* Initialize the pin structure */
    gpio_t pwm[] = {
        {M1A_PORT, M1A_PIN, M1A_CLK},
        {M2A_PORT, M2A_PIN, M2A_CLK},
        {M3A_PORT, M3A_PIN, M3A_CLK},
        {M4A_PORT, M4A_PIN, M4A_CLK},

        {M1B_PORT, M1B_PIN, M1B_CLK},
        {M2B_PORT, M2B_PIN, M2B_CLK},
        {M3B_PORT, M3B_PIN, M3B_CLK},
        {M4B_PORT, M4B_PIN, M4B_CLK},
    };
    
    // Initialize the PWM pins
    for (int i = 0; i < MAX_MOTOR*2; i++)
    {
        /* PWM output pin */
        RCC_APB2PeriphClockCmd(pwm[i].clock, ENABLE);
        GPIO_InitStructure.GPIO_Pin = pwm[i].pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Alternate function push-pull output
        GPIO_Init(pwm[i].port, &GPIO_InitStructure);
    }

    motor_enable = MOTOR_ENABLE_A | MOTOR_ENABLE_B | MOTOR_ENABLE_C | MOTOR_ENABLE_D;
}

// Check whether the motor is initialized, returns 1 if yes, 0 if no
uint8_t Motor_Get_Enable_State(uint8_t enable_m)
{
    return motor_enable & enable_m;
}
