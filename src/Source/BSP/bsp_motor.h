
#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "stm32f10x.h"
#include "config.h"   /* BATTERY_NOMINAL_Z10, for the pre-first-ADC-sample fallback */

#define MOTOR_A_IN1_PORT  GPIOB                /* GPIO port */
#define MOTOR_A_IN1_PIN   GPIO_Pin_0          /* GPIO pin number */
#define MOTOR_A_IN1_CLK   RCC_APB2Periph_GPIOB /* GPIO port clock */
#define MOTOR_A_IN2_PORT  GPIOB
#define MOTOR_A_IN2_PIN   GPIO_Pin_1
#define MOTOR_A_IN2_CLK   RCC_APB2Periph_GPIOB


#define MOTOR_B_IN1_PORT  GPIOB
#define MOTOR_B_IN1_PIN   GPIO_Pin_5
#define MOTOR_B_IN1_CLK   RCC_APB2Periph_GPIOB
#define MOTOR_B_IN2_PORT  GPIOB
#define MOTOR_B_IN2_PIN   GPIO_Pin_4
#define MOTOR_B_IN2_CLK   RCC_APB2Periph_GPIOB


#define MOTOR_C_IN1_PORT  GPIOD
#define MOTOR_C_IN1_PIN   GPIO_Pin_2
#define MOTOR_C_IN1_CLK   RCC_APB2Periph_GPIOD
#define MOTOR_C_IN2_PORT  GPIOC
#define MOTOR_C_IN2_PIN   GPIO_Pin_12
#define MOTOR_C_IN2_CLK   RCC_APB2Periph_GPIOC


#define MOTOR_D_IN1_PORT  GPIOC
#define MOTOR_D_IN1_PIN   GPIO_Pin_11
#define MOTOR_D_IN1_CLK   RCC_APB2Periph_GPIOC
#define MOTOR_D_IN2_PORT  GPIOC
#define MOTOR_D_IN2_PIN   GPIO_Pin_10
#define MOTOR_D_IN2_CLK   RCC_APB2Periph_GPIOC


#define M1A_PORT  GPIOA
#define M1A_PIN   GPIO_Pin_11
#define M1A_CLK   RCC_APB2Periph_GPIOA
#define M1B_PORT  GPIOA
#define M1B_PIN   GPIO_Pin_8
#define M1B_CLK   RCC_APB2Periph_GPIOA

#define M2A_PORT  GPIOB
#define M2A_PIN   GPIO_Pin_0
#define M2A_CLK   RCC_APB2Periph_GPIOB
#define M2B_PORT  GPIOB
#define M2B_PIN   GPIO_Pin_1
#define M2B_CLK   RCC_APB2Periph_GPIOB

#define M3A_PORT  GPIOC
#define M3A_PIN   GPIO_Pin_6
#define M3A_CLK   RCC_APB2Periph_GPIOC
#define M3B_PORT  GPIOC
#define M3B_PIN   GPIO_Pin_7
#define M3B_CLK   RCC_APB2Periph_GPIOC

#define M4A_PORT  GPIOC
#define M4A_PIN   GPIO_Pin_8
#define M4A_CLK   RCC_APB2Periph_GPIOC
#define M4B_PORT  GPIOC
#define M4B_PIN   GPIO_Pin_9
#define M4B_CLK   RCC_APB2Periph_GPIOC


#define PWM_M1_A  TIM8->CCR1
#define PWM_M1_B  TIM8->CCR2

#define PWM_M2_A  TIM8->CCR3
#define PWM_M2_B  TIM8->CCR4

#define PWM_M3_A  TIM1->CCR4
#define PWM_M3_B  TIM1->CCR1

#define PWM_M4_A  TIM1->CCR2
#define PWM_M4_B  TIM1->CCR3


#define MOTOR_ENABLE_A      (0x01)
#define MOTOR_ENABLE_B      (0x02)
#define MOTOR_ENABLE_C      (0x04)
#define MOTOR_ENABLE_D      (0x08)


#define MOTOR_SUNRISE_IGNORE_PULSE  (2000)
/* Fallback dead-zone offset. Reached only while Bat_Voltage_Z10() is still implausible --
 * chiefly the window between the scheduler starting and the first ADC conversion, where the
 * motors are already commandable from the CAN RX ISR. The stock literal 1600 was tuned for
 * the 12.6 V pack; deriving it from the declared pack keeps that window honest whichever
 * pack is fitted (1757 counts on a 2S, 1171 on a 3S). Both stay inside [MOTOR_IGNORE_MIN,
 * MOTOR_IGNORE_MAX]. Forward references are fine: a macro expands at its point of use. */
#define MOTOR_IGNORE_PULSE  (((MOTOR_DEAD_ZONE_MV) * (MOTOR_MAX_PULSE)) / ((BATTERY_NOMINAL_Z10) * 100))
#define MOTOR_MAX_PULSE     (3600)
#define MOTOR_FREQ_DIVIDE   (0)


/* ---------------------------------------------------------------------------
 * Dead-zone compensation, scaled with the battery voltage.
 *
 * A DC motor only breaks away from static friction above a certain voltage
 * ACROSS THE MOTOR, and the PWM only delivers  V_motor = duty * V_battery.
 * So the duty needed to start turning is  V_breakaway / V_battery  -- it depends
 * on the battery, and a single fixed offset cannot be right at two voltages.
 *
 * The stock firmware adds a fixed MOTOR_IGNORE_PULSE = 1600 (44.4 % duty), which
 * is only correct for the 12.6 V pack it was tuned on. Measured on this robot:
 *
 *   V_breakaway per motor -- TWO different robots, do not mix them up:
 *     Bench (real Rosmaster, 65 mm wheels):  M1 4.10  M2 4.15  M3 3.92  M4 4.10 V
 *     MecaMate final robot (120 mm wheels, measured 15/07 wheels-free at 12.6 V):
 *                                            M1 6.14  M2 5.63  M3 5.97  M4 6.14 V
 *   The final robot's motors start MUCH harder (~6 V vs ~4 V) even though the wheels
 *   turn MORE freely by hand: hand-turning is kinetic friction, starting under power is
 *   stiction + the motor's winding resistance. Different motors, higher start voltage.
 *
 *   With the fixed 1600, on an 8.4 V (2S) pack:
 *     1 % command -> 45 % duty -> 3.74 V  -> nothing moves
 *     the wheels only start turning around 9 % of command.
 *   And on the 12.6 V pack it OVER-compensates: 1 % command already delivers
 *     5.6 V to the motor, so the speed jumps instead of ramping.
 *
 * MOTOR_DEAD_ZONE_MV is the measured breakaway voltage. Motor_Get_Ignore_Pulse()
 * turns it back into a pulse count for whatever the battery currently reads, so
 * that 1 % of command sits right at the edge of motion and 100 % is full scale,
 * WHATEVER the pack. That also makes the host-side kS feedforward stable instead
 * of drifting as the battery discharges.
 *
 * NOTE: measured with the wheels free. Under the robot's own weight the breakaway
 * is higher -- that extra torque is the PID's job (or the host's), not this
 * offset's, which only has to get an unloaded motor moving.
 * ------------------------------------------------------------------------- */
#define MOTOR_DEAD_ZONE_MV  (6150)   /* MecaMate final robot: measured wheels-free max, */
                                     /* 15/07/2026. Was 4100 (the bench value).        */
                                     /* WHEELS-FREE value -- under the 7 kg load on the */
                                     /* ground it will be higher; re-measure and raise. */
#define MOTOR_IGNORE_MIN    (800)    /* guard: never leave less headroom than this */
#define MOTOR_IGNORE_MAX    (2400)   /* guard: keep at least 1200 counts of range  */

int16_t Motor_Get_Ignore_Pulse(void);


// MOTOR: M1 M2 M3 M4
// MOTOR: L1 L2 R1 R2
typedef enum {
    MOTOR_ID_M1 = 0,
    MOTOR_ID_M2,
    MOTOR_ID_M3,
    MOTOR_ID_M4,
    MAX_MOTOR
} Motor_ID;


void Motor_PWM_Init(uint16_t arr, uint16_t psc);

void Motor_Set_Pwm(uint8_t id, int16_t speed);
void Motor_Stop(uint8_t brake);
void MOTOR_GPIO_Init(void);

uint8_t Motor_Get_Enable_State(uint8_t id);

void Motor_Check_Start(void);
int Motor_Check_Result(uint8_t Encoder_id);

void Motor_Close_Brake(void);

#endif
