
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
#define MOTOR_MAX_PULSE     (3600)
#define MOTOR_FREQ_DIVIDE   (0)
/* The old scalar MOTOR_IGNORE_PULSE fallback is gone: Motor_Get_Ignore_Pulse() now derives
 * that window's offset per-axis from BATTERY_NOMINAL_Z10 directly, so a single shared
 * constant would only have been a way for the four axes to disagree with themselves. */


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
 * MOTOR_DEAD_ZONE_MV_Mx is each axis's breakaway voltage. Motor_Get_Ignore_Pulse()
 * turns it back into a pulse count for whatever the battery currently reads, so
 * that 1 % of command sits right at the edge of motion and 100 % is full scale,
 * WHATEVER the pack. That also makes the host-side kS feedforward stable instead
 * of drifting as the battery discharges.
 *
 * NOTE: measured with the wheels free. Under the robot's own weight the breakaway
 * is higher -- that extra torque is the PID's job (or the host's), not this
 * offset's, which only has to get an unloaded motor moving.
 * ------------------------------------------------------------------------- */
/* THE FOUR MOTORS ARE IDENTICAL to within measurement noise. A 5-run x 2-direction
 * campaign (17/07/2026, wheels-free, per-axis, vitesse-soutenue criterion) gave the
 * three healthy motors 6.22 / 6.15 / 6.06 V and the replaced M4 6.21 V -- mean 6.16 V.
 * The between-axis spread (0.08 V over the healthy three) is SMALLER than one axis's
 * own run-to-run noise (M2 alone: sigma 0.18 V), so the per-axis dispersion is noise,
 * not signal, and a single value is the honest choice. Kept as four named constants
 * only so a REAL difference could be encoded later if one is ever proven across runs.
 *
 * HISTORY -- do not repeat. The first cut used [6140,5630,5970,6140], read as "not
 * four identical motors, 0.51 V spread". That was ONE unlucky run of a noisy motor
 * (M2's 5.63) plus a FAILING motor (old M4's 6.82, 6/10 reliability, -0.28 V AV/AR
 * asymmetry -- a hardware fault, replaced 17/07, not a real breakaway). The stock
 * scalar 6150 it replaced was already right to 0.1 %. The per-axis table chased noise.
 *
 * WHEELS-FREE IS THE CORRECT CONDITION, and this is not a fallback -- it is the only
 * condition that isolates the quantity this offset exists to cancel. Verified on the
 * ground 17/07/2026: an axis driven alone measures the tyre's traction limit, the
 * skid-steer pivot geometry and the steering play, all at once -- never the motor's
 * dry friction. With the wheels off the ground none of those three terms exists.
 * The robot's weight is a RESISTIVE TORQUE, not Coulomb friction: that is the PID's
 * job, not this offset's. Do not "correct" these values with a load term -- that is
 * the double compensation we rejected on 15/07. */
#define MOTOR_DEAD_ZONE_MV_M1   (6150)
#define MOTOR_DEAD_ZONE_MV_M2   (6150)
#define MOTOR_DEAD_ZONE_MV_M3   (6150)
#define MOTOR_DEAD_ZONE_MV_M4   (6150)

/* No MOTOR_IGNORE_MIN: the smallest reachable offset is 6150*3600/(200*100) = 1107, so a
 * floor of 800 was unreachable dead code. Only the ceiling below is a live guard. */
/* Clamping the offset below full scale is what CREATES the ceiling that kills the card:
 * it caps V_motor at MOTOR_IGNORE_MAX/3600 * Vbat regardless of what the motor needs.
 * The offset and the PID's saturation both come from Motor_Get_Ignore_Pulse(), so
 *     ignore + PID_sat = MOTOR_MAX_PULSE
 * holds by construction at any value -- there is no range to "protect". The stock 2400
 * capped V_motor at 0.667 * Vbat, which is the ceiling we are removing.
 *
 * WHY MOTOR_MAX_PULSE - 1 AND NOT MOTOR_MAX_PULSE. The last count is not a rounding
 * nicety, it is what keeps the SIGN of the command alive. At ignore = MOTOR_MAX_PULSE
 * the PID's limit is exactly 0, so pwm_output clamps to 0 -- and Motor_Ignore_Dead_Zone()
 * only adds the offset to a NON-ZERO pulse (a commanded zero must stay zero), so a
 * full-throttle command collapses to 0 % duty: the exact opposite of the intent.
 * Worse, the clamp bites at a different voltage on each axis because the KS differ
 * (v10 <= 61 for M1/M4, <= 59 for M3, <= 56 for M2): in the [57,61] band M1/M4 would
 * go dead while M2/M3 ran at 100 %, i.e. an UNCOMMANDED YAW on a collapsing pack, with
 * the cliff falling on a single ADC count. Reserving one count keeps limit >= 1, so the
 * output saturates at +/-1, the offset is applied, and 1 + 3599 = 3600 -- the motor
 * really does get 100 % duty in the COMMANDED direction, on every axis. That is the
 * physically honest answer to "the motor needs more volts than the pack has", and the
 * invariant still holds exactly: 3599 + 1 = 3600. */
#define MOTOR_IGNORE_MAX    (MOTOR_MAX_PULSE - 1) /* no ceiling, but keep the sign alive */

int16_t Motor_Get_Ignore_Pulse(uint8_t id);


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
