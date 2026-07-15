#ifndef __CONFIG_H__
#define __CONFIG_H__

// Release build, must be confirmed as 1 before release
#define APP_RELEASE                  1


/* ===========================================================================
 * BATTERY PACK -- THE ONLY LINE TO CHANGE WHEN YOU SWAP THE PACK.
 *
 *   BATTERY_CELLS 2  ->  2S pack,  8.4 V full   (the pack currently in use)
 *   BATTERY_CELLS 3  ->  3S pack, 12.6 V full   (the stock Yahboom pack)
 *
 * Change the number, rebuild, reflash. Three things follow from it: the battery
 * alarm thresholds (app_bat.c), the RGB fuel-gauge scale (app_rgb.c), and the
 * dead-zone fallback used before the first ADC conversion (bsp_motor.h).
 *
 * Before this existed, the thresholds were keyed on the CHASSIS type, which is a
 * different thing entirely: a mecanum chassis was assumed to carry a 12.6 V pack.
 * Running a 2S pack on a mecanum chassis therefore sat permanently below the low
 * threshold, and Yahboom papered over that with a "blind window" that disabled the
 * alarm between 6.5 V and 8.5 V. That window had two teeth (see app_bat.c).
 *
 * IF YOU FORGET TO REFLASH when swapping packs, the firmware does not silently
 * mis-protect the pack -- it refuses to run, ~2.1 s after boot:
 *   - 3S pack on a BATTERY_CELLS 2 build -> reads ~124 > 88 -> BATTERY_OVER_VOLTAGE
 *   - 2S pack on a BATTERY_CELLS 3 build -> reads  ~83 < 96 -> BATTERY_LOW
 * Both latch until a power cycle. Refusing to move is the right answer to a
 * pack/firmware mismatch; draining someone's pack to 2.2 V/cell is not.
 *
 * All values are in TENTHS OF A VOLT -- the unit of Voltage_Z10, i.e. the value AS
 * READ back by Adc_Get_Battery_Volotage() (bsp_adc.c), not the true pack voltage.
 * On this board a 8.4 V pack reads 83. That offset is the divider's resistor
 * tolerance and VDDA, not a firmware bias: the 4.03f factor is (10+3.3)/3.3 to
 * within 0.01 %. The ADC chain has never been calibrated against a meter, so keep
 * a few tenths of margin on every threshold below.
 * ======================================================================== */
#define BATTERY_CELLS                3

/* Both profiles are defined unconditionally so the preprocessor can check that they
 * stay separable -- see the #error below. Only the selected one is used. */
#define BAT_2S_NOMINAL_Z10           84    /* 8.4 V, full pack                        */
#define BAT_2S_LOW_Z10               65    /* 6.5 V = 3.25 V/cell. Yahboom's own 2S   */
                                           /* floor (the CAR_SUNRISE profile), and    */
                                           /* the exact point that already trips today */
#define BAT_2S_OVER_Z10              88    /* 8.8 V. Unreachable by a full 2S (83),   */
                                           /* and any 3S is far above it. Yahboom used */
                                           /* 85 here: only 2 counts over a full pack, */
                                           /* too tight for an uncalibrated divider.   */

/* The BATTERY_CELLS 3 slot is the MecaMate final robot's ACTUAL pack, not a generic 3S:
 * a 12 V 3000 mAh Li-ion pack (10C / 20 A fuse, XT30). Its real operating envelope, from
 * the pack's own spec sheet + the owner:
 *   - full charge: reaches ~13.0 V at times (peaks).
 *   - minimum NO-LOAD voltage: 9.0 V. Resting below that permanently damages the cells.
 *   - BUT "periodic dips below 9.0 V UNDER LOAD are expected and OK" (spec, verbatim).
 * That last line is why the low alarm sits BELOW 9.0 V, not at it: under load the terminal
 * voltage sags (up to ~20 A through the fuse), and a 9.0-9.6 V alarm would false-lock the
 * robot on every hard pull. The 2.1 s continuous-sample filter already rides out brief dips;
 * the threshold below is a backstop against genuine depletion, not a load-sag detector. */
#define BAT_3S_NOMINAL_Z10           120   /* 12.0 V nominal (dead-zone fallback only)  */
#define BAT_3S_LOW_Z10               85    /* 8.5 V backstop. Below the 9.0 V no-load    */
                                           /* floor to tolerate load sag; the 2.1 s      */
                                           /* filter handles periodic dips. TUNABLE: if  */
                                           /* heavy sustained load false-locks, drop to  */
                                           /* 80 (8.0 V); the host should soft-warn near  */
                                           /* 9.5-10 V resting before this ever fires.    */
#define BAT_3S_OVER_Z10              138    /* 13.8 V. ~0.8 V above the pack's 13 V peaks, */
                                           /* robust to ADC variance; still catches a     */
                                           /* real overcharge/charger fault (>4.6 V/cell).*/

/* Per-profile sanity only. The old cross-profile "2S ceiling < 3S floor" check is gone:
 * it protected a use case that no longer exists (one board hot-swapped between an 8.4 V and
 * a 12.6 V pack). Each robot now has ONE dedicated pack, selected by BATTERY_CELLS here. */
#if (BAT_2S_LOW_Z10 >= BAT_2S_OVER_Z10) || (BAT_3S_LOW_Z10 >= BAT_3S_OVER_Z10)
  #error "Each profile needs a non-empty normal band: low < over."
#endif

#if   (BATTERY_CELLS == 2)
  #define BATTERY_NOMINAL_Z10        BAT_2S_NOMINAL_Z10
  #define BAT_LOW_VOLTAGE_Z10        BAT_2S_LOW_Z10
  #define BAT_OVER_VOLTAGE_Z10       BAT_2S_OVER_Z10
#elif (BATTERY_CELLS == 3)
  #define BATTERY_NOMINAL_Z10        BAT_3S_NOMINAL_Z10
  #define BAT_LOW_VOLTAGE_Z10        BAT_3S_LOW_Z10
  #define BAT_OVER_VOLTAGE_Z10       BAT_3S_OVER_Z10
#else
  #error "BATTERY_CELLS must be 2 (8.4 V pack) or 3 (12.6 V pack)."
#endif


#if APP_RELEASE

// Interval time for automatically sending data, unit is 1 millisecond, minimum value is 10
#define AUTO_SEND_TIMEOUT            40


/* Car function switches */
#define ENABLE_LOW_BATTERY_ALARM     1
#define ENABLE_CLEAR_RXBUF           1
#define ENABLE_CHECKSUM              1
#define ENABLE_IWDG                  0
#define ENABLE_KEY_RELEASE           1
#define ENABLE_OLED                  1
#define ENABLE_CAR_SUNRISE_ONLY      0


#define ENABLE_ICM20948              1
#define ENABLE_MPU9250               1
#define ENABLE_IMU_ERROR_PASS        0
#define ENABLE_YAW_ADJUST            1
#define ENABLE_ROLL_PITCH            0
#define ENABLE_REAL_WHEEL            0


/* Save function settings to Flash */
#define ENABLE_RESET_FLASH           0
#define ENABLE_FLASH                 1
#define ENABLE_AUTO_REPORT           1
#define ENABLE_IMU_AUTOREPORT        0    /* IMU frames (0x0B raw + 0x0C attitude) in the   */
                                          /* auto-report. OFF: the ROS driver's IMU comes    */
                                          /* from the muto, not the Rosmaster, so these are   */
                                          /* dead traffic. Re-enable (1) if a host ever needs */
                                          /* the Rosmaster IMU. See vTask_Auto_Report.        */


/* Debug printing functions */
#define ENABLE_DEBUG_ICM_ATT         0
#define ENABLE_DEBUG_MPU_ATT         0
#define ENABLE_DEBUG_SBUS            0
#define ENABLE_DEBUG_YAW             0
#define ENABEL_DEBUG_ENCODER         0


#else /* Debug settings are modified here, not compiled in the release build */

// Interval time for automatically sending data, unit is 1 millisecond, minimum value is 10
#define AUTO_SEND_TIMEOUT            40


/* Car function switches */
#define ENABLE_LOW_BATTERY_ALARM     1
#define ENABLE_CLEAR_RXBUF           1
#define ENABLE_CHECKSUM              1
#define ENABLE_IWDG                  0
#define ENABLE_KEY_RELEASE           0
#define ENABLE_OLED                  1
#define ENABLE_CAR_SUNRISE_ONLY      0


#define ENABLE_ICM20948              0
#define ENABLE_MPU9250               0
#define ENABLE_IMU_ERROR_PASS        0
#define ENABLE_YAW_ADJUST            1
#define ENABLE_ROLL_PITCH            1
#define ENABLE_REAL_WHEEL            0


/* Save function settings to Flash */
#define ENABLE_RESET_FLASH           0
#define ENABLE_FLASH                 0
#define ENABLE_AUTO_REPORT           0
#define ENABLE_IMU_AUTOREPORT        0    /* see the release branch above */


/* Debug printing functions */
#define ENABLE_DEBUG_ICM_ATT         0
#define ENABLE_DEBUG_MPU_ATT         0
#define ENABLE_DEBUG_SBUS            0
#define ENABLE_DEBUG_YAW             0
#define ENABEL_DEBUG_ENCODER         0



#endif /* APP_RELEASE */

#endif /* __CONFIG_H__ */
