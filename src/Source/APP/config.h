#ifndef __CONFIG_H__
#define __CONFIG_H__

// Release build, must be confirmed as 1 before release
#define APP_RELEASE                  1

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


/* Debug printing functions */
#define ENABLE_DEBUG_ICM_ATT         0
#define ENABLE_DEBUG_MPU_ATT         0
#define ENABLE_DEBUG_SBUS            0
#define ENABLE_DEBUG_YAW             0
#define ENABEL_DEBUG_ENCODER         0



#endif /* APP_RELEASE */

#endif /* __CONFIG_H__ */
