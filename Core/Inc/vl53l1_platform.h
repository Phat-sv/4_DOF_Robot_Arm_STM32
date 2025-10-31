/**
 * @file       vl53l1_platform.h
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2025-05-17
 * @author     Phat Nguyen Tan
 * @author
 *
 * @brief      <>
 *
 * @note
 * @example
 *
 * @example
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

/* Includes ----------------------------------------------------------- */
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdint.h>

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

/* Public defines ----------------------------------------------------- */
#define TOF_XSHUT_Pin       GPIO_PIN_4
#define TOF_XSHUT_GPIO_Port GPIOB
#define TOF_GPIO_Pin        GPIO_PIN_5
#define TOF_GPIO_GPIO_Port  GPIOB
#define TOF_SCL_Pin         GPIO_PIN_6
#define TOF_SCL_GPIO_Port   GPIOB
#define TOF_SDA_Pin         GPIO_PIN_7
#define TOF_SDA_GPIO_Port   GPIOB

#define VL53L1__ADDR 0x52    // the I2C chip address
#define VL53L1__USING_XSHUT  // uncomment this line if XSHUT pin of VL35L1X is connected
#define VL53L1__USING_GPIO   // uncomment this line if GPIO pin of VL35L1X is connected

// 1 = short, 2 = long
#define VL53L1__DISTANCE_MODE (2)

// in ms, possible values: [15 (only if DISTANCE MODE is 1), 20, 33, 50, 100, 200, 500]
// that's the time for a single reading
#define VL53L1__TIMING_BUDGET (200)

// in ms, added to TB to get IM setting
#define VL53L1__TB_IM_DELTA (10)

// in ms, it must be > = VL53L1_TIMING_BUDGET
// that's the interval (including reading time) between two readings in continuous mode
#define VL53L1__INTERMEASUREMENT (VL53L1__TIMING_BUDGET + VL53L1__TB_IM_DELTA)

#define IT_POLARITY_LOW  0
#define IT_POLARITY_HIGH 1
#define ROI_X_SIZE       16
#define ROI_Y_SIZE       16
#define ROI_CENTER       0

// 0 = below, 1 = beyond, 2 = out of window, 3 = inside window
#define VL53L1__WINDOW_MODE (0)
// (mm) lower window limit. For WINDOW_MODE = 0, 2, 3
#define VL53L1__LOWER_THRESHOLD (2000)
// (mm) upper window limit. For WINDOW_MODE = 1, 2, 3
#define VL53L1__UPPER_THRESHOLD (4000)

#define VL53L1__CALIB_OFFSET (33)
#define VL53L1__CALIB_XTALK  (0)

/* 0 = no errors,
 * 1 = warning: high standard deviation
 * 2 = warning: low return signal level
 * 4 - 7 errors
 */
#define VL53L1__RANGE_STATUS_THRESH (7)  // acceptable values: 0, 1, 2, 4, 7

//=============================================== DEVICE PARAMETERS
//==============================================//
/* you shouldn't need to change anything here after */
// Reference Registers for VL53L1X
// allowing to validate I2C connection on boot
#define VL53L1__MODELID_INDEX    0x010F
#define VL53L1__MODELID_VALUE    0xEA
#define VL53L1__MODULETYPE_INDEX 0x0110
#define VL53L1__MODULETYPE_VALUE 0xCC
#define VL53L1__MASKREV_INDEX    0x0111
#define VL53L1__MASKREV_VALUE    0x10

#define I2C_COMM_TIMEOUT 100    // ms timeout for the I2C communication
#define VL53L1__IO_ERROR (-13)  // error code returned by I/O interface functions

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief <>
 */
typedef struct
{
  uint32_t dummy;
} VL53L1_Dev_t;

/**
 * @brief <>
 */
typedef VL53L1_Dev_t *VL53L1_DEV;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
/** @brief choose i2c channel for VL53L1X
 * To be implemented by the developer
 */
void vl53l1x_assignI2C(I2C_HandleTypeDef *hi2c_device);

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);

/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);

/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);

/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);

/** @brief VL53L1_WrDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);

/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata);

/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata);

/** @brief VL53L1_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata);

/** @brief VL53L1_WaitMs() definition.\n
 * To be implemented by the developer
 */
void VL53L1_WaitMs(uint16_t dev, uint16_t wait_ms);

#endif  // _VL53L1_PLATFORM_H_
