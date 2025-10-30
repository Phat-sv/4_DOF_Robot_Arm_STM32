/**
 * @file       hiwonder_bus_servo_driver.h
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2025-05-12
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
#ifndef __HIWONDER_BUS_SERVO_DRIVER_H
#define __HIWONDER_BUS_SERVO_DRIVER_H

/* Includes ----------------------------------------------------------- */
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Public defines ----------------------------------------------------- */
/*
When the ID is 254, a broadcast will be sent to all servos, which can be used to read information from servos
with unknown IDs
*/
#define ID_ALL             254
#define SERVO_FRAME_HEADER 0x55

#define SERVO_MOVE_TIME_WRITE      1
#define SERVO_MOVE_TIME_READ       2
#define SERVO_MOVE_TIME_WAIT_WRITE 7
#define SERVO_MOVE_TIME_WAIT_READ  8
#define SERVO_MOVE_START           11
#define SERVO_MOVE_STOP            12
#define SERVO_ID_WRITE             13
#define SERVO_ID_READ              14
#define SERVO_ANGLE_OFFSET_ADJUST  17
#define SERVO_ANGLE_OFFSET_WRITE   18
#define SERVO_ANGLE_OFFSET_READ    19
#define SERVO_ANGLE_LIMIT_WRITE    20
#define SERVO_ANGLE_LIMIT_READ     21
#define SERVO_VIN_LIMIT_WRITE      22
#define SERVO_VIN_LIMIT_READ       23
#define SERVO_TEMP_MAX_LIMIT_WRITE 24
#define SERVO_TEMP_MAX_LIMIT_READ  25
#define SERVO_TEMP_READ            26
#define SERVO_VIN_READ             27
#define SERVO_POS_READ             28
#define SERVO_OR_MOTOR_MODE_WRITE  29
#define SERVO_OR_MOTOR_MODE_READ   30
#define SERVO_LOAD_OR_UNLOAD_WRITE 31
#define SERVO_LOAD_OR_UNLOAD_READ  32
#define SERVO_LED_CTRL_WRITE       33
#define SERVO_LED_CTRL_READ        34
#define SERVO_LED_ERROR_WRITE      35
#define SERVO_LED_ERROR_READ       36

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief <status returned when read data from servo>
 */
typedef enum
{
  READ_OK      = 1,
  HEADER_ERROR = 2,
  ID_ERROR,
  LENGTH_ERROR,
  COMMAND_ERROR,
  CHECKSUM_ERROR,
  TX_ERROR,
  RX_ERROR
} enum_result_status_t;

/**
 * @brief <status of cammand sent to servo>
 */
typedef enum
{
  SET_FAILED = 0,
  SET_OK     = 1
} enum_set_command_status_t;

/**
 * @brief <define mode for servo, servo mode for position control or motor mode like a normal mortor>
 */
typedef enum
{
  SERVO_MODE = 0,
  MOTOR_MODE = 1
} enum_running_mode_t;

/**
 * @brief <struct containing low and high limits of different parameters which are read from servo>
 */
typedef struct
{
  int low_limit;
  int high_limit;
  // enum_result_status_t result;
} struct_range_t;

/**
 * @brief <struct containing returned result which are read from servo>
 */
typedef struct
{
  int returned_result;
} struct_result_t;

/* Public macros ------------------------------------------------------ */
/**
 * @brief  <macro function to get the low 8 bits of A>
 *
 * @param[in]     <A>  <data that we want to get its lower 8 bits>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 */
#define GET_LOW_BYTE(A) (uint8_t)((A))

/**
 * @brief  <macro function to get the high 8 bits of A>
 *
 * @param[in]     <A>  <data that we want to get its higher 8 bits>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 */
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)

/**
 * @brief  <macro function to merge A and B as a 16-bit integer, with A as the high 8 bits and B as the low 8
 * bits>
 *
 * @param[in]     <A>  <A is the high 8 bits>
 * @param[in]     <B>  <B is the low 8 bits>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 */
static inline uint16_t BYTE_TO_HW(uint8_t high, uint8_t low)
{
  return ((uint16_t)high << 8) | low;
}

/* Public variables --------------------------------------------------- */
extern UART_HandleTypeDef huart1; /**< uart channel is used in microcontroller */

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  <calculate check sum>
 *
 * @param[in]     <buf[]>  <buffer that we want to calculate checksum>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return  value of checksum calculated
 */
uint8_t check_sum(uint8_t buf[]);

/**
 * @brief  <write servo ID>
 *
 * @param[in]     <param_name>  <param_despcription>
 * @param[out]    <param_name>  <param_despcription>
 * @param[inout]  <param_name>  <param_despcription>
 *
 * @attention  <just only 1 servo per time ortherwise all servos will have the same ID>
 *
 * @return
 *  - 0: SET_FAILED
 *  - 1: SET_OK
 */
enum_set_command_status_t SerialServoSetID(UART_HandleTypeDef *huart, uint8_t oldID, uint8_t newID);

/**
 * @brief  <When the command is sent to servo, the servo will be rotated from current
 angle to parameter angle at uniform speed within parameter time. After the
 command reaches servo, servo will rotate immediately>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <ID of servo>
 * @param[in]     <position>  <desired position>
 * @param[in]     <time>  <time for movememnt>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 0: SET_FAILED
 *  - 1: SET_OK
 */
enum_set_command_status_t
SerialServoMove(UART_HandleTypeDef *huart, uint8_t id, int16_t position, uint16_t time);

/**
 * @brief  <inquiry the servo ID number via broadcast ID without knowing the ID number of the servo>
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <*result>  <struct pointer bring returned value>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <can only access a servo, or it will return data caused bus conflict>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t SerialServoReadID(UART_HandleTypeDef *huart, struct_result_t *result);

/**
 * @brief  <read servo position>
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <*result>  <struct pointer bring returned value>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t SerialServoReadPosition(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result);

/**
 * @brief  <read deviation>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <*result>  <struct pointer bring returned value>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t SerialServoReadDev(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result);

/**
 * @brief  <read rotation range>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <*angle>  <struct pointer bring returned value (high limit and low limit)>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t SerialServoReadAngleRange(UART_HandleTypeDef *huart, uint8_t id, struct_range_t *angle);

/**
 * @brief  <read voltage>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <*result>  <struct pointer bring returned value>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t SerialServoReadVin(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result);

/**
 * @brief  <read voltage range>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <*range>  <struct pointer bring returned value (high limit and low limit)>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t SerialServoReadVinLimit(UART_HandleTypeDef *huart, uint8_t id, struct_range_t *range);

/**
 * @brief  <read temperature>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <*result>  <struct pointer bring returned value>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t SerialServoReadTemp(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result);

/**
 * @brief  <read temperature alarm threashold>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <*range>  <struct pointer bring returned value (high limit)>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t SerialServoReadTempLimit(UART_HandleTypeDef *huart, uint8_t id, struct_range_t *range);

/**
 * @brief  <read servo status>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <*result>  <struct pointer bring returned value>
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 1: READ_OK
 *  - 2000: HEADER_ERROR
 *  - 2001: ID_ERROR
 *  - 2002: LENGTH_ERROR
 *  - 2003: COMMAND_ERROR
 *  - 2004: CHECKSUM_ERROR
 */
enum_result_status_t
SerialServoReadLoadOrUnload(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result);

/**
 * @brief  <stop rotation>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 0: SET_FAILED
 *  - 1: SET_OK
 */
enum_set_command_status_t SerialServoStopMove(UART_HandleTypeDef *huart, uint8_t id);

/**
 * @brief  <set servo mode>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[in]     <mode>  <choose either SERVO_MODE or MOTOR_MODE>
 * @param[in]     <speed>  <speed of servo or motor, range from -1000 to 1000>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 0: SET_FAILED
 *  - 1: SET_OK
 */
enum_set_command_status_t
SerialServoSetMode(UART_HandleTypeDef *huart, uint8_t id, enum_running_mode_t mode, int16_t speed);

/**
 * @brief  <servo power-on>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 0: SET_FAILED
 *  - 1: SET_OK
 */
enum_set_command_status_t SerialServoLoad(UART_HandleTypeDef *huart, uint8_t id);

/**
 * @brief  <servo poweroff>
 *
 * @param[in]     <*huart>  <which channel is used>
 * @param[in]     <id>  <id of servo that we want to read from>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return
 *  - 0: SET_FAILED
 *  - 1: SET_OK
 */
enum_set_command_status_t SerialServoUnload(UART_HandleTypeDef *huart, uint8_t id);

#endif  // __HIWONDER_BUS_SERVO_DRIVER_H

/* End of file -------------------------------------------------------- */
