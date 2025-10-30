/**
 * @file       hiwonder_bus_servo_driver.c
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

/* Includes ----------------------------------------------------------- */
#include "bus_servo_driver.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
uint8_t  Rx_data[15] = {0};
uint16_t rxLen       = 0;

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
uint8_t check_sum(uint8_t buf[])
{
  uint8_t  i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++)
  {
    temp += buf[i];
  }
  temp = ~temp;
  i    = (uint8_t)temp;
  return i;
}

enum_set_command_status_t SerialServoSetID(UART_HandleTypeDef *huart, uint8_t oldID, uint8_t newID)
{
  uint8_t buf[7];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = oldID;
  buf[3]          = 4;
  buf[4]          = SERVO_ID_WRITE;
  buf[5]          = newID;
  buf[6]          = check_sum(buf);
  if (HAL_UART_Transmit(huart, buf, sizeof(buf), 1500) != HAL_OK)
    return SET_FAILED;
  else
    return SET_OK;
}

enum_set_command_status_t
SerialServoMove(UART_HandleTypeDef *huart, uint8_t id, int16_t position, uint16_t time)
{
  uint8_t buf[10];
  if (position < 0)
    position = 0;
  if (position > 1500)
    position = 1500;
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 7;
  buf[4]          = SERVO_MOVE_TIME_WRITE;
  buf[5]          = GET_LOW_BYTE(position);
  buf[6]          = GET_HIGH_BYTE(position);
  buf[7]          = GET_LOW_BYTE(time);
  buf[8]          = GET_HIGH_BYTE(time);
  buf[9]          = check_sum(buf);
  if (HAL_UART_Transmit(huart, buf, sizeof(buf), 1500) != HAL_OK)
    return SET_FAILED;
  else
    return SET_OK;
}

enum_result_status_t SerialServoReadID(UART_HandleTypeDef *huart, struct_result_t *result)
{
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = ID_ALL;
  buf[3]          = 3;
  buf[4]          = SERVO_ID_READ;
  buf[5]          = check_sum(buf);
  HAL_UART_Transmit(huart, buf, sizeof(buf), 1500);

  HAL_UARTEx_ReceiveToIdle(huart, Rx_data, sizeof(Rx_data), &rxLen, 1500);
  if ((Rx_data[0] == SERVO_FRAME_HEADER) && (Rx_data[1] == SERVO_FRAME_HEADER))
  {
    if (Rx_data[3] == 0x04)
    {
      if (Rx_data[4] == 0x0E)
      {
        if (check_sum(Rx_data) == Rx_data[6])
        {
          result->returned_result = (int16_t)BYTE_TO_HW(0x00, Rx_data[5]);
          return READ_OK;
        }
        else
          return CHECKSUM_ERROR;
      }
      else
        return COMMAND_ERROR;
    }
    else
      return LENGTH_ERROR;
  }
  else
    return HEADER_ERROR;
}

enum_result_status_t SerialServoReadPosition(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result)
{
  uint8_t buf[6];
  uint8_t rx_buf[8];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_POS_READ;
  buf[5]          = check_sum(buf);

  if (HAL_UART_Transmit(huart, buf, sizeof(buf), 1500) != HAL_OK)
    return TX_ERROR;

  if (HAL_UART_Receive(huart, rx_buf, 8, 1500) != HAL_OK)
    return RX_ERROR;

  if (rx_buf[0] != SERVO_FRAME_HEADER || rx_buf[1] != SERVO_FRAME_HEADER)
    return HEADER_ERROR;

  if (rx_buf[2] != id)
    return ID_ERROR;

  if (rx_buf[3] != 0x05)
    return LENGTH_ERROR;

  if (rx_buf[4] != SERVO_POS_READ)
    return COMMAND_ERROR;

  if (check_sum(rx_buf) != rx_buf[7])
    return CHECKSUM_ERROR;

  result->returned_result = (int16_t)BYTE_TO_HW(rx_buf[6], rx_buf[5]);

  return READ_OK;
}

enum_result_status_t SerialServoReadDev(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result)
{
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_ANGLE_OFFSET_READ;
  buf[5]          = check_sum(buf);
  HAL_UART_Transmit(huart, buf, sizeof(buf), 1500);

  HAL_UARTEx_ReceiveToIdle(huart, Rx_data, sizeof(Rx_data), &rxLen, 1500);
  if ((Rx_data[0] == SERVO_FRAME_HEADER) && (Rx_data[1] == SERVO_FRAME_HEADER))
  {
    if (Rx_data[2] == id)
    {
      if (Rx_data[3] == 0x04)
      {
        if (Rx_data[4] == SERVO_ANGLE_OFFSET_READ)
        {
          if (check_sum(Rx_data) == Rx_data[6])
          {
            result->returned_result = (int16_t)BYTE_TO_HW(0x00, Rx_data[5]);
            return READ_OK;
          }
          else
            return CHECKSUM_ERROR;
        }
        else
          return COMMAND_ERROR;
      }
      else
        return LENGTH_ERROR;
    }
    else
      return ID_ERROR;
  }
  else
    return HEADER_ERROR;
}

enum_result_status_t SerialServoReadAngleRange(UART_HandleTypeDef *huart, uint8_t id, struct_range_t *angle)
{
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_ANGLE_LIMIT_READ;
  buf[5]          = check_sum(buf);
  HAL_UART_Transmit(huart, buf, sizeof(buf), 1500);

  HAL_UARTEx_ReceiveToIdle(huart, Rx_data, sizeof(Rx_data), &rxLen, 1500);
  if ((Rx_data[0] == SERVO_FRAME_HEADER) && (Rx_data[1] == SERVO_FRAME_HEADER))
  {
    if (Rx_data[2] == id)
    {
      if (Rx_data[3] == 0x07)
      {
        if (Rx_data[4] == SERVO_ANGLE_LIMIT_READ)
        {
          if (check_sum(Rx_data) == Rx_data[9])
          {
            angle->low_limit  = (int16_t)BYTE_TO_HW(Rx_data[6], Rx_data[5]);
            angle->high_limit = (int16_t)BYTE_TO_HW(Rx_data[8], Rx_data[7]);
            return READ_OK;
          }
          else
            return CHECKSUM_ERROR;
        }
        else
          return COMMAND_ERROR;
      }
      else
        return LENGTH_ERROR;
    }
    else
      return ID_ERROR;
  }
  else
    return HEADER_ERROR;
}

enum_result_status_t SerialServoReadVin(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result)
{
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_VIN_READ;
  buf[5]          = check_sum(buf);
  HAL_UART_Transmit(huart, buf, sizeof(buf), 1500);

  HAL_UARTEx_ReceiveToIdle(huart, Rx_data, sizeof(Rx_data), &rxLen, 1500);
  if ((Rx_data[0] == SERVO_FRAME_HEADER) && (Rx_data[1] == SERVO_FRAME_HEADER))
  {
    if (Rx_data[2] == id)
    {
      if (Rx_data[3] == 0x05)
      {
        if (Rx_data[4] == SERVO_VIN_READ)
        {
          if (check_sum(Rx_data) == Rx_data[7])
          {
            result->returned_result = (int16_t)BYTE_TO_HW(Rx_data[6], Rx_data[5]);
            return READ_OK;
          }
          else
            return CHECKSUM_ERROR;
        }
        else
          return COMMAND_ERROR;
      }
      else
        return LENGTH_ERROR;
    }
    else
      return ID_ERROR;
  }
  else
    return HEADER_ERROR;
}

enum_result_status_t SerialServoReadVinLimit(UART_HandleTypeDef *huart, uint8_t id, struct_range_t *range)
{
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_VIN_LIMIT_READ;
  buf[5]          = check_sum(buf);
  HAL_UART_Transmit(huart, buf, sizeof(buf), 1500);

  HAL_UARTEx_ReceiveToIdle(huart, Rx_data, sizeof(Rx_data), &rxLen, 1500);
  if ((Rx_data[0] == SERVO_FRAME_HEADER) && (Rx_data[1] == SERVO_FRAME_HEADER))
  {
    if (Rx_data[2] == id)
    {
      if (Rx_data[3] == 0x07)
      {
        if (Rx_data[4] == SERVO_VIN_LIMIT_READ)
        {
          if (check_sum(Rx_data) == Rx_data[9])
          {
            range->low_limit  = (int16_t)BYTE_TO_HW(Rx_data[6], Rx_data[5]);
            range->high_limit = (int16_t)BYTE_TO_HW(Rx_data[8], Rx_data[7]);
            return READ_OK;
          }
          else
            return CHECKSUM_ERROR;
        }
        else
          return COMMAND_ERROR;
      }
      else
        return LENGTH_ERROR;
    }
    else
      return ID_ERROR;
  }
  else
    return HEADER_ERROR;
}

enum_result_status_t SerialServoReadTemp(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result)
{
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_TEMP_READ;
  buf[5]          = check_sum(buf);
  HAL_UART_Transmit(huart, buf, sizeof(buf), 1500);

  HAL_UARTEx_ReceiveToIdle(huart, Rx_data, sizeof(Rx_data), &rxLen, 1500);
  if ((Rx_data[0] == SERVO_FRAME_HEADER) && (Rx_data[1] == SERVO_FRAME_HEADER))
  {
    if (Rx_data[2] == id)
    {
      if (Rx_data[3] == 0x04)
      {
        if (Rx_data[4] == SERVO_TEMP_READ)
        {
          if (check_sum(Rx_data) == Rx_data[6])
          {
            result->returned_result = (int16_t)BYTE_TO_HW(0x00, Rx_data[5]);
            return READ_OK;
          }
          else
            return CHECKSUM_ERROR;
        }
        else
          return COMMAND_ERROR;
      }
      else
        return LENGTH_ERROR;
    }
    else
      return ID_ERROR;
  }
  else
    return HEADER_ERROR;
}

enum_result_status_t SerialServoReadTempLimit(UART_HandleTypeDef *huart, uint8_t id, struct_range_t *range)
{
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_TEMP_MAX_LIMIT_READ;
  buf[5]          = check_sum(buf);
  HAL_UART_Transmit(huart, buf, sizeof(buf), 1500);

  HAL_UARTEx_ReceiveToIdle(huart, Rx_data, sizeof(Rx_data), &rxLen, 1500);
  if ((Rx_data[0] == SERVO_FRAME_HEADER) && (Rx_data[1] == SERVO_FRAME_HEADER))
  {
    if (Rx_data[2] == id)
    {
      if (Rx_data[3] == 0x04)
      {
        if (Rx_data[4] == SERVO_TEMP_MAX_LIMIT_READ)
        {
          if (check_sum(Rx_data) == Rx_data[6])
          {
            range->high_limit = (int16_t)BYTE_TO_HW(0x00, Rx_data[5]);
            return READ_OK;
          }
          else
            return CHECKSUM_ERROR;
        }
        else
          return COMMAND_ERROR;
      }
      else
        return LENGTH_ERROR;
    }
    else
      return ID_ERROR;
  }
  else
    return HEADER_ERROR;
}

enum_result_status_t
SerialServoReadLoadOrUnload(UART_HandleTypeDef *huart, uint8_t id, struct_result_t *result)
{
  uint8_t buf[6];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_LOAD_OR_UNLOAD_READ;
  buf[5]          = check_sum(buf);
  HAL_UART_Transmit(huart, buf, sizeof(buf), 1500);

  HAL_UARTEx_ReceiveToIdle(huart, Rx_data, sizeof(Rx_data), &rxLen, 1500);
  if ((Rx_data[0] == SERVO_FRAME_HEADER) && (Rx_data[1] == SERVO_FRAME_HEADER))
  {
    if (Rx_data[2] == id)
    {
      if (Rx_data[3] == 0x04)
      {
        if (Rx_data[4] == SERVO_LOAD_OR_UNLOAD_READ)
        {
          if (check_sum(Rx_data) == Rx_data[6])
          {
            result->returned_result = (int16_t)BYTE_TO_HW(0x00, Rx_data[5]);
            return READ_OK;
          }
          else
            return CHECKSUM_ERROR;
        }
        else
          return COMMAND_ERROR;
      }
      else
        return LENGTH_ERROR;
    }
    else
      return ID_ERROR;
  }
  else
    return HEADER_ERROR;
}

enum_set_command_status_t SerialServoStopMove(UART_HandleTypeDef *huart, uint8_t id)
{
  uint8_t buf[6];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 3;
  buf[4]          = SERVO_MOVE_STOP;
  buf[5]          = check_sum(buf);
  if (HAL_UART_Transmit(huart, buf, sizeof(buf), 1500) != HAL_OK)
    return SET_FAILED;
  else
    return SET_OK;
}

enum_set_command_status_t
SerialServoSetMode(UART_HandleTypeDef *huart, uint8_t id, enum_running_mode_t mode, int16_t speed)
{
  uint8_t buf[10];

  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 7;
  buf[4]          = SERVO_OR_MOTOR_MODE_WRITE;
  buf[5]          = mode;
  buf[6]          = 0;
  buf[7]          = GET_LOW_BYTE((uint16_t)speed);
  buf[8]          = GET_HIGH_BYTE((uint16_t)speed);
  buf[9]          = check_sum(buf);

  if (HAL_UART_Transmit(huart, buf, sizeof(buf), 1500) != HAL_OK)
    return SET_FAILED;
  else
    return SET_OK;
}

enum_set_command_status_t SerialServoLoad(UART_HandleTypeDef *huart, uint8_t id)
{
  uint8_t buf[7];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 4;
  buf[4]          = SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5]          = 1;
  buf[6]          = check_sum(buf);

  if (HAL_UART_Transmit(huart, buf, sizeof(buf), 1500) != HAL_OK)
    return SET_FAILED;
  else
    return SET_OK;
}

enum_set_command_status_t SerialServoUnload(UART_HandleTypeDef *huart, uint8_t id)
{
  uint8_t buf[7];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2]          = id;
  buf[3]          = 4;
  buf[4]          = SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5]          = 0;
  buf[6]          = check_sum(buf);

  if (HAL_UART_Transmit(huart, buf, sizeof(buf), 1500) != HAL_OK)
    return SET_FAILED;
  else
    return SET_OK;
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
