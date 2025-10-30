/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "stm32l4xx.h"

// file for robot arm
#include "bus_servo_driver.h"
#include "kinematic.h"
// file for vl53l1x sensor
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "filters.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// define actions received from rasp
#define START_GRIP     (0x30)
#define SEND_DATA      (0x33)
#define NONE           (0x34)
#define RELEASE_OBJECT (0x36)
#define IN_PROGRESS    (0x38)

// define ID data for uart2 (stm <--> rasp)
#define ID_DISTANCE (uint8_t)(0xB1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_rx;
DMA_HandleTypeDef  hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void compute_target_coordinates(float  t1,
                                float  t2,
                                float  t3,
                                float  t4,
                                float  distance,
                                float *x,
                                float *y,
                                float *z)
{
  t1 = DEG_TO_RAD(t1);
  t2 = DEG_TO_RAD(t2);
  t3 = DEG_TO_RAD(t3);
  t4 = DEG_TO_RAD(t4);

  float temp_x = cosf(t1) * (120 * cosf(t2 + t3 + t4) + 190 * cosf(t2 + t3) + 155 * cosf(t2) + 22) +
                 (distance - 35) * cosf(t1);
  float temp_y = sinf(t1) * (120 * cosf(t2 + t3 + t4) + 190 * cosf(t2 + t3) + 155 * cosf(t2) + 22) +
                 (distance - 35) * sinf(t1);
  float temp_z = 120.0f * sinf(t2 + t3 + t4) + 190.0f * sinf(t2 + t3) + 155.0f * sinf(t2) + 57.45f;

  if (isnan(temp_x) || isnan(temp_y) || isnan(temp_z))
  {
    *x = 380;
    *y = 0;
    *z = 212.45;
  }
  else
  {
    *x = temp_x;
    *y = temp_y;
    *z = temp_z;
  }
}

void compute_target_coordinates_1(float  t1,
                                  float  t2,
                                  float  t3,
                                  float  t4,
                                  float  distance,
                                  float *x_t,
                                  float *y_t,
                                  float *z_t,
                                  float *x_g,
                                  float *y_g,
                                  float *z_g)
{
  t1 = DEG_TO_RAD(t1);
  t2 = DEG_TO_RAD(t2);
  t3 = DEG_TO_RAD(t3);
  t4 = DEG_TO_RAD(t4);

  float X = cosf(t1) * (120 * cosf(t2 + t3 + t4) + 190 * cosf(t2 + t3) + 155 * cosf(t2) + 22);
  float Y = sinf(t1) * (120 * cosf(t2 + t3 + t4) + 190 * cosf(t2 + t3) + 155 * cosf(t2) + 22);
  float Z = 120 * sinf(t2 + t3 + t4) + 190 * sinf(t2 + t3) + 155 * sinf(t2) + 57.45;
  *x_g    = X;
  *y_g    = Y;
  *z_g    = Z;

  float theta = t1;
  float elev  = t2 + t3 + t4;

  float dx = (distance - 25.0f) * cosf(elev) * cosf(theta);
  float dy = (distance - 25.0f) * cosf(elev) * sinf(theta);
  float dz = (distance - 25.0f) * sinf(elev);

  float temp_x = X + dx;
  float temp_y = Y + dy;
  float temp_z = Z + dz;

  if (isnan(temp_x) || isnan(temp_y) || isnan(temp_z))
  {
    *x_t = 380;
    *y_t = 0;
    *z_t = 212.45;
  }
  else
  {
    *x_t = temp_x;
    *y_t = temp_y;
    *z_t = temp_z;
  }
}

uint8_t to_straight_position(void)
{
  struct_result_t p1, p2, p3, p4;

  if (SerialServoReadPosition(&huart1, 1, &p1) != READ_OK)
    return 1;
  if (SerialServoReadPosition(&huart1, 2, &p2) != READ_OK)
    return 2;
  if (SerialServoReadPosition(&huart1, 3, &p3) != READ_OK)
    return 3;
  if (SerialServoReadPosition(&huart1, 4, &p4) != READ_OK)
    return 4;

  float delta1 = fabsf((float)(480 - p1.returned_result));
  float delta2 = fabsf((float)(1000 - p2.returned_result));
  float delta3 = fabsf((float)(0 - p3.returned_result));
  float delta4 = fabsf((float)(500 - p4.returned_result));

  float    max_delta = fmaxf(fmaxf(delta1, delta2), fmaxf(delta3, delta4));
  uint16_t move_time = (uint16_t)(max_delta / 0.18f);
  if (move_time < 1800)
    move_time = 1800;

  if (SerialServoMove(&huart1, 2, 1000, move_time) != SET_OK)
    return 6;
  HAL_Delay(5);
  if (SerialServoMove(&huart1, 3, 0, move_time) != SET_OK)
    return 7;
  HAL_Delay(5);
  if (SerialServoMove(&huart1, 4, 500, move_time) != SET_OK)
    return 8;
  HAL_Delay(5);
  if (SerialServoMove(&huart1, 1, 480, move_time) != SET_OK)
    return 5;
  HAL_Delay(5);

  return 9;
}

uint8_t to_ready_position_1(void)
{
  float           required_moving_time = 0;
  uint16_t        moving_time          = 0;
  struct_result_t position1, position2, position3, position4;

  if (SerialServoReadPosition(&huart1, 3, &position3) != READ_OK)
    return 0;
  required_moving_time = (620 - position3.returned_result) / 0.1;
  moving_time          = convert_to_absolute_int(required_moving_time);
  if (SerialServoMove(&huart1, 3, 620, moving_time) != SET_OK)
    return 2;
  HAL_Delay(moving_time);

  if (SerialServoReadPosition(&huart1, 2, &position2) != READ_OK)
    return 3;
  required_moving_time = (520 - position2.returned_result) / 0.1;
  moving_time          = convert_to_absolute_int(required_moving_time);
  if (SerialServoMove(&huart1, 2, 520, moving_time) != SET_OK)
    return 4;
  HAL_Delay(20);

  if (SerialServoReadPosition(&huart1, 4, &position4) != READ_OK)
    return 5;
  required_moving_time = (450 - position4.returned_result) / 0.1;
  moving_time          = convert_to_absolute_int(required_moving_time);
  if (SerialServoMove(&huart1, 4, 450, moving_time) != SET_OK)
    return 6;
  HAL_Delay(moving_time);

  if (SerialServoReadPosition(&huart1, 1, &position1) != READ_OK)
    return 7;
  required_moving_time = (450 - position1.returned_result) / 0.1;
  moving_time          = convert_to_absolute_int(required_moving_time);
  if (SerialServoMove(&huart1, 1, 450, moving_time) != SET_OK)
    return 8;
  HAL_Delay(moving_time);

  return 1;
}

static inline void open_gripper(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
}

static inline void close_gripper(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1580);
}

/**
 * @brief This function process data into a package to transmit
 */
void send_data_with_id(uint8_t id, float value);

/*
return 1: OK
return 0: FAILED
*/
uint8_t to_folded_position(void);

/*
return 1: OK
return 0: FAILED
*/
uint8_t to_ready_position(void);

/*
return 9: OK
return 1 --> 8: FAILED
*/
uint8_t to_home_position(void);

/*
return 9: OK
return 1 --> 8: FAILED
*/
uint8_t to_moving_position(void);

/*
 soft reset system
*/
void soft_reset(void);

/*
 read all joint variables
 return 0: failed (if any joint cannot be read
 return 1: ok
*/
uint8_t read_joint_variables(float    *t1,
                             float    *t2,
                             float    *t3,
                             float    *t4,
                             uint16_t *p1,
                             uint16_t *p2,
                             uint16_t *p3,
                             uint16_t *p4);

/*
 reset vl53l1x
*/
void reset_tof_sensor(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// FILE __stdout;
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 3000);
  return ch;
}

//========================= variables for robot arm ==========================//
// 4 joint variables and servo positions of robot arm
float    theta_1 = 0;
float    theta_2 = 0;
float    theta_3 = 0;
float    theta_4 = 0;
uint16_t pos1    = 0;
uint16_t pos2    = 0;
uint16_t pos3    = 0;
uint16_t pos4    = 0;

// status of reading joints function
uint8_t read_joints_status = 0;

// coordinates of target
float x_gripper = 0;
float y_gripper = 0;
float z_gripper = 0;
// coordinates of target
float x_target = 0;
float y_target = 0;
float z_target = 0;

//========================= variables for vl53l1x ===========================//
uint8_t  status            = 0;  // common status
uint8_t  range_status      = 0;  // common status
uint16_t measured_distance = 0;
bool     distance_updated  = false;

//======== variables to transmit and receive uart data with raspberry and bus servo ========//
uint8_t          received_data_from_rasp[2];
uint8_t          required_action     = NONE;
bool             request_reset       = false;
volatile uint8_t uart_rasp_done_flag = 0;

//============================= debug variables ===============================//
struct_required_angle_t destination_angle;
uint16_t                retry             = 0;
uint8_t                 read_joint_cnt    = 0;
uint8_t                 read_distance_cnt = 0;
uint8_t                 ready_status      = 20;
struct_range_t          AngleRange;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  // MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  //==================== initiate timer 1 channel 1 to control gripper ======================//
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // test gripper
  open_gripper();
  HAL_Delay(300);
  close_gripper();
  HAL_Delay(300);
  open_gripper();
  HAL_Delay(300);

  //=================== interrupt uart for to receive comamnd from raspberry ================//
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, received_data_from_rasp, sizeof(received_data_from_rasp));
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  //======================= initial actions for sensor and robot arm ========================//
  // VL53L1X init
  vl53l1x_assignI2C(&hi2c1);
  VL53L1__Init();
  VL53L1X_StartRanging(VL53L1__ADDR);

  // return to folded position
  to_folded_position();
  //	HAL_Delay(5000);
  //	SerialServoReadAngleRange(&huart1, 2, &AngleRange);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //============================== CONTROL ROBOT ARM ====================================//
    // if raspberry request srm reset itself
    if (request_reset == true)
      soft_reset();

    // if raspberry asks distance value
    if (required_action == SEND_DATA)
    {
      send_data_with_id(ID_DISTANCE, measured_distance);
      required_action = NONE;
    }

    // if raspberry request to release object
    else if (required_action == RELEASE_OBJECT)
    {
      static uint8_t  release_state = 0;
      static uint32_t release_time  = 0;
      switch (release_state)
      {
      case 0:
        to_home_position();
        release_time  = HAL_GetTick();
        release_state = 1;
        break;

      case 1:
        if (HAL_GetTick() - release_time >= 2300)
        {
          open_gripper();
          release_time  = HAL_GetTick();
          release_state = 2;
        }
        break;

      case 2:
        if (HAL_GetTick() - release_time >= 20)
        {
          uint8_t buffer[2] = {0xCC, 0x04};
          HAL_UART_Transmit_DMA(&huart2, buffer, sizeof(buffer));
          to_folded_position();
          required_action = NONE;
          release_state   = 0;  // reset
        }
        break;
      }
    }

    // if raspberry request to grip object (from folded position)
    else if (required_action == START_GRIP)
    {
      static uint32_t grip_start_time = 0;
      static uint8_t  grip_state      = 0;
      if (grip_state == 0)
      {
        open_gripper();
        ready_status = to_ready_position();
        // ready_status = to_ready_position_1();
        grip_start_time = HAL_GetTick();
        grip_state      = 1;
      }
      if (grip_state == 1)
      {
        if (HAL_GetTick() - grip_start_time >= 4800)
        {
          grip_start_time = HAL_GetTick();
          grip_state      = 2;
        }
      }
      if (grip_state == 2)
      {
        if (HAL_GetTick() - grip_start_time >= 200)
        {
          required_action = IN_PROGRESS;
          grip_state      = 0;
        }
      }
    }

    // robot arm is processing, and will inform result to raspberry
    else if (required_action == IN_PROGRESS)
    {
      static uint8_t  in_progress_state = 0;
      static uint32_t in_progress_time  = 0;

      switch (in_progress_state)
      {
      case 0:
        in_progress_time   = HAL_GetTick();
        retry              = 0;
        read_joint_cnt     = 0;
        read_distance_cnt  = 0;
        read_joints_status = 0;
        in_progress_state  = 1;
        break;
      case 1:
        if (read_joints_status == 0)
        {
          read_joints_status =
            read_joint_variables(&theta_1, &theta_2, &theta_3, &theta_4, &pos1, &pos2, &pos3, &pos4);
          read_joint_cnt++;
        }
        if ((read_joints_status == 1) && distance_updated)
        {
          read_distance_cnt++;
          distance_updated = false;
          if ((measured_distance >= 15) && (measured_distance <= 215))
          {
            compute_target_coordinates(theta_1,
                                       theta_2,
                                       theta_3,
                                       theta_4,
                                       measured_distance,
                                       &x_target,
                                       &y_target,
                                       &z_target);
            // compute_target_coordinates_1(theta_1, theta_2, theta_3, theta_4, measured_distance, &x_target,
            // &y_target, &z_target, &x_gripper, &y_gripper, &z_gripper);
            if ((x_target >= 225) && (x_target <= 460) && (y_target >= -65) && (y_target <= 65) &&
                (z_target >= -130) && (z_target <= 250))
            {
              inverse_kinematic(x_target, y_target, z_target, pos1, pos2, pos3, pos4, &destination_angle);
              // inverse(x_target, y_target, z_target, theta_2, theta_3, theta_4, &destination_angle);
              in_progress_time  = HAL_GetTick();
              in_progress_state = 2;
            }
          }
        }
        if (HAL_GetTick() - in_progress_time > 5500)
        {
          if (read_joint_cnt >= 10)
          {
            uint8_t buffer[2] = {0xCC, 0x02};  // inform to rasp: cannot read joints
            HAL_UART_Transmit_DMA(&huart2, buffer, sizeof(buffer));
          }
          if (read_distance_cnt >= 10)
          {
            uint8_t buffer[2] = {0xCC, 0x03};  // inform to rasp: cannot read distance or it is invalid
            HAL_UART_Transmit_DMA(&huart2, buffer, sizeof(buffer));
          }
          required_action = NONE;
          to_folded_position();
          in_progress_state  = 0;
          read_joints_status = 0;
          read_joint_cnt     = 0;
          read_distance_cnt  = 0;
          // soft_reset();
        }
        break;
      case 2:
        if (HAL_GetTick() - in_progress_time >= 20)
        {
          close_gripper();
          in_progress_time  = HAL_GetTick();
          in_progress_state = 3;
        }
        break;
      case 3:
        if (HAL_GetTick() - in_progress_time >= 250)
        {
          to_moving_position();
          // to_straight_position();

          // inform to rasp that processing done, continue to move
          uint8_t buffer[2] = {0xCC, 0x01};
          HAL_UART_Transmit_DMA(&huart2, buffer, sizeof(buffer));
          // reset all variables
          required_action    = NONE;
          in_progress_state  = 0;
          read_joints_status = 0;
          read_joint_cnt     = 0;
          read_distance_cnt  = 0;
        }
        break;
      }
    }

    // reset indepedent window watchdog timer (timeout = 4.5s)
    // HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 1;
  RCC_OscInitStruct.PLL.PLLN            = 9;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance              = I2C1;
  hi2c1.Init.Timing           = 0x00200410;
  hi2c1.Init.OwnAddress1      = 0;
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2      = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance       = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window    = 3299;
  hiwdg.Init.Reload    = 3299;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef        sMasterConfig        = {0};
  TIM_OC_InitTypeDef             sConfigOC            = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 23;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 20000;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter      = 0;
  sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter     = 0;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance                    = USART1;
  huart1.Init.BaudRate               = 115200;
  huart1.Init.WordLength             = UART_WORDLENGTH_8B;
  huart1.Init.StopBits               = UART_STOPBITS_1;
  huart1.Init.Parity                 = UART_PARITY_NONE;
  huart1.Init.Mode                   = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = 115200;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin  = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 PA7 PA11 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                        GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 */
  GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_XSHUT_Pin */
  GPIO_InitStruct.Pin   = TOF_XSHUT_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_XSHUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_GPIO_Pin */
  GPIO_InitStruct.Pin  = TOF_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOF_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin  = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief This function handles EXTI line[9:5] interrupts. Using this function to read signal from interrupt
 * pin of sensor
 */
void EXTI9_5_IRQHandler(void)
{
  status |= VL53L1X_GetRangeStatus(VL53L1__ADDR, &range_status);
  if (range_status > VL53L1__RANGE_STATUS_THRESH)  // VL53L1__RANGE_STATUS_THRESH
  {
    status |= VL53L1X_ClearInterrupt(
      VL53L1__ADDR);  // non acceptable range status, reset for a new ranging notification
  }
  else
  {
    status |= VL53L1X_GetDistance(VL53L1__ADDR, &measured_distance);
    distance_updated = true;
    status |= VL53L1X_ClearInterrupt(VL53L1__ADDR);  // Everything read. Restart interrupt
  }
  HAL_GPIO_EXTI_IRQHandler(TOF_GPIO_Pin);
}

/**
 * @brief This function handles data from higher board, which reads camera, to decide the next action for
 * robot arm.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    switch (received_data_from_rasp[0])
    {
    case 0xBB:
      if (Size == 2)
      {
        switch (received_data_from_rasp[1])
        {
        case 0x04:
          required_action = SEND_DATA;
          break;
        case 0x05:
          required_action = START_GRIP;
          break;
        case 0x06:
          required_action = RELEASE_OBJECT;
          break;
        }
        break;
      }
    case 0xFF:
      request_reset = true;
      break;
    default:
      break;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, received_data_from_rasp, sizeof(received_data_from_rasp));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, received_data_from_rasp, sizeof(received_data_from_rasp));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
}

/**
 * @brief This function handles USART transmit DMA complete
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // uart communicate with bus servo
  }
  if (huart->Instance == USART2)
  {
    // uart communicate with raspberry
    uart_rasp_done_flag = 1;
  }
}

uint8_t to_folded_position(void)
{
  struct_result_t p1, p2, p3, p4;

  if (SerialServoReadPosition(&huart1, 1, &p1) != READ_OK)
    return 1;
  if (SerialServoReadPosition(&huart1, 2, &p2) != READ_OK)
    return 2;
  if (SerialServoReadPosition(&huart1, 3, &p3) != READ_OK)
    return 3;
  if (SerialServoReadPosition(&huart1, 4, &p4) != READ_OK)
    return 4;

  float time1 = fabsf((float)(855 - p1.returned_result)) / 0.15f;
  float time2 = fabsf((float)(275 - p2.returned_result)) / 0.15f;
  float time3 = fabsf((float)(750 - p3.returned_result)) / 0.15f;
  float time4 = fabsf((float)(500 - p4.returned_result)) / 0.15f;

  if (SerialServoMove(&huart1, 1, 855, (uint16_t)time1) != SET_OK)
    return 5;
  HAL_Delay((uint16_t)time1);
  if (SerialServoMove(&huart1, 2, 275, (uint16_t)time2) != SET_OK)
    return 6;
  HAL_Delay((uint16_t)time2);
  if (SerialServoMove(&huart1, 4, 500, (uint16_t)time4) != SET_OK)
    return 8;
  HAL_Delay((uint16_t)time4);
  if (SerialServoMove(&huart1, 3, 750, (uint16_t)time3) != SET_OK)
    return 7;
  HAL_Delay((uint16_t)time3);

  return 9;
}

uint8_t to_ready_position(void)
{
  struct_result_t p1, p2, p3, p4;

  if (SerialServoReadPosition(&huart1, 1, &p1) != READ_OK)
    return 1;
  if (SerialServoReadPosition(&huart1, 2, &p2) != READ_OK)
    return 2;
  if (SerialServoReadPosition(&huart1, 3, &p3) != READ_OK)
    return 3;
  if (SerialServoReadPosition(&huart1, 4, &p4) != READ_OK)
    return 4;

  float delta1 = fabsf((float)(450 - p1.returned_result));
  float delta2 = fabsf((float)(854 - p2.returned_result));
  float delta3 = fabsf((float)(500 - p3.returned_result));
  float delta4 = fabsf((float)(854 - p4.returned_result));

  float    max_delta = fmaxf(fmaxf(delta1, delta2), fmaxf(delta3, delta4));
  uint16_t move_time = (uint16_t)(max_delta / 0.18f);
  if (move_time < 1500)
    move_time = 1500;

  if (SerialServoMove(&huart1, 3, 500, move_time) != SET_OK)
    return 6;
  HAL_Delay(25);
  if (SerialServoMove(&huart1, 4, 854, move_time) != SET_OK)
    return 8;
  HAL_Delay(25);
  if (SerialServoMove(&huart1, 2, 854, move_time) != SET_OK)
    return 7;
  HAL_Delay(20);
  if (SerialServoMove(&huart1, 1, 450, move_time) != SET_OK)
    return 5;
  HAL_Delay(20);

  //  if (SerialServoMove(&huart1, 3, 500, move_time) != SET_OK)
  //    return 6;
  //	HAL_Delay(20);
  //	if (SerialServoMove(&huart1, 4, 800, move_time) != SET_OK)
  //    return 8;
  //	HAL_Delay(20);
  //  if (SerialServoMove(&huart1, 2, 800, move_time) != SET_OK)
  //    return 7;
  //	HAL_Delay(20);
  //  if (SerialServoMove(&huart1, 1, 450, move_time) != SET_OK)
  //    return 5;
  //	HAL_Delay(20);

  //	if (SerialServoMove(&huart1, 3, 417, move_time) != SET_OK)
  //    return 6;
  //	HAL_Delay(20);
  //	if (SerialServoMove(&huart1, 4, 791, move_time) != SET_OK)
  //    return 8;
  //	HAL_Delay(20);
  //  if (SerialServoMove(&huart1, 2, 875, move_time) != SET_OK)
  //    return 7;
  //	HAL_Delay(20);
  //  if (SerialServoMove(&huart1, 1, 450, move_time) != SET_OK)
  //    return 5;
  //	HAL_Delay(20);

  return 9;
}

uint8_t to_home_position(void)
{
  struct_result_t p1, p2, p3, p4;

  if (SerialServoReadPosition(&huart1, 1, &p1) != READ_OK)
    return 1;
  if (SerialServoReadPosition(&huart1, 2, &p2) != READ_OK)
    return 2;
  if (SerialServoReadPosition(&huart1, 3, &p3) != READ_OK)
    return 3;
  if (SerialServoReadPosition(&huart1, 4, &p4) != READ_OK)
    return 4;

  float delta1 = fabsf((float)(480 - p1.returned_result));
  float delta2 = fabsf((float)(625 - p2.returned_result));
  float delta3 = fabsf((float)(375 - p3.returned_result));
  float delta4 = fabsf((float)(500 - p4.returned_result));

  float    max_delta = fmaxf(fmaxf(delta1, delta2), fmaxf(delta3, delta4));
  uint16_t move_time = (uint16_t)(max_delta / 0.18f);
  if (move_time < 1800)
    move_time = 1800;

  if (SerialServoMove(&huart1, 2, 625, move_time) != SET_OK)
    return 6;
  HAL_Delay(5);
  if (SerialServoMove(&huart1, 3, 375, move_time) != SET_OK)
    return 7;
  HAL_Delay(5);
  if (SerialServoMove(&huart1, 4, 500, move_time) != SET_OK)
    return 8;
  HAL_Delay(5);
  if (SerialServoMove(&huart1, 1, 480, move_time) != SET_OK)
    return 5;
  HAL_Delay(5);

  return 9;
}

uint8_t to_moving_position(void)
{
  struct_result_t p1, p2, p3, p4;

  // if (SerialServoReadPosition(&huart1, 2, &p2) != READ_OK) return 1;
  if (SerialServoMove(&huart1, 2, 525, 1500) != SET_OK)
    return 2;
  HAL_Delay(10);

  if (SerialServoReadPosition(&huart1, 1, &p1) != READ_OK)
    return 3;
  if (SerialServoReadPosition(&huart1, 2, &p2) != READ_OK)
    return 4;
  if (SerialServoReadPosition(&huart1, 3, &p3) != READ_OK)
    return 5;
  if (SerialServoReadPosition(&huart1, 4, &p4) != READ_OK)
    return 6;

  float delta1 = fabsf((float)(835 - p1.returned_result));
  float delta2 = fabsf((float)(310 - p2.returned_result));
  float delta3 = fabsf((float)(480 - p3.returned_result));
  float delta4 = fabsf((float)(265 - p4.returned_result));

  float    max_delta = fmaxf(fmaxf(delta1, delta2), fmaxf(delta3, delta4));
  uint16_t move_time = (uint16_t)(max_delta / 0.18f);
  if (move_time < 1500)
    move_time = 1500;

  if (SerialServoMove(&huart1, 1, 835, move_time) != SET_OK)
    return 7;
  HAL_Delay(15);
  if (SerialServoMove(&huart1, 2, 310, move_time) != SET_OK)
    return 8;
  HAL_Delay(15);
  if (SerialServoMove(&huart1, 3, 480, move_time) != SET_OK)
    return 10;
  HAL_Delay(15);
  if (SerialServoMove(&huart1, 4, 265, move_time) != SET_OK)
    return 11;
  HAL_Delay(15);

  return 9;
}

void soft_reset(void)
{
  NVIC_SystemReset();
}

uint8_t read_joint_variables(float    *t1,
                             float    *t2,
                             float    *t3,
                             float    *t4,
                             uint16_t *p1,
                             uint16_t *p2,
                             uint16_t *p3,
                             uint16_t *p4)
{
  uint8_t         servo_id = 0;
  struct_result_t position1, position2, position3, position4;

  servo_id = 1;
  if (SerialServoReadPosition(&huart1, servo_id, &position1) == READ_OK)
  {
    *t1 = ((float)(position1.returned_result - 480)) * 0.24f;  // 480 is center
    *p1 = (uint16_t)position1.returned_result;
  }
  else
    return 0;
  HAL_Delay(3);

  servo_id = 2;
  if (SerialServoReadPosition(&huart1, servo_id, &position2) == READ_OK)
  {
    if (position2.returned_result >= 1000)
      *t2 = 0.0f;
    else
      *t2 = -((float)(position2.returned_result - 1000)) * 0.24f;
    *p2 = (uint16_t)position2.returned_result;
  }
  else
    return 0;
  HAL_Delay(3);

  servo_id = 3;
  if (SerialServoReadPosition(&huart1, servo_id, &position3) == READ_OK)
  {
    if (position3.returned_result <= 0)
      *t3 = 0.0f;
    else
      *t3 = -((float)(position3.returned_result)) * 0.24f;
    *p3 = (uint16_t)position3.returned_result;
  }
  else
    return 0;
  HAL_Delay(3);

  servo_id = 4;
  if (SerialServoReadPosition(&huart1, servo_id, &position4) == READ_OK)
  {
    *t4 = ((float)(position4.returned_result - 500)) * 0.24f;  // 500 is center
    *p4 = (uint16_t)position4.returned_result;
  }
  else
    return 0;

  return 1;
}

void send_data_with_id(uint8_t id, float value)
{
  uint8_t packet[6];

  memset(packet, 0, sizeof(packet));
  packet[0] = id;
  memcpy(&packet[1], &value, sizeof(float));

  uint8_t sum = 0;
  for (int i = 0; i < 5; i++)
    sum += packet[i];
  packet[5] = ~sum;

  HAL_UART_Transmit_DMA(&huart2, packet, sizeof(packet));
}

void reset_tof_sensor(void)
{
  HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_Delay(5);  // Giữ thấp ít nhất 1ms
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
