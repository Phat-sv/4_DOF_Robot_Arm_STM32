/**
 * @file       filters.c
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2025-05-18
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
#include "filters.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
float kalman_1D(float data)
{
  static float A = 1;
  static float H = 1;
  static float Q = 1.5;
  static float R = 2.0;

  static float x_output  = 0;
  static float P         = 2;
  static float first_run = 1;

  float x_predict, P_predict;
  float K_gain;

  float A_transpose = A;
  float H_transpose = H;

  if (first_run)
  {
    first_run = 0;
    x_output  = data;
  }

  // predict step
  x_predict = A * x_output;
  P_predict = (A * P * A_transpose) + Q;

  // compute kalman gain
  K_gain = (P_predict * H_transpose) / (H * P_predict * H_transpose + R);

  // Compute the estimate
  x_output = x_predict + K_gain * (data - H * x_predict);

  // compute the error covariance
  P = P_predict - (K_gain * H * P_predict);

  return x_output;
}

float moving_average_filter(float x)  // didn't tested yet
{
#define N 3
  static float   xbuf[N + 1] = {0};
  static float   prevAvg     = 0;
  static uint8_t firstRun    = 1;

  if (firstRun)
  {
    for (uint8_t i = 0; i < N + 1; i++)
    {
      xbuf[i] = x;
    }
    prevAvg  = x;
    firstRun = 0;
  }

  // Shift buffer
  for (int i = 0; i < N; i++)
  {
    xbuf[i] = xbuf[i + 1];
  }
  xbuf[N] = x;

  // Update average
  float avg = prevAvg + (x - xbuf[0]) / N;
  prevAvg   = avg;
  return avg;
}

float low_pass_filter(float coefficient, float data)
{
  static uint8_t first_run               = 1;
  static float   previous_filtered_value = 0;
  float          filtered_value;

  if (first_run)
  {
    previous_filtered_value = data;
    first_run               = 0;
  }

  if (coefficient > 1.0f)
    coefficient = 1.0f;
  if (coefficient < 0.0f)
    coefficient = 0.0f;

  filtered_value          = coefficient * previous_filtered_value + (1 - coefficient) * data;
  previous_filtered_value = filtered_value;
  return filtered_value;
}

float high_pass_filter(float coefficient, float data)
{
  static uint8_t first_run               = 1;
  static float   previous_filtered_value = 0;
  static float   previous_measured_value = 0;
  float          filtered_value;

  if (first_run)
  {
    previous_filtered_value = 0;
    previous_measured_value = data;
    first_run               = 0;
  }

  if (coefficient > 1.0f)
    coefficient = 1.0f;
  if (coefficient < 0.0f)
    coefficient = 0.0f;

  // filtered_value = 0.5 * (2 - coefficient) * (data - previous_measured_value) + (1 - coefficient) *
  // previous_filtered_value;
  filtered_value          = coefficient * (previous_filtered_value + data - previous_measured_value);
  previous_filtered_value = filtered_value;
  previous_measured_value = data;

  return filtered_value;
}
/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
