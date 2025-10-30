/**
 * @file       arm_kinematic.c
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
#include "kinematic.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */

enum_result_kinematic_t inverse_kinematic(float                    destination_X,
                                          float                    destination_Y,
                                          float                    destination_Z,
                                          uint16_t                 p1,
                                          uint16_t                 p2,
                                          uint16_t                 p3,
                                          uint16_t                 p4,
                                          struct_required_angle_t *angle)
{
  float n1, n2, n3, nx, ny;

  // calculate theta1
  angle->required_theta1 = atan2f(destination_Y, destination_X);

  // variables
  n1 = destination_X * cosf(angle->required_theta1) + destination_Y * sinf(angle->required_theta1) - 22;
  n2 = destination_Z - 57.45;
  n3 = destination_X * sinf(angle->required_theta1) - destination_Y * cosf(angle->required_theta1);
  nx = n1 - 120;
  ny = n2;

  // calculate theta3
  float cos_theta3 = ((n1 - 120) * (n1 - 120) + n2 * n2 - 60125) / 58900.0;
  if (cos_theta3 > 1.0f)
    cos_theta3 = 1.0f;
  if (cos_theta3 < -1.0f)
    cos_theta3 = -1.0f;
  float sin_theta3a       = sqrtf(1 - cos_theta3 * cos_theta3);
  float sin_theta3b       = -sin_theta3a;
  angle->required_theta3a = atan2f(sin_theta3a, cos_theta3);
  angle->required_theta3b = atan2f(sin_theta3b, cos_theta3);

  // calculate theta2
  float cos_theta2a =
    (nx * (190 * cos_theta3 + 155) + 190 * ny * sin_theta3a) /
    ((190 * cos_theta3 + 155) * (190 * cos_theta3 + 155) + 190 * 190 * sin_theta3a * sin_theta3a);
  float cos_theta2b =
    (nx * (190 * cos_theta3 + 155) + 190 * ny * sin_theta3b) /
    ((190 * cos_theta3 + 155) * (190 * cos_theta3 + 155) + 190 * 190 * sin_theta3b * sin_theta3b);
  float sin_theta2a =
    (ny * (190 * cos_theta3 + 155) - 190 * nx * sin_theta3a) /
    ((190 * cos_theta3 + 155) * (190 * cos_theta3 + 155) + 190 * 190 * sin_theta3a * sin_theta3a);
  float sin_theta2b =
    (ny * (190 * cos_theta3 + 155) - 190 * nx * sin_theta3b) /
    ((190 * cos_theta3 + 155) * (190 * cos_theta3 + 155) + 190 * 190 * sin_theta3b * sin_theta3b);
  angle->required_theta2a = atan2f(sin_theta2a, cos_theta2a);
  angle->required_theta2b = atan2f(sin_theta2b, cos_theta2b);

  // calculate theta4
  angle->required_theta4a = -(angle->required_theta2a) - (angle->required_theta3a);
  angle->required_theta4b = -(angle->required_theta2b) - (angle->required_theta3b);

  // transform angle into control value
  uint16_t destination1 = 480;
  uint16_t destination2 = 625;
  uint16_t destination3 = 375;
  uint16_t destination4 = 500;
  float    destination2a, destination2b, destination3a, destination3b, destination4a, destination4b;

  destination1 = 480 + (uint16_t)((RAD_TO_DEG(angle->required_theta1)) / 0.24);

  destination2a = 1000 - (uint16_t)((RAD_TO_DEG(angle->required_theta2a)) / 0.24);
  destination2b = 1000 - (uint16_t)((RAD_TO_DEG(angle->required_theta2b)) / 0.24);

  destination3a = 0 - (uint16_t)((RAD_TO_DEG(angle->required_theta3a)) / 0.24);
  destination3b = 0 - (uint16_t)((RAD_TO_DEG(angle->required_theta3b)) / 0.24);

  destination4a = 500 + (uint16_t)((RAD_TO_DEG(angle->required_theta4a)) / 0.24);
  destination4b = 500 + (uint16_t)((RAD_TO_DEG(angle->required_theta4b)) / 0.24);

  bool is_valid_a = angle->required_theta2a > 0.0f && angle->required_theta3a < 0.0f;
  bool is_valid_b = angle->required_theta2b > 0.0f && angle->required_theta3b < 0.0f;

  if (is_valid_a)
  {
    destination2 = destination2a;
    destination3 = destination3a;
    destination4 = destination4a;
  }
  else if (is_valid_b)
  {
    destination2 = destination2b;
    destination3 = destination3b;
    destination4 = destination4b;
  }
  else
  {
    return FAILED;
  }

  float delta1 = fabsf((float)(destination1 - p1));
  float delta2 = fabsf((float)(destination2 - p2));
  float delta3 = fabsf((float)(destination3 - p3));
  float delta4 = fabsf((float)(destination4 - p4));

  float    max_delta = fmaxf(fmaxf(delta1, delta2), fmaxf(delta3, delta4));
  uint16_t move_time = (uint16_t)(max_delta / 0.18f);
  if (move_time < 1800)
    move_time = 1800;

  SerialServoMove(&huart1, 1, destination1, move_time);
  HAL_Delay(5);
  SerialServoMove(&huart1, 3, destination3, move_time);
  HAL_Delay(5);
  SerialServoMove(&huart1, 4, destination4, move_time);
  HAL_Delay(5);
  SerialServoMove(&huart1, 2, destination2, move_time);
  HAL_Delay(move_time);

  return OK;
}

enum_result_kinematic_t inverse(float                    destination_X,
                                float                    destination_Y,
                                float                    destination_Z,
                                float                    t2,
                                float                    t3,
                                float                    t4,
                                struct_required_angle_t *angle)
{
  float x_w, z_w;
  t2  = DEG_TO_RAD(t2);
  t3  = DEG_TO_RAD(t3);
  t4  = DEG_TO_RAD(t4);
  x_w = destination_X - 120 * cosf(t2 + t3 + t4);
  z_w = destination_Z - 120 * sinf(t2 + t3 + t4);

  float D   = (x_w * x_w + z_w * z_w - 155 * 155 - 190 * 190) / (2 * 155 * 190);
  float t1  = atan2f(destination_Y, destination_X);
  float t3a = atan2f(sqrt(1 - D * D), D);
  float t3b = atan2f(-sqrt(1 - D * D), D);
  float t2a = atan2f(z_w, x_w) - atan2f(190 * sinf(t3a), 155 + 190 * cosf(t3a));
  float t2b = atan2f(z_w, x_w) - atan2f(190 * sinf(t3b), 155 + 190 * cosf(t3b));

  float t4a = (t2 + t3 + t4) - t2a - t3a;
  float t4b = (t2 + t3 + t4) - t2b - t3b;

  // transform angle into control value
  uint16_t destination1 = 480;
  uint16_t destination2 = 625;
  uint16_t destination3 = 375;
  uint16_t destination4 = 500;
  float    destination2a, destination2b, destination3a, destination3b, destination4a, destination4b;

  destination1 = 480 + (uint16_t)((RAD_TO_DEG(t1)) / 0.24);

  destination2a = 1000 - (uint16_t)((RAD_TO_DEG(t2a)) / 0.24);
  destination2b = 1000 - (uint16_t)((RAD_TO_DEG(t2b)) / 0.24);

  destination3a = 0 - (uint16_t)((RAD_TO_DEG(t3a)) / 0.24);
  destination3b = 0 - (uint16_t)((RAD_TO_DEG(t3b)) / 0.24);

  destination4a = 500 + (uint16_t)((RAD_TO_DEG(t4a)) / 0.24);
  destination4b = 500 + (uint16_t)((RAD_TO_DEG(t4b)) / 0.24);

  bool is_valid_a = t2a > 0.0f && t3a < 0.0f;
  bool is_valid_b = t2b > 0.0f && t3b < 0.0f;

  if (is_valid_a)
  {
    destination2 = destination2a;
    destination3 = destination3a;
    destination4 = destination4a;
  }
  else if (is_valid_b)
  {
    destination2 = destination2b;
    destination3 = destination3b;
    destination4 = destination4b;
  }
  else
  {
    return FAILED;
  }

  struct_result_t p1, p2, p3, p4;

  if (SerialServoReadPosition(&huart1, 1, &p1) != READ_OK)
    return FAILED;
  if (SerialServoReadPosition(&huart1, 2, &p2) != READ_OK)
    return FAILED;
  if (SerialServoReadPosition(&huart1, 3, &p3) != READ_OK)
    return FAILED;
  if (SerialServoReadPosition(&huart1, 4, &p4) != READ_OK)
    return FAILED;

  float time1 = fabsf((float)(destination1 - p1.returned_result)) / 0.15f;
  float time2 = fabsf((float)(destination2 - p2.returned_result)) / 0.15f;
  float time3 = fabsf((float)(destination3 - p3.returned_result)) / 0.15f;
  float time4 = fabsf((float)(destination4 - p4.returned_result)) / 0.15f;

  float max_time = fmaxf(fmaxf(time1, time2), fmaxf(time3, time4));
  if (max_time < 1800)
    max_time = 1800;
  uint16_t move_time = (uint16_t)max_time;

  SerialServoMove(&huart1, 1, destination1, move_time);
  HAL_Delay(5);
  SerialServoMove(&huart1, 3, destination3, move_time);
  HAL_Delay(5);
  SerialServoMove(&huart1, 4, destination4, move_time);
  HAL_Delay(5);
  SerialServoMove(&huart1, 2, destination2, move_time);

  HAL_Delay(move_time + 100);

  return OK;
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
