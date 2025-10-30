/**
 * @file       arm_kinematic.h
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
#ifndef __KINEMATIC_H
#define __KINEMATIC_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "bus_servo_driver.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief <status returned when read data from servo>
 */
typedef enum
{
  OK     = 1,
  FAILED = 2
} enum_result_kinematic_t;

/**
 * @brief <>
 */
typedef struct
{
  float X;
  float Y;
  float Z;
} struct_coordinates_t;

/**
 * @brief <>
 */
typedef struct
{
  float required_theta1;
  float required_theta2a;
  float required_theta2b;
  float required_theta3a;
  float required_theta3b;
  float required_theta4a;
  float required_theta4b;
} struct_required_angle_t;

/* Public macros ------------------------------------------------------ */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

static inline float DEG_TO_RAD(float angle)
{
  return angle * PI / 180.0f;
}

static inline float RAD_TO_DEG(float angle)
{
  return angle * 180.0f / PI;
}

static inline uint16_t convert_to_absolute_int(float a)
{
  return (uint16_t)((a < 0.0f) ? -a : a);
}

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

enum_result_kinematic_t inverse_kinematic(float                    destination_X,
                                          float                    destination_Y,
                                          float                    destination_Z,
                                          uint16_t                 p1,
                                          uint16_t                 p2,
                                          uint16_t                 p3,
                                          uint16_t                 p4,
                                          struct_required_angle_t *angle);

enum_result_kinematic_t inverse(float                    destination_X,
                                float                    destination_Y,
                                float                    destination_Z,
                                float                    t2,
                                float                    t3,
                                float                    t4,
                                struct_required_angle_t *angle);
#endif  // __KINEMATIC_H
