/**
 * @file       filters.h
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

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __FILTERS_H
#define __FILTERS_H

/* Includes ----------------------------------------------------------- */
// #include "arm_math.h"
#include <math.h>
#include <stdint.h>

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  <filter 1D data for simple situation such as distance, temperature>
 *
 * @param[in]     <data>  <measured data from sensor>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <API attention note>
 *
 * @return  filtered data (or estimated data)
 */
float kalman_1D(float data);

/**
 * @brief  <Filter out data types that vary a lot over time, such as sonar sensors>
 *
 * @param[in]     <data>  <measured data from sensor>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <The larger n, the smoother the filtered data but the higher the delay.
 Inside the function, there is a defined parameter named N. That is the number of data points
 that are used to calculate average. Change "N" to change filter coefficient>
 *
 * @return  filtered data
 */
float moving_average_filter(float data);

/**
 * @brief  <filter high frequency noise (retain low frequency values)>
 * @param[in]     <coefficient>  <coefficient of low pass filter, from 0 to 1>
 * @param[in]     <data>  <measured data from sensor>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <coefficient --> 0: faster response, less smooth; coefficient --> 1: tight filtering, slow
 * response to new data> <coefficient = 0: no filter, return original signal; coefficient = 1: Filter is
 * "frozen", not updated>
 *
 * @return  filtered data
 */
float low_pass_filter(float coefficient, float data);

/**
 * @brief  <filter high frequency noise (retain low frequency values)>
 *
 * @param[in]     <coefficient>  <coefficient of low pass filter, from 0 to 1>
 * @param[in]     <data>  <measured data from sensor>
 * @param[out]    none
 * @param[inout]  none
 *
 * @attention  <coefficient --> 0: strong filter; coefficient --> 1: weak filter>
 *             <coefficient = 0: ...; coefficient = 1: ...>
 * @return  filtered data
 */
float high_pass_filter(float coefficient, float data);

#endif  // __FILTERS_H

/* End of file -------------------------------------------------------- */
