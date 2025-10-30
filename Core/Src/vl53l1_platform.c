/**
 * @file       vl53l1_platform.c
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

/* Includes ----------------------------------------------------------- */
#include "vl53l1_platform.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static I2C_HandleTypeDef *_vl53l1x_i2c_port = NULL;

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void vl53l1x_assignI2C(I2C_HandleTypeDef *hi2c_device)
{
	_vl53l1x_i2c_port = hi2c_device;
}


int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
	if (HAL_I2C_Mem_Read(_vl53l1x_i2c_port, dev, index, I2C_MEMADD_SIZE_16BIT, data, 1, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}


int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
	if (HAL_I2C_Mem_Read(_vl53l1x_i2c_port, dev, index, I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, 2, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else {
		*data=__REVSH(*data);
		return 0;
	}
}


int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
	if (HAL_I2C_Mem_Read(_vl53l1x_i2c_port, dev, index, I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, 4, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else {
		*data=__REV(*data);
		return 0;
	}
}


int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	if (HAL_I2C_Mem_Read(_vl53l1x_i2c_port, dev, index, I2C_MEMADD_SIZE_16BIT, pdata, count, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}


int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
	if (HAL_I2C_Mem_Write(_vl53l1x_i2c_port, dev, index, I2C_MEMADD_SIZE_16BIT, &data, 1, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}


int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
	data=__REVSH(data);
	if (HAL_I2C_Mem_Write(_vl53l1x_i2c_port, dev, index, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&data, 2, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}


int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
	data=__REV(data);
	if (HAL_I2C_Mem_Write(_vl53l1x_i2c_port, dev, index, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&data, 4, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}


int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	if (HAL_I2C_Mem_Write(_vl53l1x_i2c_port, dev, index, I2C_MEMADD_SIZE_16BIT, pdata, count, I2C_COMM_TIMEOUT))
		return VL53L1__IO_ERROR;
	else
		return 0;
}


void VL53L1_WaitMs(uint16_t dev, uint16_t wait_ms)
{
	HAL_Delay(wait_ms);
}


/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */








