/******************************************************************************
 *		Header File Containing I2C related Informations and
 *		Drivers (API's) Related I2C Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l476xx.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
#define I2C_SCL_SPEED_SM		(100000)
#define I2C_SCL_SPEED_FM		(400000)

/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/
typedef enum _I2C_ACKState_
{
	eI2cAckDi = 0,
	eI2cACKEn,
	eI2cAckMax
} I2C_ACKState;

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/
/*
 * @brief : Configuration Structure for I2C.
 */
typedef struct _I2C_Config_t_
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint16_t I2C_FMDutyCycle;
} I2C_Config_t;

/*
 * @brief : Handle Structure for I2C.
 */
typedef struct _I2C_Handle_t_
{
	I2C_Regdef_t* pI2Cx;
	I2C_Config_t  I2CConfig;
} I2C_Handle_t;

/******************************************************************************
 * Function Declarations.
 *****************************************************************************/
void I2C_Init(I2C_Handle_t* ptI2CHandle);
void I2C_DeInit(I2C_Regdef_t* pI2Cx);
void I2C_Enable(I2C_Regdef_t* pI2Cx, uint8_t ucEnorDi);
void I2C_PclkCtrl(I2C_Regdef_t* pI2Cx,uint8_t EnorDi);
void I2C_MasterTxData(I2C_Handle_t* ptI2CHandle, uint8_t* ucTxBuff,
					  uint16_t uhTxSize, uint8_t ucSlaveAddr);
void I2C_MasterRxData(I2C_Handle_t* ptI2CHandle, uint8_t* ucRxBuff,
		  uint16_t uhRxSize, uint8_t ucSlaveAddr);
void I2C_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal);
bool IsI2CBusy(I2C_Regdef_t* pI2Cx);




#endif /* INC_I2C_H_ */
