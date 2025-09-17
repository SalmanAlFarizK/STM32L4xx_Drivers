/******************************************************************************
 *		Source File Containing I2C related Informations and
 *		Drivers (API's) Related I2C Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "i2c.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
/* I2C CR1 Bit positions. */
#define I2C_CR1_PE				(0x00)
#define I2C_CR1_TXIE			(0x01)
#define I2C_CR1_RXIE			(0x02)
#define I2C_CR1_ADDRIE			(0x03)
#define I2C_CR1_NACKIE			(0x04)
#define I2C_CR1_STOPIE			(0x05)
#define I2C_CR1_TCIE			(0x06)
#define I2C_CR1_ERRIE			(0x07)
#define I2C_CR1_DNF				(0x08)
#define I2C_CR1_ANFOFF			(0x0C)
#define I2C_CR1_TXDMAEN			(0x0E)
#define I2C_CR1_RXDMAEN			(0x0F)
#define I2C_CR1_SBC				(0x10)
#define I2C_CR1_NOSTRETCH		(0x11)
#define I2C_CR1_WUPEN			(0x12)
#define I2C_CR1_GCEN			(0x13)
#define I2C_CR1_SMBHEN			(0x14)
#define I2C_CR1_SMBDEN			(0x15)
#define I2C_CR1_ALERTEN			(0x16)
#define I2C_CR1_PECEN			(0x17)

/* I2C CR2 Bit positions. */
#define I2C_CR2_SADD			(0x00)
#define I2C_CR2_RD_WRN 			(0x0A)
#define I2C_CR2_ADD10			(0x0B)
#define I2C_CR2_HEAD10R 		(0x0C)
#define I2C_CR2_START			(0x0D)
#define I2C_CR2_STOP			(0x0E)
#define I2C_CR2_NACK			(0x0F)
#define I2C_CR2_NBYTES			(0x10)
#define I2C_CR2_RELOAD			(0x18)
#define I2C_CR2_AUTOEND			(0x19)
#define I2C_CR2_PECBYTE			(0x1A)

/* I2C TIMINGR Bit positions. */
#define I2C_TIMINGR_SCLL		(0x00)
#define I2C_TIMINGR_SCLH		(0x08)
#define I2C_TIMINGR_SDADEL		(0x10)
#define I2C_TIMINGR_SCLDEL		(0x14)
#define I2C_TIMINGR_PRESC		(0x1C)

/* I2C TIMINGR Bit positions. */
#define I2C_OAR1_OA1MODE		(0x0A)
#define I2C_OAR1_OA1EN			(0x0F)

/* I2C TIMINGR Bit positions. */
#define I2C_ISR_TXE				(0x00)
#define I2C_ISR_TXIS			(0x01)
#define I2C_ISR_RXNE			(0x02)
#define I2C_ISR_ADDR			(0x03)
#define I2C_ISR_NACKF			(0x04)
#define I2C_ISR_STOPF			(0x05)
#define I2C_ISR_TC				(0x06)
#define I2C_ISR_TCR				(0x07)
#define I2C_ISR_BERR			(0x08)
#define I2C_ISR_ARLO			(0x09)
#define I2C_ISR_OVR				(0x0A)
#define I2C_ISR_PECERR			(0x0B)
#define I2C_ISR_TIMEOUT			(0x0C)
#define I2C_ISR_ALERT			(0x0D)
#define I2C_ISR_BUSY			(0x0F)
#define I2C_ISR_DIR				(0x10)
#define I2C_ISR_ADDCODE			(0x11)

/* I2C ICR Bit positions. */
#define I2C_ICR_ADDRCF			(0x03)
#define I2C_ICR_NACKCF			(0x04)
#define I2C_ICR_STOPCF			(0x05)
#define I2C_ICR_BERRCF			(0x08)
#define I2C_ICR_ARLOCF			(0x09)
#define I2C_ICR_OVRCF			(0x0A)
#define I2C_ICR_PECCF			(0x0B)
#define I2C_ICR_TIMOUTCF		(0x0C)
#define I2C_ICR_ALERTCF			(0x0D)

/******************************************************************************
 * static function declaration.
 *****************************************************************************/
static void GenerateStartCodition(I2C_Regdef_t* pI2Cx);
static void GenerateStopCodition(I2C_Regdef_t* pI2Cx);


/******************************************************************************
 * @brief : Function For Initializing I2C Peripheral.
 * @fn : I2C_Init(I2C_Handle_t* ptI2CHandle)
 *
 * @param[in] : ptI2CHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_Init(I2C_Handle_t* ptI2CHandle)
{
	/* Variable initialization. */
	uint32_t uiRegVal = 0;

	/* Validity Check. */
	if(NULL != ptI2CHandle)
	{
		/* Enable the Clock for I2C peripheral. */
		I2C_PclkCtrl(ptI2CHandle->pI2Cx, ENABLE);

		/* ACK configure. */
		if(ptI2CHandle->I2CConfig.I2C_AckControl >= eI2cAckDi
		&& ptI2CHandle->I2CConfig.I2C_AckControl < eI2cAckMax)
		{
			/* Calculate the register value. */
			uiRegVal = (ptI2CHandle->I2CConfig.I2C_AckControl << I2C_CR2_NACK);

			/* Reset the actual register. */
			ptI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_NACK);

			/* Set the actual register. */
			ptI2CHandle->pI2Cx->I2C_CR2 |= uiRegVal;

			/* Reset the local register. */
			uiRegVal = 0;
		}

		/* Configure the timing register. */
		if(I2C_SCL_SPEED_SM == ptI2CHandle->I2CConfig.I2C_SCLSpeed)
		{
			/* Standard mode configuration. */
			ptI2CHandle->pI2Cx->I2C_TIMINGR = 0x10420A28;
		}
		else if(I2C_SCL_SPEED_FM == ptI2CHandle->I2CConfig.I2C_SCLSpeed)
		{
			/* Fast mode configuration. */
			ptI2CHandle->pI2Cx->I2C_TIMINGR = 0x00B1112E;
		}

		/* Configure the Own Address 1 Register. */

		/* Reset the OA1 EN bit. */
		ptI2CHandle->pI2Cx->I2C_OAR1 &= ~(1 << I2C_OAR1_OA1EN);

		/* Set the device address. */
		ptI2CHandle->pI2Cx->I2C_OAR1 |= (ptI2CHandle->I2CConfig.I2C_DeviceAddress << 1);

		/* Set the OA1 EN bit. */
		ptI2CHandle->pI2Cx->I2C_OAR1 |= (1 << I2C_OAR1_OA1EN);
	}
	return;
}

/******************************************************************************
 * @brief : Function For De-Initializing I2C Peripheral.
 * @fn : I2C_DeInit(I2C_Regdef_t* pI2Cx)
 *
 * @param[in] : pI2Cx
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_DeInit(I2C_Regdef_t* pI2Cx)
{
	return;
}

/******************************************************************************
 * @brief : Function For Enabling I2C Peripheral.
 * @fn : I2C_Enable(I2C_Regdef_t* pI2Cx, uint8_t ucEnorDi)
 *
 * @param[in] : pI2Cx
 * @param[in] : ucEnorDi
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_Enable(I2C_Regdef_t* pI2Cx, uint8_t ucEnorDi)
{
	/* Validity check. */
	if(NULL != pI2Cx)
	{
		if(ENABLE == ucEnorDi)
		{
			pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_PE);
		}

		else if(DISABLE == ucEnorDi)
		{
			pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_PE);
		}
	}

	return;
}

/******************************************************************************
 * @brief : Function For Enabling or Disabling peripheral clock for I2C.
 * @fn : I2C_PclkCtrl(I2C_Regdef_t* pI2Cx,uint8_t EnorDi)
 *
 * @param[in] : pI2Cx
 * @param[in] : EnorDi
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_PclkCtrl(I2C_Regdef_t* pI2Cx,uint8_t EnorDi)
{
	/* Validation check. */
	if(NULL != pI2Cx)
	{
		if(ENABLE == EnorDi)
		{
			if(I2C1 == pI2Cx)
			{
				I2C1_PCLK_EN();
			}

			else if(I2C2 == pI2Cx)
			{
				I2C2_PCLK_EN();
			}

			else if(I2C3 == pI2Cx)
			{
				I2C3_PCLK_EN();
			}
		}

		else if(DISABLE == EnorDi)
		{
			if(I2C1 == pI2Cx)
			{
				I2C1_PCLK_DI();
			}

			else if(I2C2 == pI2Cx)
			{
				I2C2_PCLK_DI();
			}

			else if(I2C3 == pI2Cx)
			{
				I2C3_PCLK_DI();
			}
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For transmitting data over I2C peripheral.
 * @fn : I2C_MasterTxData(I2C_Handle_t* ptI2CHandle, uint8_t* ucTxBuff,
					  uint16_t uhTxSize, uint8_t ucSlaveAddr)
 *
 * @param[in] : ptI2CHandle
 * @param[in] : ucTxBuff
 * @param[in] : uhTxSize
 * @param[in] : ucSlaveAddr
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_MasterTxData(I2C_Handle_t* ptI2CHandle, uint8_t* ucTxBuff,
					  uint16_t uhTxSize, uint8_t ucSlaveAddr)
{
	/* Parameter validity check. */
	if((NULL != ptI2CHandle)
	&& (NULL != ucTxBuff)
	&& (NULL != ucTxBuff + uhTxSize))
	{

		/* Wait if the start bit is set (bus is busy). */
		while(IsI2CBusy(ptI2CHandle->pI2Cx));

		/* Send the address of slave . */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (ucSlaveAddr << 1);

		/* Program the number of bytes to be transmitted. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (uhTxSize << I2C_CR2_NBYTES);

		/* Configure write enable. */
		ptI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_RD_WRN);

		/* Configure auto end. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_AUTOEND);

		/* Generate the start condition. */
		GenerateStartCodition(ptI2CHandle->pI2Cx);

		/* Start the data transfer operation. */
		for(uint32_t i = 0; i < uhTxSize; ++i)
		{
			/* Wait until transmit data register . */
			while(!(ptI2CHandle->pI2Cx->I2C_ISR & ( 1 << I2C_ISR_TXE)));

			/* Load the data into data register. */
			ptI2CHandle->pI2Cx->I2C_TXDR = ucTxBuff[i];

		}

		/* Generate a stop condition. */
		GenerateStopCodition(ptI2CHandle->pI2Cx);

		/* Wait until stop is set. */
		while(!(ptI2CHandle->pI2Cx->I2C_ISR & ( 1 << I2C_ISR_STOPF)));

	}
	return;
}

/******************************************************************************
 * @brief : Function For Receiving data over I2C peripheral.
 * @fn : I2C_RxData(I2C_Regdef_t* pI2Cx, uint8_t* ucRxBuff, uint16_t uhRxSize)
 *
 * @param[in] : pI2Cx
 * @param[in] : ucRxBuff
 * @param[in] : uhRxSize
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_MasterRxData(I2C_Handle_t* ptI2CHandle, uint8_t* ucRxBuff,
		  uint16_t uhRxSize, uint8_t ucSlaveAddr)
{
	/* Validity Check. */
	if((NULL != ptI2CHandle)
	&& (NULL != ucRxBuff)
	&& (NULL != ucRxBuff + uhRxSize))
	{
		/* Wait if the start bit is set (bus is busy). */
		while(IsI2CBusy(ptI2CHandle->pI2Cx));

		/* Send the address of slave . */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (ucSlaveAddr << 1);

		/* Program the number of bytes to be received. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (uhRxSize << I2C_CR2_NBYTES);

		/* Configure read enable. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_RD_WRN);

		/* Configure auto end. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_AUTOEND);

		/* Generate the start condition. */
		GenerateStartCodition(ptI2CHandle->pI2Cx);

		/* read data from register and load into the buffer. */
		for(uint32_t i = 0; i < uhRxSize; ++i)
		{
			/* Wait until data arrives in data register . */
			while(!(ptI2CHandle->pI2Cx->I2C_ISR & ( 1 << I2C_ISR_RXNE)));

			/* Copy the data into the buffer. */
			ucRxBuff[i] = ptI2CHandle->pI2Cx->I2C_RXDR;
		}

		/* Generate a stop condition. */
		GenerateStopCodition(ptI2CHandle->pI2Cx);

		/* Wait until stop is set. */
		while(!(ptI2CHandle->pI2Cx->I2C_ISR & ( 1 << I2C_ISR_STOPF)));
	}
	return;
}

/******************************************************************************
 * @brief : Function For Configure Interrupt for I2C Peripheral.
 * @fn : I2C_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi)
 *
 * @param[in] : IRQNum
 * @param[in] : EnOrDi
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi)
{
	if(ENABLE == EnOrDi)
	{
		if(IRQNum <= 31)
		{
			/* ISER0 Register. */
			*NVIC_ISER0 |= ( 1 << IRQNum);
		}
		else if(IRQNum > 31 && IRQNum < 64)
		{
			/* ISER1 Register. */
			*NVIC_ISER1 |= ( 1 << (IRQNum % 32));
		}
		else if(IRQNum >= 64 && IRQNum < 96)
		{
			/* ISER2 Register. */
			*NVIC_ISER2 |= ( 1 << (IRQNum % 64));
		}
	}
	else
	{
		if(IRQNum <= 31)
		{
			/* ISER0 Register. */
			*NVIC_ICER0 |= ( 1 << IRQNum);
		}
		else if(IRQNum > 31 && IRQNum < 64)
		{
			/* ISER1 Register. */
			*NVIC_ICER1 |= ( 1 << (IRQNum % 32));
		}
		else if(IRQNum >= 64 && IRQNum < 96)
		{
			/* ISER2 Register. */
			*NVIC_ICER2 |= ( 1 << (IRQNum % 64));
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For Configuring priority for i2c interrupt.
 * @fn : I2C_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal)
 *
 * @param[in] : IRQNum
 * @param[in] : PriorityVal
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal)
{
	return;
}

/******************************************************************************
 * @brief : Function For checking the i2c busy flag.
 * @fn : IsI2CBusy(I2C_Regdef_t* pI2Cx)
 *
 * @param[in] : pI2Cx
 *
 * @param[out] : bool.
 *
 * @return : None
 *
 *****************************************************************************/
bool IsI2CBusy(I2C_Regdef_t* pI2Cx)
{
	bool bI2cBsyStat = false;

	if(pI2Cx->I2C_ISR & ( 1 << I2C_ISR_BUSY))
	{
		bI2cBsyStat = true;
	}

	return bI2cBsyStat;
}

/******************************************************************************
 * @brief : Function For checking the i2c busy flag.
 * @fn : IsI2CBusy(I2C_Regdef_t* pI2Cx)
 *
 * @param[in] : pI2Cx
 *
 * @param[out] : bool.
 *
 * @return : None
 *
 *****************************************************************************/
static void GenerateStartCodition(I2C_Regdef_t* pI2Cx)
{
	if(NULL != pI2Cx)
	{
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_START);
	}

	return;
}

/******************************************************************************
 * @brief : Function For checking the i2c busy flag.
 * @fn : IsI2CBusy(I2C_Regdef_t* pI2Cx)
 *
 * @param[in] : pI2Cx
 *
 * @param[out] : bool.
 *
 * @return : None
 *
 *****************************************************************************/
static void GenerateStopCodition(I2C_Regdef_t* pI2Cx)
{
	if(NULL != pI2Cx)
	{
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_STOP);
	}

	return;
}

/******************************************************************************
 * @brief : Function For checking the i2c busy flag.
 * @fn : IsI2CBusy(I2C_Regdef_t* pI2Cx)
 *
 * @param[in] : pI2Cx
 *
 * @param[out] : bool.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_MasterTxDataIT(I2C_Handle_t* ptI2CHandle, uint8_t* ucTxBuff,
					  uint16_t uhTxSize, uint8_t ucSlaveAddr)
{
	/* Parameter validity check. */
	if((NULL != ptI2CHandle)
	&& (NULL != ucTxBuff)
	&& (NULL != ucTxBuff + uhTxSize))
	{
		/* Assign the TX buffer address. */
		ptI2CHandle->pucTxBuff = ucTxBuff;

		/* Assign the TX length info. */
		ptI2CHandle->uhTxLen = uhTxSize;

		/* Update the busy flag. */
		ptI2CHandle->ucTxRxState = eI2cTxBusy;

		/* Set the salve address as device address. */
		ptI2CHandle->ucDevAddr = ucSlaveAddr;

		/* Configure Control register. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (ucSlaveAddr << 1);

		/* Program the number of bytes to be transmitted. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (uhTxSize << I2C_CR2_NBYTES);

		/* Configure write enable. */
		ptI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_RD_WRN);

		/* Configure auto end. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_AUTOEND);

		/* Generate the start condition. */
		GenerateStartCodition(ptI2CHandle->pI2Cx);

		ptI2CHandle->pI2Cx->I2C_CR1 |= ((1 << I2C_CR1_TXIE) | (1 << I2C_CR1_STOPIE) | (1 << I2C_CR1_ERRIE) | (1 << I2C_CR1_TCIE));

	}
}

/******************************************************************************
 * @brief : Function For checking the i2c busy flag.
 * @fn : IsI2CBusy(I2C_Regdef_t* pI2Cx)
 *
 * @param[in] : pI2Cx
 *
 * @param[out] : bool.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_MasterRxDataIT(I2C_Handle_t* ptI2CHandle, uint8_t* ucRxBuff,
		  uint16_t uhRxSize, uint8_t ucSlaveAddr)
{
	/* Parameter validity check. */
	if((NULL != ptI2CHandle)
	&& (NULL != ucRxBuff)
	&& (NULL != ucRxBuff + uhRxSize))
	{
		/* Assign the TX buffer address. */
		ptI2CHandle->pucRxBuff = ucRxBuff;

		/* Assign the TX length info. */
		ptI2CHandle->uhRxLen = uhRxSize;

		/* Update the busy flag. */
		ptI2CHandle->ucTxRxState = eI2cRxBusy;

		/* Set the salve address as device address. */
		ptI2CHandle->ucDevAddr = ucSlaveAddr;

		/* Configure Control register. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (ucSlaveAddr << 1);

		/* Program the number of bytes to be transmitted. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (uhRxSize << I2C_CR2_NBYTES);

		/* Configure write enable. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_RD_WRN);

		/* Configure auto end. */
		ptI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_AUTOEND);

		/* Generate the start condition. */
		GenerateStartCodition(ptI2CHandle->pI2Cx);

		ptI2CHandle->pI2Cx->I2C_CR1 |= (1<<I2C_CR1_RXIE) | (1<<I2C_CR1_STOPIE) | (1<<I2C_CR1_ERRIE);

	}
}

/******************************************************************************
 * @brief : Function For checking the i2c busy flag.
 * @fn : IsI2CBusy(I2C_Regdef_t* pI2Cx)
 *
 * @param[in] : pI2Cx
 *
 * @param[out] : bool.
 *
 * @return : None
 *
 *****************************************************************************/
void I2C_IRQHandling(I2C_Handle_t* ptI2CHandle)
{
	/* Variable initialization. */
	uint32_t uiReg1 = 0;
	uint32_t uiReg2 = 0;

	if(NULL != ptI2CHandle)
	{
		/* Check for the transmit interrupt. */
		uiReg1 = (ptI2CHandle->pI2Cx->I2C_CR1 & ( 1 << I2C_CR1_TXIE));
		uiReg2 = (ptI2CHandle->pI2Cx->I2C_ISR & ( 1 << I2C_ISR_TXIS));

		if(uiReg1 && uiReg2)
		{
			if(ptI2CHandle->uhTxLen > 0)
			{
				ptI2CHandle->pI2Cx->I2C_TXDR = *(ptI2CHandle->pucTxBuff++);
				ptI2CHandle->uhTxLen--;

				/* Check if transmission completed. */
				if(0 == ptI2CHandle->uhTxLen)
				{
					ptI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_TXIE);
				}
			}
		}

		/* Check Reception interrupt. */
		uiReg1 = (ptI2CHandle->pI2Cx->I2C_CR1 & ( 1 << I2C_CR1_RXIE));
		uiReg2 = (ptI2CHandle->pI2Cx->I2C_ISR & ( 1 << I2C_ISR_RXNE));

		if(uiReg1 && uiReg2)
		{
			if(ptI2CHandle->uhRxLen > 0)
			{
				/* Load the data to the reception buffer. */
				*(ptI2CHandle->pucRxBuff++) = ptI2CHandle->pI2Cx->I2C_RXDR;

				/* Decrement the size. */
				ptI2CHandle->uhRxLen -= 1;

				/* Check if reception completed. */
				if(0 == ptI2CHandle->uhRxLen)
				{
					ptI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_RXIE);
				}
			}
		}

		/* Check for STOPF detection */
		uiReg1 = (ptI2CHandle->pI2Cx->I2C_CR1 & ( 1 << I2C_CR1_STOPIE));
		uiReg2 = (ptI2CHandle->pI2Cx->I2C_ISR & ( 1 << I2C_ISR_STOPF));

		if(uiReg1 && uiReg2)
		{
			/* Clear Stop detection flag. */
			ptI2CHandle->pI2Cx->I2C_ICR |= ( 1 << I2C_ICR_STOPCF);

			/* reset the state and buffers. */
			ptI2CHandle->ucTxRxState = eI2cRxTxReady;
			ptI2CHandle->pucRxBuff = NULL;
			ptI2CHandle->pucTxBuff = NULL;
			ptI2CHandle->uhRxLen = 0;
			ptI2CHandle->uhTxLen = 0;
		}

		/* Generate Stop Condition if transmission completed. */
		uiReg1 = (ptI2CHandle->pI2Cx->I2C_CR1 & ( 1 << I2C_CR1_TCIE));
		uiReg2 = (ptI2CHandle->pI2Cx->I2C_ISR & ( 1 << I2C_ISR_TC));

		if(uiReg1 && uiReg2)
		{
			if(ptI2CHandle->uhTxLen == 0 &&
			   ptI2CHandle->ucTxRxState == eI2cTxBusy)
			{
				GenerateStopCodition(ptI2CHandle->pI2Cx);
			}
		}
	}
}
