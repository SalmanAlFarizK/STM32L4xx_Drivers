/******************************************************************************
 *		Source File Containing SPI related Informations and
 *		Drivers (API's) Related SPI Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "spi.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
/* SPI CR1 Bit positions. */
#define SPI_CR1_CPHA		(0x00)
#define SPI_CR1_CPOL		(0x01)
#define SPI_CR1_MODE		(0x02)
#define SPI_CR1_BAUD		(0x03)
#define SPI_CR1_SPE			(0x06)
#define SPI_CR1_SSI			(0x08)
#define SPI_CR1_SSM			(0x09)
#define SPI_CR1_RXONLY		(0x0A)
#define SPI_CR1_BIDIMODE	(0x0F)


/* SPI CR2 Bit positions. */
#define SPI_CR2_SSOE		(0x02)
#define SPI_CR2_ERRIE		(0x05)
#define SPI_CR2_RXNEIE		(0x06)
#define SPI_CR2_TXEIE		(0x07)
#define SPI_CR2_DS			(0x08)

/* SPI SR Bit positions. */
#define SPI_SR_RXNE			(0x00)
#define SPI_SR_TXE			(0x01)
#define SPI_SR_MODF			(0x05)
#define SPI_SR_OVR			(0x06)
#define SPI_SR_BSY			(0x07)
#define SPI_SR_FRE			(0x08)

/* SPI Reset macros. */
#define SPI1_REG_RESET()		do{(RCC->RCC_APB2RSTR  |= (1 << 12)); (RCC->RCC_APB2RSTR  &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()		do{(RCC->RCC_APB1RSTR1 |= (1 << 14)); (RCC->RCC_APB1RSTR1 &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()		do{(RCC->RCC_APB1RSTR1 |= (1 << 15)); (RCC->RCC_APB1RSTR1 &= ~(1 << 15));}while(0)

#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3

/******************************************************************************
 * Static function declaration.
 *****************************************************************************/
static void HandleSpiTxeInt(SPI_Handle_t* ptSpiHandle);
static void HandleSpiRxneInt(SPI_Handle_t* ptSpiHandle);
static void HandleSpiOvrInt(SPI_Handle_t* ptSpiHandle);
static void SpiApplicationEvntCallBack(SPI_Handle_t* ptSpiHandle, uint8_t ucEvnt);

/******************************************************************************
 * @brief : Function For Initializing SPI Peripheral.
 * @fn : SPI_Init(SPI_Handle_t* ptSpiHandle)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_Init(SPI_Handle_t* ptSpiHandle)
{
	/* Parameter initialization. */
	uint32_t uiregVal = 0;

	/* Parameter check. */
	if(NULL != ptSpiHandle)
	{
		/* Set the SPI mode. */
		if(ptSpiHandle->SPIConfig.SPI_DeviceMode >= eSPIModeSlave
		&& ptSpiHandle->SPIConfig.SPI_DeviceMode < eSPIModeMax)
		{
			/* Calculate the local register value. */
			uiregVal |= (ptSpiHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MODE);

			/* Reset the actual  register. */
			ptSpiHandle->pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_MODE);

			/* Set the actual register. */
			ptSpiHandle->pSPIx->SPI_CR1 |= uiregVal;

			/* Reset the local register value. */
			uiregVal = 0;
		}

		/* Configure the Bus. */
		if(ptSpiHandle->SPIConfig.SPI_BusConfig >= eSPIBusConfigFullDuplex
		&& ptSpiHandle->SPIConfig.SPI_BusConfig < eSPIBusConfigMax)
		{
			switch(ptSpiHandle->SPIConfig.SPI_BusConfig)
			{
				/* BIDI mode should be cleared */
				case eSPIBusConfigFullDuplex:
				{
					/* Calculate the local register value. */
					uiregVal &= ~(1 << SPI_CR1_BIDIMODE);

					/* Reset the actual register value. */
					ptSpiHandle->pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_BIDIMODE);

					/* Set the actual register. */
					ptSpiHandle->pSPIx->SPI_CR1 |= uiregVal;

					/* Reset the local register. */
					uiregVal = 0;

					break;
				}

				/* BIDI mode should be set */
				case eSPIBusConfigHalfDuplex:
				{
					/* Calculate the local register value. */
					uiregVal |= ( 1 << SPI_CR1_BIDIMODE);

					/* Reset the actual register value. */
					ptSpiHandle->pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_BIDIMODE);

					/* Set the actual register. */
					ptSpiHandle->pSPIx->SPI_CR1 |= uiregVal;

					/* Reset the local register. */
					uiregVal = 0;

					break;
				}

				/* BIDI mode should be cleared
				 RX only bit must be set. */
				case eSPIBusConfigSimplexRxOnly:
				{
					/* C;ear the BIDI mode. */
					uiregVal &= ~(1 << SPI_CR1_BIDIMODE);

					/* Set the RX only bit. */
					uiregVal |= ( 1 << SPI_CR1_RXONLY );

					/* Reset the actual register. */
					ptSpiHandle->pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_BIDIMODE);

					ptSpiHandle->pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_RXONLY);

					/* Set the actual register. */
					ptSpiHandle->pSPIx->SPI_CR1 |= uiregVal;

					/* Reset the local register. */
					uiregVal = 0;

					break;
				}

				default:
				{
					/* Reset the register value. */
					uiregVal = 0;
					break;
				}
			}
		}

		/* Configure SPI serial clock speed (baud rate) */
		if(ptSpiHandle->SPIConfig.SPI_SclkSpeed >= eSpiSclkDiv2
		&& ptSpiHandle->SPIConfig.SPI_SclkSpeed < eSpiSclkDivmax)
		{
			/* Calculate the register value. */
			uiregVal |= (ptSpiHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BAUD);

			/* Reset the actual register. */
			ptSpiHandle->pSPIx->SPI_CR1 &= ~(0x7 << SPI_CR1_BAUD);

			/* Set the actual register value. */
			ptSpiHandle->pSPIx->SPI_CR1 |= uiregVal;

			/* Reset the local register. */
			uiregVal = 0;
		}

		/* Configure the DFF. */
		if(ptSpiHandle->SPIConfig.SPI_DFF >= eSPIDff8Bits
		&& ptSpiHandle->SPIConfig.SPI_DFF < eSPIDffMax)
		{
			/* Calculate the resister value. */
			uiregVal |= (ptSpiHandle->SPIConfig.SPI_DFF << SPI_CR2_DS);

			/* Reset the actual register. */
			ptSpiHandle->pSPIx->SPI_CR2 &= ~(0xF << SPI_CR2_DS);

			/* Set the actual register. */
			ptSpiHandle->pSPIx->SPI_CR2 |= uiregVal;

			/* Reset the local register. */
			uiregVal = 0;
		}

		/* Configure CPOL*/
		if(ptSpiHandle->SPIConfig.SPI_CPOL >= eSPICpolState0
		&& ptSpiHandle->SPIConfig.SPI_CPOL < eSPICpolStateMax)
		{
			/* Calculate the register value. */
			uiregVal |= (ptSpiHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

			/* Reset the actual register. */
			ptSpiHandle->pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_CPOL);

			/* Set the actual register. */
			ptSpiHandle->pSPIx->SPI_CR1 |= uiregVal;

			/* Reset the local register. */
			uiregVal = 0;
		}

		/* Configure CPHASE. */
		if(ptSpiHandle->SPIConfig.SPI_CPHA >= eSPICphaseState0
		&& ptSpiHandle->SPIConfig.SPI_CPHA < eSPICphaseStateMax)
		{
			/* Calculate the register value. */
			uiregVal |= (ptSpiHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

			/* Reset the actual register. */
			ptSpiHandle->pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_CPHA);

			/* Set the actual register. */
			ptSpiHandle->pSPIx->SPI_CR1 |= uiregVal;

			/* Reset the local register. */
			uiregVal = 0;
		}

		/* Configure SSM. */
		if(ptSpiHandle->SPIConfig.SPI_SSM >= eSsmSwEn
		&& ptSpiHandle->SPIConfig.SPI_SSM < eSsmMax)
		{
			/* Calculate the register value. */
			uiregVal |= (ptSpiHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

			/* Reset the actual register. */
			ptSpiHandle->pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SSM);

			/* Set the actual register. */
			ptSpiHandle->pSPIx->SPI_CR1 |= uiregVal;

			/* Reset the local register. */
			uiregVal = 0;
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For De Initializing SPI Peripheral.
 * @fn : SPI_Init(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : pSPIx
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_DeInit(SPI_Regdef_t* pSPIx)
{
	/* parameter validity check. */
	if(NULL != pSPIx)
	{
		if(SPI1 == pSPIx)
		{
			SPI1_REG_RESET();
		}
		else if(SPI2 == pSPIx)
		{
			SPI2_REG_RESET();
		}
		else if(SPI3 == pSPIx)
		{
			SPI3_REG_RESET();
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For De Initializing SPI Peripheral.
 * @fn : SPI_Init(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : pSPIx
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_Enable(SPI_Regdef_t* pSPIx, uint8_t ucEnorDi)
{
	/* Parameter validity check. */
	if(NULL != pSPIx)
	{
		if(ENABLE == ucEnorDi)
		{
			pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
		}
		else if(DISABLE == ucEnorDi)
		{
			pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
		}
	}

	return;
}

/******************************************************************************
 * @brief : Function For De Initializing SPI Peripheral.
 * @fn : SPI_Init(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : pSPIx
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_SsiConfig(SPI_Regdef_t* pSPIx, uint8_t ucEnorDi)
{
	/* Parameter validity check. */
	if(NULL != pSPIx)
	{
		if(ENABLE == ucEnorDi)
		{
			pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
		}
		else if(DISABLE == ucEnorDi)
		{
			pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
		}
	}

	return;
}

/******************************************************************************
 * @brief : Function For controlling the peripheral clock of SPI Peripheral.
 * @fn : SPI_PclkCtrl(SPI_Regdef_t* pSPIx, uint8_t EnorDi)
 *
 * @param[in] : pSPIx
 * @param[in] : EnorDi
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_PclkCtrl(SPI_Regdef_t* pSPIx, uint8_t EnorDi)
{
	if(ENABLE == EnorDi)
	{
		if(SPI1 == pSPIx)
		{
			SPI1_PCLK_EN();
		}
		else if(SPI2 == pSPIx)
		{
			SPI2_PCLK_EN();
		}
		else if(SPI3 == pSPIx)
		{
			SPI3_PCLK_EN();
		}
	}
	else if(DISABLE == EnorDi)
	{
		if(SPI1 == pSPIx)
		{
			SPI1_PCLK_DI();
		}
		else if(SPI2 == pSPIx)
		{
			SPI2_PCLK_DI();
		}
		else if(SPI3 == pSPIx)
		{
			SPI3_PCLK_DI();
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For Transmitting data through SPI Peripheral.
 * @fn : SPI_TxData(SPI_Regdef_t* pSPIx, uint8_t* ucTxBuff, uint16_t uhTxSize)
 *
 * @param[in] : pSPIx
 * @param[in] : ucTxBuff
 * @param[in] : uhTxSize
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_TxData(SPI_Regdef_t* pSPIx, uint8_t* ucTxBuff, uint16_t uhTxSize)
{
	/* Parameter validity check. */
	uint8_t ucTxIndex = 0;

	if((NULL != pSPIx)
	&& (NULL != ucTxBuff)
	&& (NULL != ucTxBuff + uhTxSize))
	{
		while(ucTxIndex < uhTxSize)
		{
			/* Wait until TX buffer is not empty. */
			while(!(pSPIx->SPI_SR & ( 1 << SPI_SR_TXE)));

			/* Transfer the data. */
			pSPIx->SPI_DR = ucTxBuff[ucTxIndex];

			/* Increment the index. */
			ucTxIndex += 1;
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For Receiving data through SPI Peripheral.
 * @fn : SPI_RxData(SPI_Regdef_t* pSPIx, uint8_t* ucRxBuff, uint16_t uhrxSize)
 *
 * @param[in] : pSPIx
 * @param[in] : ucRxBuff
 * @param[in] : uhRxSize
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_RxData(SPI_Regdef_t* pSPIx, uint8_t* ucRxBuff, uint16_t uhRxSize)
{
	/* Variable Initialization. */
	uint8_t ucRxIdx = 0;

	/* Parameter validity check. */
	if(NULL != pSPIx
	&& NULL != ucRxBuff
	&& NULL != ucRxBuff + uhRxSize)
	{
		while(ucRxIdx < uhRxSize)
		{
			/* Wait until TX buffer is not empty. */
			while(!(pSPIx->SPI_SR & ( 1 << SPI_SR_RXNE)));

			/* read from data register. */
			ucRxBuff[ucRxIdx] = pSPIx->SPI_DR;

			/* Increment the index. */
			ucRxIdx += 1;
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For Receiving data through SPI Peripheral.
 * @fn : SPI_RxData(SPI_Regdef_t* pSPIx, uint8_t* ucRxBuff, uint16_t uhrxSize)
 *
 * @param[in] : pSPIx
 * @param[in] : ucRxBuff
 * @param[in] : uhRxSize
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_TxRxData(SPI_Regdef_t* pSPIx, uint8_t* ucTxBuff,
		uint8_t* ucRxBuff, uint16_t uhSize)
{
    uint16_t idx = 0;

    if((pSPIx == NULL) || (ucTxBuff == NULL) || (ucRxBuff == NULL) || (uhSize == 0))
    {
        return;
    }

    for(idx = 0; idx < uhSize; idx++)
    {
        /* Wait until TX buffer is empty */
        while(!(pSPIx->SPI_SR & (1 << SPI_SR_TXE)));

        /* Send data */
        pSPIx->SPI_DR = ucTxBuff[idx];

        /* Wait until data is received */
        while(!(pSPIx->SPI_SR & (1 << SPI_SR_RXNE)));

        /* Read received data */
        ucRxBuff[idx] = pSPIx->SPI_DR;
    }

    /* Wait for transmission to complete */
    while(pSPIx->SPI_SR & (1 << SPI_SR_BSY));
}

/******************************************************************************
 * @brief : Function For Configuring Interrupt SPI Peripheral.
 * @fn : SPI_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi)
 *
 * @param[in] : IRQNum
 * @param[in] : EnOrDi
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi)
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
 * @brief : Function For Configuring Interrupt  Priority for SPI Peripheral.
 * @fn : SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal)
 *
 * @param[in] : IRQNum
 * @param[in] : PriorityVal
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal)
{
	return;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
bool IsSpiBusy(SPI_Regdef_t* pSPIx)
{
	/* Parameter Initialization. */
	bool isSpiBusy = false;

	/* Parameter validity check. */
	if(NULL != pSPIx)
	{
		/* Check the BSY bit state. */
		if(pSPIx->SPI_SR & (1 << SPI_SR_BSY) )
		{
			isSpiBusy = true;
		}
	}

	/* Return the state. */
	return isSpiBusy;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_TxDataIT(SPI_Handle_t* ptSpiHandle, uint8_t* pucTxBuff,
				uint16_t uhTxSize)
{
	/* Validity check. */
	if((NULL != ptSpiHandle)
	&& (NULL != pucTxBuff)
	&& (NULL != pucTxBuff + uhTxSize))
	{
		if(ptSpiHandle->ucTxState != eSpiOpStateBsyInTx)
		{
			/* Save the TX buff address and length in some global variables. */
			ptSpiHandle->pucTxBuff = pucTxBuff;

			ptSpiHandle->uhTxLen = uhTxSize;

			/* Mark the SPI state as busy in transmission. */
			ptSpiHandle->ucTxState = eSpiOpStateBsyInTx;

			/* Enable the TXEIE control bit whenever TXE flag is set in SR. */
			ptSpiHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_TXEIE);

			ptSpiHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_TXEIE);
		}
	}

	return;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_RxDataIT(SPI_Handle_t* ptSpiHandle,
				  uint8_t* pucRxBuff, uint16_t uhRxSize)
{
	/* Validity check. */
	if((NULL != ptSpiHandle)
	&& (NULL != pucRxBuff)
	&& (NULL != pucRxBuff + uhRxSize))
	{
		if(ptSpiHandle->ucRxState != eSpiOpStateBsyInRx)
		{
			/* Save the TX buff address and length in some global variables. */
			ptSpiHandle->pucRxBuff = pucRxBuff;

			ptSpiHandle->uhRxLen = uhRxSize;

			/* Mark the SPI state as busy in transmission. */
			ptSpiHandle->ucRxState = eSpiOpStateBsyInRx;

			/* Enable the TXEIE control bit whenever TXE flag is set in SR. */
			ptSpiHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_RXNEIE);

			ptSpiHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_RXNEIE);
		}
	}

	return;
}


/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_IRQHandling(SPI_Handle_t* ptSpiHandle)
{
	uint8_t ucReg1 = 0;
	uint8_t ucReg2 = 0;

	if(NULL != ptSpiHandle)
	{
		/* First check the TXE bit. */
		ucReg1 = (ptSpiHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE));
		ucReg2 = (ptSpiHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE));

		if(ucReg1 && ucReg2)
		{
			/* Handle TX interrupt. */
			HandleSpiTxeInt(ptSpiHandle);

		}

		/* Check the RXNE bit. */
		ucReg1 = (ptSpiHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE));
		ucReg2 = (ptSpiHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE));

		if(ucReg1 && ucReg2)
		{
			/* Handle RX interrupt. */
			HandleSpiRxneInt(ptSpiHandle);
		}

		/* Check for OVR flag. */
		ucReg1 = ptSpiHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_OVR);
		ucReg2 = (ptSpiHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE));

		if(ucReg1 && ucReg2)
		{
			/* Handle OVR error. */
			HandleSpiOvrInt(ptSpiHandle);
		}
	}

	return;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
static void HandleSpiTxeInt(SPI_Handle_t* ptSpiHandle)
{
	if(NULL != ptSpiHandle)
	{
		/* Load the data register with the value from tx buffer. */
		ptSpiHandle->pSPIx->SPI_DR = *(ptSpiHandle->pucTxBuff);

		/* Increment the pointer. */
		ptSpiHandle->pucTxBuff += 1;

		/* Decrement the length. */
		ptSpiHandle->uhTxLen -= 1;

		if(!ptSpiHandle->uhTxLen)
		{
			/* Close the SPI communication. */
			ptSpiHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);

			/* Reset the buffers. */
			ptSpiHandle->pucTxBuff = NULL;

			/* Update the TX state. */
			ptSpiHandle->ucTxState = eSpiOpStateRdy;

			SpiApplicationEvntCallBack(ptSpiHandle, SPI_EVENT_TX_CMPLT);

		}

	}
	return;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
static void HandleSpiRxneInt(SPI_Handle_t* ptSpiHandle)
{
	if(NULL != ptSpiHandle)
	{
		/* Load the data register with the value from tx buffer. */
		*(ptSpiHandle->pucRxBuff) = (uint8_t)ptSpiHandle->pSPIx->SPI_DR;

		/* Increment the pointer. */
		ptSpiHandle->pucRxBuff += 1;

		/* Decrement the length. */
		ptSpiHandle->uhRxLen -= 1;

		if(!ptSpiHandle->uhRxLen)
		{
			/* Close the SPI communication. */
			ptSpiHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);

			/* Reset the buffers. */
			ptSpiHandle->pucRxBuff = NULL;

			/* Update the TX state. */
			ptSpiHandle->ucTxState = eSpiOpStateRdy;

			SpiApplicationEvntCallBack(ptSpiHandle, SPI_EVENT_RX_CMPLT);

		}

	}
	return;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
static void HandleSpiOvrInt(SPI_Handle_t* ptSpiHandle)
{
	uint8_t ucTempVal = 0;

	/* Clear the OVR flag. */
	if(ptSpiHandle->ucTxState != eSpiOpStateBsyInTx)
	{
		ucTempVal = ptSpiHandle->pSPIx->SPI_DR;

		ucTempVal = ptSpiHandle->pSPIx->SPI_SR;
	}

	/* Inform the application. */
	SpiApplicationEvntCallBack(ptSpiHandle, SPI_EVENT_OVR_ERR);

	(void)ucTempVal;

	return;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_ClearOvrFlag(SPI_Regdef_t* pSPIx)
{
	if(NULL != pSPIx)
	{
		pSPIx->SPI_SR &= ~(1 << SPI_SR_OVR);
	}

	return;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_CloseTransmission(SPI_Handle_t* ptSpiHandle)
{
	if(NULL != ptSpiHandle)
	{
		/* Close the SPI communication. */
		ptSpiHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);

		/* Reset the buffers. */
		ptSpiHandle->pucTxBuff = NULL;

		/* Update the TX state. */
		ptSpiHandle->ucTxState = eSpiOpStateRdy;

		ptSpiHandle->uhTxLen = 0;
	}

	return;
}

/******************************************************************************
 * @brief : Function For Clearing interruot pending request.
 * @fn : GPIO_IRQHandling(SPI_Regdef_t* pSPIx)
 *
 * @param[in] : ptSpiHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SPI_CloseReception(SPI_Handle_t* ptSpiHandle)
{
	if(NULL != ptSpiHandle)
	{
		/* Close the SPI communication. */
		ptSpiHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);

		/* Reset the buffers. */
		ptSpiHandle->pucRxBuff = NULL;

		/* Update the TX state. */
		ptSpiHandle->ucTxState = eSpiOpStateRdy;

		ptSpiHandle->uhRxLen = 0;
	}

	return;
}

void SPI_TransmitReceiveIT(SPI_Handle_t* ptSpiHandle,
        uint8_t* pucTxBuff, uint8_t* pucRxBuff, uint16_t uhSize)
{
	if((NULL != ptSpiHandle) && (NULL != pucTxBuff) && (NULL != pucRxBuff))
	{
		ptSpiHandle->pucTxBuff = pucTxBuff;
		ptSpiHandle->pucRxBuff = pucRxBuff;
		ptSpiHandle->uhTxLen = uhSize;
		ptSpiHandle->uhRxLen = uhSize;

		ptSpiHandle->ucTxState = eSpiOpStateBsyInTx;
		ptSpiHandle->ucRxState = eSpiOpStateBsyInRx;

		// Enable both interrupts
		ptSpiHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE) | (1 << SPI_CR2_RXNEIE);

		// Kickstart transmission by writing first byte
		if(uhSize > 0)
		{
			ptSpiHandle->pSPIx->SPI_DR = *pucTxBuff++;
			ptSpiHandle->uhTxLen--;
		}
	}
}

static void SpiApplicationEvntCallBack(SPI_Handle_t* ptSpiHandle, uint8_t ucEvnt)
{
	return;
}
