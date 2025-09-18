/******************************************************************************
 *		Source File Containing UART related Informations and
 *		Drivers (API's) Related UART Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "uart.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
/* USART CR1 Bit positions. */
#define USART_CR1_UE		(0)
#define USART_CR1_UESM		(1)
#define USART_CR1_RE		(2)
#define USART_CR1_TE		(3)
#define USART_CR1_IDLEIE	(4)
#define USART_CR1_RXNEIE	(5)
#define USART_CR1_TCIE		(6)
#define USART_CR1_TXEIE		(7)
#define USART_CR1_PEIE		(8)
#define USART_CR1_PS		(9)
#define USART_CR1_PCE		(10)
#define USART_CR1_WAKE		(11)
#define USART_CR1_M0		(12)
#define USART_CR1_MME		(13)
#define USART_CR1_CMIE		(14)
#define USART_CR1_OVER8		(15)
#define USART_CR1_DEDT		(16)
#define USART_CR1_DEAT		(21)
#define USART_CR1_RTOIE		(26)
#define USART_CR1_EOBIE		(27)
#define USART_CR1_M1		(28)

/* USART CR2 Bit positions. */
#define USART_CR2_ADDM7		(4)
#define USART_CR2_LBDL		(5)
#define USART_CR2_LBDIE		(6)
#define USART_CR2_LBCL		(8)
#define USART_CR2_CPHA		(9)
#define USART_CR2_CPOL		(10)
#define USART_CR2_CLKEN		(11)
#define USART_CR2_STOP		(12)
#define USART_CR2_LINEN		(14)
#define USART_CR2_SWAP		(15)
#define USART_CR2_RXINV		(16)
#define USART_CR2_TXINV		(17)
#define USART_CR2_DATAINV	(18)
#define USART_CR2_MSBFIRST	(19)
#define USART_CR2_ABREN		(20)
#define USART_CR2_ABRMOD	(21)
#define USART_CR2_RTOEN		(23)

/* USART CR3 Bit positions. */
#define USART_CR3_EIE		(0)
#define USART_CR3_IREN		(1)
#define USART_CR3_IRLP		(2)
#define USART_CR3_HDSEL		(3)
#define USART_CR3_NACK		(4)
#define USART_CR3_SCEN		(5)
#define USART_CR3_DMAR		(6)
#define USART_CR3_DMAT		(7)
#define USART_CR3_RTSE		(8)
#define USART_CR3_CTSE		(9)
#define USART_CR3_CTSIE		(10)
#define USART_CR3_ONEBIT	(11)
#define USART_CR3_OVRDIS	(12)
#define USART_CR3_DDRE		(13)
#define USART_CR3_DEM		(14)
#define USART_CR3_DEP		(15)
#define USART_CR3_SCARCNT0	(17)
#define USART_CR3_SCARCNT1	(18)
#define USART_CR3_SCARCNT2	(19)
#define USART_CR3_WUS0		(20)
#define USART_CR3_WUS1		(21)
#define USART_CR3_WUFIE		(22)
#define USART_CR3_UCESM		(23)
#define USART_CR3_TCBGTIE	(24)

/* USART RQR Bit positions. */
#define USART_RQR_ABRRQ		(0)
#define USART_RQR_SBKRQ		(1)
#define USART_RQR_MMRQ		(2)
#define USART_RQR_RXFRQ		(3)
#define USART_RQR_TXFRQ		(4)

/* USART ISR Bit positions. */
#define USART_ISR_PE		(0)
#define USART_ISR_FE		(1)
#define USART_ISR_NF		(2)
#define USART_ISR_ORE		(3)
#define USART_ISR_IDLE		(4)
#define USART_ISR_RXNE		(5)
#define USART_ISR_TC		(6)
#define USART_ISR_TXE		(7)
#define USART_ISR_LBDF		(8)
#define USART_ISR_CTSIF		(9)
#define USART_ISR_CTS		(10)
#define USART_ISR_RTOF		(11)
#define USART_ISR_EOBF		(12)
#define USART_ISR_ABRE		(14)
#define USART_ISR_ABRF		(15)
#define USART_ISR_BUSY		(16)
#define USART_ISR_CMF		(17)
#define USART_ISR_SBKF		(18)
#define USART_ISR_RWU		(19)
#define USART_ISR_WUF		(20)
#define USART_ISR_TEACK		(21)
#define USART_ISR_REACK		(22)
#define USART_ISR_TCBGT		(25)

/* USART ICR Bit positions. */
#define USART_ICR_PECF		(0)
#define USART_ICR_FECF		(1)
#define USART_ICR_NCF		(2)
#define USART_ICR_ORECF		(3)
#define USART_ICR_IDLECF	(4)
#define USART_ICR_TCCF		(6)
#define USART_ICR_TCBGTCF	(7)
#define USART_ICR_LBDCF		(8)
#define USART_ICR_CTSCF		(9)
#define USART_ICR_RTOCF		(11)
#define USART_ICR_EOBCF		(12)
#define USART_ICR_CMCF		(17)
#define USART_ICR_WUCF		(20)

/* USART Reset macros. */
#define USART1_REG_RESET()		do{(RCC->RCC_APB2RSTR  |= (1 << 14)); (RCC->RCC_APB2RSTR  &= ~(1 << 14));}while(0)
#define USART2_REG_RESET()		do{(RCC->RCC_APB1RSTR1 |= (1 << 17)); (RCC->RCC_APB1RSTR1 &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()		do{(RCC->RCC_APB1RSTR1 |= (1 << 18)); (RCC->RCC_APB1RSTR1 &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()		do{(RCC->RCC_APB1RSTR1 |= (1 << 19)); (RCC->RCC_APB1RSTR1 &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()		do{(RCC->RCC_APB1RSTR1 |= (1 << 20)); (RCC->RCC_APB1RSTR1 &= ~(1 << 20));}while(0)

/******************************************************************************
 * Static function declaration.
 *****************************************************************************/
static void USART_TxEnable(USART_Regdef_t* pUSARTx, uint8_t ucEnorDi);
static void USART_RxEnable(USART_Regdef_t* pUSARTx, uint8_t ucEnorDi);
static void SetBaudRate(USART_Regdef_t* pUSARTx, uint32_t uiBaudVal);

/******************************************************************************
 * Function Definitions.
 *****************************************************************************/

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
void USART_Init(USART_Handle_t* ptUSARTHandle)
{
	/* Validity check. */
	if(NULL != ptUSARTHandle)
	{
		/* Enable the clock for the corresponding USART Peripheral. */
		USART_PclkCtrl(ptUSARTHandle->pUSARTx, ENABLE);

		/* Configure the mode. */
		switch(ptUSARTHandle->USARTConfig.USART_Mode)
		{
			case eUsartModeTx:
			{
				USART_TxEnable(ptUSARTHandle->pUSARTx, ENABLE);

				break;
			}

			case eUsartModeRx:
			{
				USART_RxEnable(ptUSARTHandle->pUSARTx, ENABLE);

				break;
			}

			case eUsartModeTxRx:
			{
				USART_TxEnable(ptUSARTHandle->pUSARTx, ENABLE);

				USART_RxEnable(ptUSARTHandle->pUSARTx, ENABLE);

				break;
			}

			default:
			{
				break;
			}
		}

		/* Parity Control. */
		if(ptUSARTHandle->USARTConfig.USART_ParityControl >= eUsartParityNone
		&& ptUSARTHandle->USARTConfig.USART_ParityControl < eUsartParityMax)
		{
			if(eUsartParityEvn == ptUSARTHandle->USARTConfig.USART_ParityControl)
			{
				/* Enable Parity control bit. */
				ptUSARTHandle->pUSARTx->USART_CR1 |= ( 1 << USART_CR1_PCE);

				/* Enable parity selection as even. */
				ptUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_PS);
			}

			else if(eUsartParityOdd == ptUSARTHandle->USARTConfig.USART_ParityControl)
			{
				/* Enable Parity control bit. */
				ptUSARTHandle->pUSARTx->USART_CR1 |= ( 1 << USART_CR1_PCE);

				/* Enable parity selection as odd. */
				ptUSARTHandle->pUSARTx->USART_CR1 |= ( 1 << USART_CR1_PS);
			}

			else if(eUsartParityNone == ptUSARTHandle->USARTConfig.USART_ParityControl)
			{
				/* Disable the parity control bit. */
				ptUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_PCE);
			}
		}

		/* Configure word length. */
		if(ptUSARTHandle->USARTConfig.USART_WordLength >= eUsartWordLen8Bits
		&& ptUSARTHandle->USARTConfig.USART_WordLength < eUsartWordLenMax)
		{
			if(eUsartWordLen8Bits == ptUSARTHandle->USARTConfig.USART_WordLength)
			{
				ptUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_M0);

				ptUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_M1);
			}

			else if(eUsartWordLen9Bits == ptUSARTHandle->USARTConfig.USART_WordLength)
			{
				ptUSARTHandle->pUSARTx->USART_CR1 |= ( 1 << USART_CR1_M0);

				ptUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_M1);
			}
		}

		/* Configure Stop bits. */
		if(ptUSARTHandle->USARTConfig.USART_NoOfStopBits >= eUsartStopBit1
		&& ptUSARTHandle->USARTConfig.USART_NoOfStopBits < eUsartStopBitMax)
		{
			/* Reset the actual register. */
			ptUSARTHandle->pUSARTx->USART_CR2 &= ~(0x03 << USART_CR2_STOP);

			/* Set the actual register. */
			ptUSARTHandle->pUSARTx->USART_CR2 |= (ptUSARTHandle->USARTConfig.USART_NoOfStopBits <<
												  USART_CR2_STOP);
		}

		/* Configure hardware flow control. */
		if(ptUSARTHandle->USARTConfig.USART_HwFlowControl >= eUsartHwFlwCtrlNone
		&& ptUSARTHandle->USARTConfig.USART_HwFlowControl < eUsartHwFlwCtrlMax)
		{
			if(eUsartHwFlwCtrlNone == ptUSARTHandle->USARTConfig.USART_HwFlowControl)
			{
				/* Reset both CTSE and RTSE. */
				ptUSARTHandle->pUSARTx->USART_CR3 &= ~(0x02 << USART_CR3_RTSE);
			}

			else if(eUsartHwFlwCtrlCts == ptUSARTHandle->USARTConfig.USART_HwFlowControl)
			{
				/* SET CTSE. */
				ptUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_CTSE);
			}

			else if(eUsartHwFlwCtrlRts == ptUSARTHandle->USARTConfig.USART_HwFlowControl)
			{
				/* Set RTSE. */
				ptUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_RTSE);
			}

			else if(eUsartHwFlwCtrlCtsRts == ptUSARTHandle->USARTConfig.USART_HwFlowControl)
			{
				/* Set both CTSE and RTSE. */
				ptUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_CTSE);

				ptUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_RTSE);
			}
		}

		/* Set USART baud rate. */
		SetBaudRate(ptUSARTHandle->pUSARTx, ptUSARTHandle->USARTConfig.USART_Baud);

	}
}

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
void USART_DeInit(USART_Regdef_t* pUSARTx)
{
	/* Validity check. */
	if(NULL != pUSARTx)
	{
		if(USART1 == pUSARTx)
		{
			USART1_REG_RESET();
		}

		else if(USART2 == pUSARTx)
		{
			USART2_REG_RESET();
		}

		else if(USART3 == pUSARTx)
		{
			USART3_REG_RESET();
		}

		else if(UART4 == pUSARTx)
		{
			UART4_REG_RESET();
		}

		else if(UART5 == pUSARTx)
		{
			UART5_REG_RESET();
		}
	}

	return;
}

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
static void USART_TxEnable(USART_Regdef_t* pUSARTx, uint8_t ucEnorDi)
{
	/* Validity check. */
	if(NULL != pUSARTx)
	{
		/* Enable. */
		if(ENABLE == ucEnorDi)
		{
			pUSARTx->USART_CR1 |= (1 << USART_CR1_TE);
		}

		/* Disable. */
		else if(DISABLE == ucEnorDi)
		{
			pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TE);
		}
	}

	return;
}

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
static void USART_RxEnable(USART_Regdef_t* pUSARTx, uint8_t ucEnorDi)
{
	/* Validity check. */
	if(NULL != pUSARTx)
	{
		/* Enable. */
		if(ENABLE == ucEnorDi)
		{
			pUSARTx->USART_CR1 |= (1 << USART_CR1_RE);
		}

		/* Disable. */
		else if(DISABLE == ucEnorDi)
		{
			pUSARTx->USART_CR1 &= ~(1 << USART_CR1_RE);
		}
	}

	return;
}

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
static void SetBaudRate(USART_Regdef_t* pUSARTx, uint32_t uiBaudVal)
{
	/* Parameter initialization. */
	uint32_t uiPclkVal = 2000000;
	uint32_t uiUsartDiv = 0;

	/* Parameter validity check. */
	if(NULL != pUSARTx)
	{
		/* Check the sampling rate. */
		if(pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8))
		{
			/* Over sampling 16. */
			uiUsartDiv = uiPclkVal / uiBaudVal;
		}
		else
		{
			/* Over sampling 8. */
			uiUsartDiv = (2 * uiPclkVal) / uiBaudVal;
		}

		/* Load the BRR register. */
		pUSARTx->USART_BRR = uiUsartDiv;
	}

	return;
}

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
void USART_PclkCtrl(USART_Regdef_t* pUSARTx,uint8_t EnorDi)
{
	if(ENABLE == EnorDi)
	{
		if(USART1 == pUSARTx)
		{
			USART1_PCLK_EN();
		}
		else if(USART2 == pUSARTx)
		{
			USART2_PCLK_EN();
		}
		else if(USART3 == pUSARTx)
		{
			USART3_PCLK_EN();
		}
		else if(UART4 == pUSARTx)
		{
			UART4_PCLK_EN();
		}
		else if(UART5 == pUSARTx)
		{
			UART5_PCLK_EN();
		}
	}
	else if(DISABLE == EnorDi)
	{
		if(USART1 == pUSARTx)
		{
			USART1_PCLK_DI();
		}
		else if(USART2 == pUSARTx)
		{
			USART2_PCLK_DI();
		}
		else if(USART3 == pUSARTx)
		{
			USART3_PCLK_DI();
		}
		else if(UART4 == pUSARTx)
		{
			UART4_PCLK_DI();
		}
		else if(UART5 == pUSARTx)
		{
			UART5_PCLK_DI();
		}
	}
	return;
}

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
void USART_PeripheralControl(USART_Regdef_t* pUSARTx,uint8_t EnorDi)
{
	/* Validity check. */
	if(NULL != pUSARTx)
	{
		if(ENABLE == EnorDi)
		{
			/* Enable the UE bit of CR1. */
			pUSARTx->USART_CR1 |= ( 1 << USART_CR1_UE);
		}
		else if(DISABLE == EnorDi)
		{
			/* Disable the UE bit of CR1. */
			pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_UE);
		}
	}

	return;
}

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
void USART_TxData(USART_Regdef_t* pUSARTx, uint8_t* ucTxBuff, uint16_t uhTxSize)
{
	/* Parameter validity check. */
	if((NULL != pUSARTx)
	&& (NULL != ucTxBuff)
	&& (NULL != ucTxBuff + uhTxSize))
	{
		for(uint32_t uiIdx = 0; uiIdx < uhTxSize; ++uiIdx)
		{
			/* Wait if the transmission is busy. */
			while(IsUSARTBusyTx(pUSARTx));

			/* Load the data to transmit data register. */
			pUSARTx->USART_TDR = ucTxBuff[uiIdx];
		}

		/* Wait until the transmission is completed. */
		while(!(pUSARTx->USART_ISR & ( 1 << USART_ISR_TC)));
	}
}

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
void USART_RxData(USART_Regdef_t* pUSARTx, uint8_t* ucRxBuff, uint16_t uhRxSize)
{

}

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
bool IsUSARTBusyTx(USART_Regdef_t* pUSARTx)
{
	bool BusyTx = true;

	/* Validity check. */
	if(NULL != pUSARTx)
	{
		/* If txe is empty. */
		if(pUSARTx->USART_ISR & ( 1 << USART_ISR_TXE))
		{
			/* Not busy anymore. */
			BusyTx = false;
		}
	}

	return BusyTx;
}

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
bool IsUSARTBusyRx(USART_Regdef_t* pUSARTx)
{
	return 0;
}
