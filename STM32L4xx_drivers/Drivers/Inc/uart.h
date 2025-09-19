/******************************************************************************
 *		Header File Containing UART related Informations and
 *		Drivers (API's) Related UART Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l476xx.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
#define USART_BAUD_1200				(1200)
#define USART_BAUD_2400				(2400)
#define USART_BAUD_9600				(9600)
#define USART_BAUD_19200			(19200)
#define USART_BAUD_38400			(38400)
#define USART_BAUD_57600			(57600)
#define USART_BAUD_115200			(115200)
#define USART_BAUD_230400			(230400)
#define USART_BAUD_460800			(460800)
#define USART_BAUD_921600			(921600)
#define USART_BAUD_2M				(2000000)
#define USART_BAUD_3M				(3000000)

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/
/*
 * @brief : Configuration Structure for USART.
 */
typedef struct _USART_Config_t_
{
	uint8_t  USART_Mode;
	uint32_t USART_Baud;
	uint8_t  USART_NoOfStopBits;
	uint8_t  USART_WordLength;
	uint8_t  USART_ParityControl;
	uint8_t  USART_HwFlowControl;
} USART_Config_t;

/*
 * @brief : Handle Structure for USART.
 */
typedef struct _USART_Handle_t_
{
	USART_Regdef_t* pUSARTx;
	USART_Config_t USARTConfig;
	uint8_t* pucTxBuffer;
	uint8_t* pucRxBuffer;
	uint16_t uhTxLen;
	uint16_t uhRxLen;
	uint8_t ucTxBsyState;
	uint8_t ucRxBsyState;
} USART_Handle_t;

/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/
/*
 * @brief : Enum representing modes USART.
 */
typedef enum _eUsart_Mode_
{
	eUsartModeTx = 0,
	eUsartModeRx,
	eUsartModeTxRx,
	eUsartModeMax
} eUsart_Mode;

/*
 * @brief : Enum representing parity USART.
 */
typedef enum _eUsart_Parity_
{
	eUsartParityNone = 0,
	eUsartParityEvn,
	eUsartParityOdd ,
	eUsartParityMax
} eUsart_Parity;

/*
 * @brief : Enum representing stop bit for USART.
 */
typedef enum _eUsart_StopBits_
{
	eUsartStopBit1 = 0,
	eUsartStopBit0_5,
	eUsartStopBit2,
	eUsartStopBit1_5,
	eUsartStopBitMax,
} eUsart_StopBits;

/*
 * @brief : Enum representing H/W flow control for USART.
 */
typedef enum _eUsart_HwFlwCtrl_
{
	eUsartHwFlwCtrlNone = 0,
	eUsartHwFlwCtrlCts,
	eUsartHwFlwCtrlRts,
	eUsartHwFlwCtrlCtsRts,
	eUsartHwFlwCtrlMax
} eUsart_HwFlwCtrl;

/*
 * @brief : Enum representing word length for USART.
 */
typedef enum _eUsart_WrdLength_
{
	eUsartWordLen8Bits = 0,
	eUsartWordLen9Bits,
	eUsartWordLenMax
} eUsart_WrdLength;

/*
 * @brief : Enum representing state of USART RX and TX.
 */
typedef enum _eUsart_State_
{
	eUsartStateReady = 0,
	eUsartStateBsyTx,
	eUsartStateBsyRx,
	eUsartStateMax,
} eUsart_State;


/******************************************************************************
 * Function Declarations.
 *****************************************************************************/
void USART_Init(USART_Handle_t* ptUSARTHandle);
void USART_DeInit(USART_Regdef_t* pUSARTx);
void USART_PclkCtrl(USART_Regdef_t* pUSARTx,uint8_t EnorDi);
void USART_PeripheralControl(USART_Regdef_t* pUSARTx,uint8_t EnorDi);
void USART_TxData(USART_Regdef_t* pUSARTx, uint8_t* ucTxBuff, uint16_t uhTxSize);
void USART_RxData(USART_Regdef_t* pUSARTx, uint8_t* ucRxBuff, uint16_t uhRxSize);

bool IsUSARTBusyTx(USART_Regdef_t* pUSARTx);
bool IsUSARTRxDataAvailable(USART_Regdef_t* pUSARTx);
void USART_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi);

void USART_IRQPriorityConfig(uint8_t ucIRQNumber, uint32_t IRQPriorityVal);
void USART_IRQHandling(USART_Handle_t* ptUSARTHandle);
void USART_TxDataIT(USART_Handle_t* ptUSARTHandle,
					uint8_t* ucTxBuff, uint16_t uhTxSize);
void USART_RxDataIT(USART_Handle_t* ptUSARTHandle,
					uint8_t* ucRxBuff, uint16_t uhRxSize);

#endif /* INC_UART_H_ */
