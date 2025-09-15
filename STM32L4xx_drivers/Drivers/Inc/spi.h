/******************************************************************************
 *		Header File Containing SPI related Informations and
 *		Drivers (API's) Related SPI Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#ifndef INC_SPI_H_
#define INC_SPI_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l476xx.h"

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/
/*
 * @brief : Configuration Structure for SPI.
 */
typedef struct _SPI_Config_t_
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;


/*
 * @brief : Handle Structure for SPI.
 */
typedef struct _SPI_Handle_t_
{
	SPI_Regdef_t* pSPIx;
	SPI_Config_t  SPIConfig;
} SPI_Handle_t;

/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/
typedef enum _SPI_CpolState_
{
	eSPICpolState0 = 0,
	eSPICpolStete1,
	eSPICpolStateMax
} eSPI_CpolState;

typedef enum _SPI_CphaseState_
{
	eSPICphaseState0 = 0,
	eSPICphaseStete1,
	eSPICphaseStateMax
} eSPI_CphaseState;

typedef enum _SPI_Mode_
{
	eSPIModeSlave = 0,
	eSPIModeMaster,
	eSPIModeMax
} eSPI_Mode;

typedef enum _SPI_BusConfig_
{
	eSPIBusConfigFullDuplex = 1,
	eSPIBusConfigHalfDuplex,
	eSPIBusConfigSimplexRxOnly,
	eSPIBusConfigMax,
} eSPI_BusConfig;

typedef enum _SPI_SclkSpeed_
{
	eSpiSclkDiv2 = 0,
	eSpiSclkDiv4,
	eSpiSclkDiv8,
	eSpiSclkDiv16,
	eSpiSclkDiv32,
	eSpiSclkDiv64,
	eSpiSclkDiv128,
	eSpiSclkDiv256,
	eSpiSclkDivmax
} eSPI_SclkSpeed;

typedef enum _SPI_Dff_
{
	eSPIDff8Bits  = 7,
	eSPIDff9Bits  = 8,
	eSPIDff10Bits = 9,
	eSPIDff11Bits = 10,
	eSPIDff12Bits = 11,
	eSPIDff13Bits = 12,
	eSPIDff14Bits = 13,
	eSPIDff15Bits = 14,
	eSPIDff16Bits = 15,
	eSPIDffMax,
} eSPI_Dff;

typedef enum _SPI_SSM_
{
	eSsmSwDi = 0,
	eSsmSwEn,
	eSsmMax,
} eSPI_SSM;

/******************************************************************************
 * Function Declarations.
 *****************************************************************************/
void SPI_Init(SPI_Handle_t* ptSpiHandle);
void SPI_DeInit(SPI_Regdef_t* pSPIx);
void SPI_Enable(SPI_Regdef_t* pSPIx, uint8_t ucEnorDi);
void SPI_SsiConfig(SPI_Regdef_t* pSPIx, uint8_t ucEnorDi);
void SPI_PclkCtrl(SPI_Regdef_t* pSPIx,uint8_t EnorDi);
void SPI_TxData(SPI_Regdef_t* pSPIx, uint8_t* ucTxBuff, uint16_t uhTxSize);
void SPI_RxData(SPI_Regdef_t* pSPIx, uint8_t* ucRxBuff, uint16_t uhRxSize);
void SPI_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal);
void SPI_IRQHandling(SPI_Regdef_t* pSPIx);
bool IsSpiBusy(SPI_Regdef_t* pSPIx);
void SPI_TxRxData(SPI_Regdef_t* pSPIx, uint8_t* ucTxBuff,
		uint8_t* ucRxBuff, uint16_t uhSize);

#endif /* INC_SPI_H_ */
