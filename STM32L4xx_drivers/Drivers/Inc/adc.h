/******************************************************************************
 *		Header File Containing I2C related Informations and
 *		Drivers (API's) Related I2C Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l476xx.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/

/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/
/*
 * @brief: Enum representing ADC resolution configuration.
 */
typedef enum _eAdcResolutionConfig_
{
	eAdcResln12bit = 0,
	eAdcResln10bit,
	eAdcResln08bit,
	eAdcResln06bit,
	eAdcReslnmax
} eAdcResolutionConfig;

/*
 * @brief: Enum representing ADC data alignment.
 */
typedef enum _eAdcDataAlingmentConfig_
{
	eAdcDataAlignRight = 0,
	eAdcDataAlignLeft,
	eAdcDataAlignmax
} eAdcDataAlingmentConfig;

/*
 * @brief: Enum representing ADC data conversion mode.
 */
typedef enum _eAdcConvModeConfig_
{
	eAdcSingleConvMode = 0,
	eAdcContinousConvMode,
	eAdcConvModeMax
} eAdcConvModeConfig;

/*
 * @brief: Enum representing ADC data conversion mode.
 */
typedef enum _eAdcSamplingTimeConfig_
{
	eAdcSamplingTime2_5Clk = 0,
	eAdcSamplingTime6_5Clk,
	eAdcSamplingTime12_5Clk,
	eAdcSamplingTime24_5Clk,
	eAdcSamplingTime47_5Clk,
	eAdcSamplingTime92_5Clk,
	eAdcSamplingTime247_5Clk,
	eAdcSamplingTime640_5Clk,
	eAdcSamplingTimeMax,
} eAdcSamplingTimeConfig;

/*
 * @brief: Enum representing ADC regular channel sequence length config.
 */
typedef enum _eAdcSeqLengthConfig_
{
	eSeqLen1Conversions = 0,
	eSeqLen2Conversions,
	eSeqLen3Conversions,
	eSeqLen4Conversions,
	eSeqLen5Conversions,
	eSeqLen6Conversions,
	eSeqLen7Conversions,
	eSeqLen8Conversions,
	eSeqLen9Conversions,
	eSeqLen10Conversions,
	eSeqLen11Conversions,
	eSeqLen12Conversions,
	eSeqLen13Conversions,
	eSeqLen14Conversions,
	eSeqLen15Conversions,
	eSeqLen16Conversions,
	eSeqLenConversionsMax,
} eAdcSeqLengthConfig;

/*
 * @brief: Enum representing ADC channel id's.
 */
typedef enum _eAdcChannelId_
{
	eAdcChannelId0 = 0,
	eAdcChannelId1,
	eAdcChannelId2,
	eAdcChannelId3,
	eAdcChannelId4,
	eAdcChannelId5,
	eAdcChannelId6,
	eAdcChannelId7,
	eAdcChannelId8,
	eAdcChannelId9,
	eAdcChannelId10,
	eAdcChannelId11,
	eAdcChannelId12,
	eAdcChannelId13,
	eAdcChannelId14,
	eAdcChannelId15,
	eAdcChannelId16,
	eAdcChannelId17,
	eAdcChannelId18,
	eAdcChannelIdMax
} eAdcChannelId;

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/
/*
 * @brief : Configuration Structure for ADC.
 */
typedef struct _ADC_Config_t_
{
	uint8_t ucAdcReslnCnfg;
	uint8_t ucAdcDataAllingment;
	uint8_t ucAdcConvMode;
	uint8_t ucAdcSamplingTime;
	uint8_t ucAdcChannelId;
} ADC_Config_t;

/*
 * @brief : Handle Structure for ADC.
 */
typedef struct _ADC_Handle_t_
{
	ADC_Regdef_t* pADC;
	ADC_Config_t  ADCConfig;
} ADC_Handle_t;

/******************************************************************************
 * Function Declarations.
 *****************************************************************************/
void AdcInit(ADC_Handle_t* ptAdcHandle);
void AdcCalibrate(ADC_Regdef_t* pADC);
void AdcEnable(ADC_Regdef_t* pADC);
uint16_t AdcReadChannel(ADC_Regdef_t* pADC, uint8_t ucChannel);


#endif /* INC_ADC_H_ */
