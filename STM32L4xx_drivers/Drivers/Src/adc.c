/******************************************************************************
 *		Source File Containing ADC related Informations and
 *		Drivers (API's) Related ADC Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "adc.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
/* ADC ISR Bit positions. */
#define ADC_ISR_ADRDY					(0x00)
#define ADC_ISR_EOSMP					(0x01)
#define ADC_ISR_EOC						(0x02)
#define ADC_ISR_EOS						(0x03)
#define ADC_ISR_OVR						(0x04)
#define ADC_ISR_JEOC					(0x05)
#define ADC_ISR_JEOS					(0x06)
#define ADC_ISR_AWD1					(0x07)
#define ADC_ISR_AWD2					(0x08)
#define ADC_ISR_AWD3					(0x09)
#define ADC_ISR_JQOVF					(0x0A)

/* ADC IER Bit positions. */
#define ADC_IER_ADRDYIE					(0x00)
#define ADC_IER_EOSMPIE					(0x01)
#define ADC_IER_EOCIE					(0x02)
#define ADC_IER_EOSIE					(0x03)
#define ADC_IER_OVRIE					(0x04)
#define ADC_IER_JEOCIE					(0x05)
#define ADC_IER_JEOSIE					(0x06)
#define ADC_IER_AWD1IE					(0x07)
#define ADC_IER_AWD2IE					(0x08)
#define ADC_IER_AWD3IE					(0x09)
#define ADC_IER_JQOVFIE					(0x0A)

/* ADC CR Bit positions. */
#define ADC_CR_ADEN						(0x00)
#define ADC_CR_ADDIS					(0x01)
#define ADC_CR_ADSTART					(0x02)
#define ADC_CR_JADSTART					(0x03)
#define ADC_CR_ADSTP					(0x04)
#define ADC_CR_JADSTP					(0x05)
#define ADC_CR_ADVREGEN					(0x1C)
#define ADC_CR_DEEPPWD					(0x1D)
#define ADC_CR_ADCALDIF					(0x1E)
#define ADC_CR_ADCAL					(0x1F)

/* ADC CFGR Bit positions. */
#define ADC_CFGR_DMAEN					(0x00)
#define ADC_CFGR_DMACFG					(0x01)
#define ADC_CFGR_DFSDMCFG				(0x02)
#define ADC_CFGR_RES					(0x03)
#define ADC_CFGR_ALIGN					(0x05)
#define ADC_CFGR_EXTSEL0				(0x06)
#define ADC_CFGR_EXTSEL1				(0x07)
#define ADC_CFGR_EXTSEL2				(0x08)
#define ADC_CFGR_EXTSEL3				(0x09)
#define ADC_CFGR_EXTEN					(0x0A)
#define ADC_CFGR_OVRMOD					(0x0C)
#define ADC_CFGR_CONT					(0x0D)
#define ADC_CFGR_AUTDLY					(0x0E)
#define ADC_CFGR_DISCEN					(0x10)
#define ADC_CFGR_DISCNUM				(0x11)
#define ADC_CFGR_JDISCEN				(0x14)
#define ADC_CFGR_JQM					(0x15)
#define ADC_CFGR_AWD1SGL				(0x16)
#define ADC_CFGR_AWD1EN					(0x17)
#define ADC_CFGR_JAWD1EN				(0x18)
#define ADC_CFGR_JAUTO					(0x19)
#define ADC_CFGR_AWD1CH					(0x1A)
#define ADC_CFGR_JQDIS					(0x1F)

/* ADC CFGR2 Bit positions. */
#define ADC_CFGR2_ROVSE					(0x00)
#define ADC_CFGR2_JOVSE					(0x01)
#define ADC_CFGR2_OVSR					(0x02)
#define ADC_CFGR2_OVSS					(0x05)
#define ADC_CFGR2_TROVS					(0x09)
#define ADC_CFGR2_ROVSM					(0x0A)

/******************************************************************************
 * Static function declaration.
 *****************************************************************************/
static void ADCPeripheralClockControl(uint8_t ucEnOrDi);
/******************************************************************************
 * Function Definitions.
 *****************************************************************************/

/******************************************************************************
 * @brief : Function For Initializing ADC Peripheral.
 * @fn : AdcInit(ADC_Handle_t* ptAdcHandle)
 *
 * @param[in] : ptAdcHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void AdcInit(ADC_Handle_t* ptAdcHandle)
{
	/* Variable initialization. */
	uint8_t ucChannelId = 0;

	/* Validity check. */
	if(NULL != ptAdcHandle)
	{
		/* Enable Clock for ADC peripheral. */
		ADCPeripheralClockControl(ENABLE);

		/* Disable ADC before Configuration. */
		ptAdcHandle->pADC->ADC_CR &= ~(1 << ADC_CR_ADEN);

		/* Wait until ADC is disabled. */
		while(ptAdcHandle->pADC->ADC_CR & ( 1 << ADC_CR_ADEN));

		/* Configure Resolution bit. */
		if(ptAdcHandle->ADCConfig.ucAdcReslnCnfg >= eAdcResln12bit
		&& ptAdcHandle->ADCConfig.ucAdcReslnCnfg < eAdcReslnmax)
		{
			/* Reset the actual register before manipulation. */
			ptAdcHandle->pADC->ADC_CFGR &= ~(0x03 << ADC_CFGR_RES);

			/* Set the resolution to the actual register. */
			ptAdcHandle->pADC->ADC_CFGR |=
					(ptAdcHandle->ADCConfig.ucAdcReslnCnfg << ADC_CFGR_RES);
		}

		/* Configure the data alignment. */
		if(ptAdcHandle->ADCConfig.ucAdcDataAllingment >= eAdcDataAlignRight
		&& ptAdcHandle->ADCConfig.ucAdcDataAllingment < eAdcDataAlignmax)
		{
			/* Reset the actual register before manipulation. */
			ptAdcHandle->pADC->ADC_CFGR &= ~(0x01 << ADC_CFGR_ALIGN);

			/* Set the alignment to the actual register. */
			ptAdcHandle->pADC->ADC_CFGR |=
				(ptAdcHandle->ADCConfig.ucAdcDataAllingment << ADC_CFGR_ALIGN);
		}

		/* Configure conversion mode. */
		if(ptAdcHandle->ADCConfig.ucAdcConvMode >= eAdcSingleConvMode
		&& ptAdcHandle->ADCConfig.ucAdcConvMode < eAdcConvModeMax)
		{
			/* Reset the actual register before manipulation. */
			ptAdcHandle->pADC->ADC_CFGR &= ~(0x01 << ADC_CFGR_CONT);

			/* Set the conversion mode to actual register. */
			ptAdcHandle->pADC->ADC_CFGR |=
				(ptAdcHandle->ADCConfig.ucAdcConvMode << ADC_CFGR_CONT);
		}

		/* Configure sampling time. */
		if(ptAdcHandle->ADCConfig.ucAdcChannelId >= eAdcChannelId0
	    && ptAdcHandle->ADCConfig.ucAdcChannelId <= eAdcChannelId9)
		{
			ucChannelId = ptAdcHandle->ADCConfig.ucAdcChannelId % 10;

			/* Reset the register before configuration. */
			ptAdcHandle->pADC->ADC_SMPR1 &= (0x07 << (0x03 * ucChannelId));

			/* Configure the actual register. */
			ptAdcHandle->pADC->ADC_SMPR1 |= (ptAdcHandle->ADCConfig.ucAdcSamplingTime
											<< (0x03 * ucChannelId));
		}

		else if(ptAdcHandle->ADCConfig.ucAdcChannelId >= eAdcChannelId10
	    && ptAdcHandle->ADCConfig.ucAdcChannelId < eAdcChannelIdMax)
		{
			ucChannelId = ptAdcHandle->ADCConfig.ucAdcChannelId % 10;

			/* Reset the register before configuration. */
			ptAdcHandle->pADC->ADC_SMPR1 &= (0x07 << (0x03 * ucChannelId));

			/* Configure the actual register. */
			ptAdcHandle->pADC->ADC_SMPR1 |= (ptAdcHandle->ADCConfig.ucAdcSamplingTime
											<< (0x03 * ucChannelId));
		}

	}

	return;
}

/******************************************************************************
 * @brief : Function For Calibrating ADC Peripheral.
 * @fn : AdcCalibrate(ADC_Regdef_t* pADC)
 *
 * @param[in] : pADC
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void AdcCalibrate(ADC_Regdef_t* pADC)
{
    /* Perform Validity Check */
    if(NULL != pADC)
    {
        /* Ensure ADC is disabled */
        pADC->ADC_CR &= ~(1 << ADC_CR_ADEN);

        /* Wait until ADC is disabled */
        while(pADC->ADC_CR & (1 << ADC_CR_ADEN));

        /* Exit deep power-down mode */
        pADC->ADC_CR &= ~(1 << ADC_CR_DEEPPWD);

        /* Enable ADC internal voltage regulator */
        pADC->ADC_CR |= (1 << ADC_CR_ADVREGEN);

        /* Wait for voltage regulator startup (min 20μs) */
        /* At 4MHz, each NOP is ~0.25μs, so 80 NOPs = ~20μs */
        for(volatile int i = 0; i < 100; i++) {
            //__NOP(); // Execute No-Operation for precise delay
        }

        /* Start calibration */
        pADC->ADC_CR |= (1 << ADC_CR_ADCAL);

        /* Wait for calibration to complete */
        while(pADC->ADC_CR & (1 << ADC_CR_ADCAL));
    }

    return;
}

/******************************************************************************
 * @brief : Function For Enabling ADC Peripheral.
 * @fn : AdcCalibrate(ADC_Regdef_t* pADC)
 *
 * @param[in] : pADC
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void AdcEnable(ADC_Regdef_t* pADC)
{
	/* Validity Check. */
	if(NULL != pADC)
	{
		/* Enable ADC. */
		pADC->ADC_CR |= (1 << ADC_CR_ADEN);

		/* Wait until ADC gets ready to operate. */
		while(!(pADC->ADC_ISR & ( 1 << ADC_ISR_ADRDY)));

		/* Clear the ready flag. */
		pADC->ADC_ISR |= ( 1 << ADC_ISR_ADRDY);
	}

	return;
}

/******************************************************************************
 * @brief : Function For reading Channel of Corresponding ADC.
 * @fn : AdcReadChannel(ADC_Regdef_t* pADC, uint8_t ucChannel)
 *
 * @param[in] : pADC
 * @param[in] : ucChannel
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
uint16_t AdcReadChannel(ADC_Regdef_t* pADC, uint8_t ucChannel)
{
	/* Variable initialization. */
	uint16_t uhAdcRes = 0;

	/* Validity Check. */
	if((NULL != pADC)
	&& (ucChannel >= eAdcChannelId0 && ucChannel < eAdcChannelIdMax))
	{

		/* Configure Sequence register. */
		/* Configure length in sequence register. */
		/* Reset the length register. */
		pADC->ADC_SQR1 &= (0x04 << 0);

		/* Set as 1 conversion in sequence. */
		pADC->ADC_SQR1 |= (0x00 << 0);

		/* Reset the 1st sequence position register. */
		pADC->ADC_SQR1 &= ~(0x1F << 6);

		/* Add channel in first sequence position. */
		pADC->ADC_SQR1 |=
				(ucChannel << 6);

		/* Start ADC Conversion. */
		pADC->ADC_CR |= ( 1 << ADC_CR_ADSTART);

		/* Wait until conversion complete. */
		while(!(pADC->ADC_ISR & (1 << ADC_ISR_EOC)));

		/* Load the result from data register. */
		uhAdcRes = pADC->ADC_DR;

		/* Clear EOC flag. */
		pADC->ADC_ISR |= (1 << ADC_ISR_EOC);
	}

	/* Return the result. */
	return uhAdcRes;
}

/******************************************************************************
 * @brief : Static Function For Enabling/Disabling clock for ADC Peripheral.
 * @fn : ADCPeripheralClockControl(unit8_t ucEnOrDi)
 *
 * @param[in] : ucEnOrDi
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
static void ADCPeripheralClockControl(uint8_t ucEnOrDi)
{
	if(ENABLE == ucEnOrDi)
	{
		ADC_PCLK_EN();
	}

	else if(DISABLE == ucEnOrDi)
	{
		ADC_PCLK_DI();
	}

	return;
}

