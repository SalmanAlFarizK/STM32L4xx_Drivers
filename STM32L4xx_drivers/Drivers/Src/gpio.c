/******************************************************************************
 *		Source File Containing GPIO related Informations and
 *		Drivers (API's) Related GPIO Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "gpio.h"

/******************************************************************************
 * Function Definitions.
 *****************************************************************************/
bool isDebugPin(Gpio_RegDef_t* GPIOx, uint8_t pin)
{
    if (GPIOx == GPIOA) {
        return (pin == 13 || pin == 14 || pin == 15);
    } else if (GPIOx == GPIOB) {
        return (pin == 3 || pin == 4);
    }
    return false;
}

/******************************************************************************
 * @brief : Function For Initializing GPIO Peripheral.
 * @fn : GPIO_Init(GPIO_Handle_t* ptGpioHandle)
 *
 * @param[in] : ptGpioHandle
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_Init(GPIO_Handle_t* ptGpioHandle)
{
	uint32_t uiRegVal = 0;
	if(NULL != ptGpioHandle)
	{
		if(isDebugPin(ptGpioHandle->pGPIOx, ptGpioHandle->GpioPinConfig.GPIO_PinNumber))
		{
			return;
		}
		/* Configure the Mode of the GPIO Pin. */
		/* Mode configuration for non interrupt mode. */
		if(ptGpioHandle->GpioPinConfig.GPIO_PinMode <= eGPIO_AnalogMode)
		{
			/* Calculate the register value. */
			uiRegVal = (ptGpioHandle->GpioPinConfig.GPIO_PinMode <<
					   (2 * ptGpioHandle->GpioPinConfig.GPIO_PinNumber));

			/* CClear the actual register. */
			ptGpioHandle->pGPIOx->MODER &= ~(0x3 << (2 * ptGpioHandle->GpioPinConfig.GPIO_PinNumber));

			/* Copy the register value to actual register. */
			ptGpioHandle->pGPIOx->MODER |= uiRegVal;

			/* Reset The temporary register. */
			uiRegVal = 0;
		}

		/* Mode configuration for interrupt mode. */
		else if(ptGpioHandle->GpioPinConfig.GPIO_PinMode > eGPIO_AnalogMode &&
				ptGpioHandle->GpioPinConfig.GPIO_PinMode < eGPIO_ModeMax)
		{
            ptGpioHandle->pGPIOx->MODER &= ~(0x3 <<
            		(2 * ptGpioHandle->GpioPinConfig.GPIO_PinNumber));

			if(eGPIO_ITFeT == ptGpioHandle->GpioPinConfig.GPIO_PinMode)
			{
				/* Configure FTSR. */
				EXTI->EXTI_FTSR1 |= ( 1 << ptGpioHandle->GpioPinConfig.GPIO_PinNumber);


				/* Clear the RTSR. */
				EXTI->EXTI_RTSR1 &= ~( 1 << ptGpioHandle->GpioPinConfig.GPIO_PinNumber);
			}

			else if(eGPIO_ITReT == ptGpioHandle->GpioPinConfig.GPIO_PinMode)
			{
				/* Configure RTSR. */
				EXTI->EXTI_RTSR1 |= ( 1 << ptGpioHandle->GpioPinConfig.GPIO_PinNumber);


				/* Clear the FTSR. */
				EXTI->EXTI_FTSR1 &= ~( 1 << ptGpioHandle->GpioPinConfig.GPIO_PinNumber);
			}

			else if(eGPIO_ITReFeT == ptGpioHandle->GpioPinConfig.GPIO_PinMode)
			{
				/* COnfigure both FTSR and RTSR*/
				EXTI->EXTI_RTSR1 |= ( 1 << ptGpioHandle->GpioPinConfig.GPIO_PinNumber);

				EXTI->EXTI_FTSR1 |= ( 1 << ptGpioHandle->GpioPinConfig.GPIO_PinNumber);
			}

			/* Configure the GPIO port selection in SYSCFG_EXTICR. */
			uint8_t temp1 = ptGpioHandle->GpioPinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = ptGpioHandle->GpioPinConfig.GPIO_PinNumber % 4;
			uint8_t portCode = GPIO_BASEADDR_TO_CODE(ptGpioHandle->pGPIOx);

			SYSCFG_PCLK_EN();

			//SYSCFG->SYSCFG_EXTICR[temp1] = portCode << (temp2 * 4);
			SYSCFG->SYSCFG_EXTICR[temp1] &= ~(0xF << (temp2 * 4));
            SYSCFG->SYSCFG_EXTICR[temp1] |= (portCode << (temp2 * 4));

			/* Enable EXTI interrupt delivery. */
			EXTI->EXTI_IMR1 |= ( 1 << ptGpioHandle->GpioPinConfig.GPIO_PinNumber);
		}

		/* Configure the Speed of the GPIO pin. */
		if(ptGpioHandle->GpioPinConfig.GPIO_PinSpeed < eGPIO_SpeedTypeMax)
		{
			/* Calculate the register value. */
			uiRegVal = (ptGpioHandle->GpioPinConfig.GPIO_PinSpeed <<
					   (2 * ptGpioHandle->GpioPinConfig.GPIO_PinNumber));

			/* CClear the actual register. */
			ptGpioHandle->pGPIOx->OSPEEDR &= ~(0x3 <<(2* ptGpioHandle->GpioPinConfig.GPIO_PinNumber));

			/* Copy the register value to actual register. */
			ptGpioHandle->pGPIOx->OSPEEDR |= uiRegVal;

			/* Reset The temporary register. */
			uiRegVal = 0;
		}

		/* Configure the PUPD settings. */
		if(ptGpioHandle->GpioPinConfig.GPIO_PinPuPdCtrl < eGPIO_PudPdMax)
		{
			/* Calculate the register value. */
			uiRegVal = (ptGpioHandle->GpioPinConfig.GPIO_PinPuPdCtrl <<
					   (2 * ptGpioHandle->GpioPinConfig.GPIO_PinNumber));

			/* CClear the actual register. */
			ptGpioHandle->pGPIOx->PUPDR &= ~(0x3 <<(2 * ptGpioHandle->GpioPinConfig.GPIO_PinNumber));

			/* Copy the register value to actual register. */
			ptGpioHandle->pGPIOx->PUPDR |= uiRegVal;

			/* Reset The temporary register. */
			uiRegVal = 0;
		}

		/* Configure the output type. */
		if(ptGpioHandle->GpioPinConfig.GPIO_PinOpType < eGPIO_OpTypeMax)
		{
			/* Calculate the register value. */
			uiRegVal = (ptGpioHandle->GpioPinConfig.GPIO_PinOpType <<
					   (ptGpioHandle->GpioPinConfig.GPIO_PinNumber));

			/* CClear the actual register. */
			ptGpioHandle->pGPIOx->OTYPER &= ~(0x1 << ptGpioHandle->GpioPinConfig.GPIO_PinNumber);

			/* Copy the register value to actual register. */
			ptGpioHandle->pGPIOx->OTYPER |= uiRegVal;

			/* Reset The temporary register. */
			uiRegVal = 0;
		}

		/* Configure the alternate functionality. */
		if(ptGpioHandle->GpioPinConfig.GPIO_PinMode == eGPIO_AfMode)
		{
			uint8_t ucPinNumber = 0;

			ucPinNumber = (ptGpioHandle->GpioPinConfig.GPIO_PinNumber % 8);
			/* Calculate the register value. */
			uiRegVal = (ptGpioHandle->GpioPinConfig.GPIO_PinAltFunMode << (4 * ucPinNumber));

			/* Check the Pin belongs to AFRL. */
			if(ptGpioHandle->GpioPinConfig.GPIO_PinNumber >= eGPIO_PIN_0 &&
			 ptGpioHandle->GpioPinConfig.GPIO_PinNumber <= eGPIO_PIN_7)
			{

				/* CClear the actual register. */
				ptGpioHandle->pGPIOx->AFRL &= ~(0xF << (4 *ucPinNumber));

				/* Copy the register value to actual register. */
				ptGpioHandle->pGPIOx->AFRL |= uiRegVal;
			}

			/* Check the Pin belongs to AFRH. */
			else if(ptGpioHandle->GpioPinConfig.GPIO_PinNumber >= eGPIO_PIN_8 &&
			 ptGpioHandle->GpioPinConfig.GPIO_PinNumber < eGPIO_Pin_Max)
			{

				/* CClear the actual register. */
				ptGpioHandle->pGPIOx->AFRH &= ~(0xF <<(4 * ucPinNumber));

				/* Copy the register value to actual register. */
				ptGpioHandle->pGPIOx->AFRH |= uiRegVal;
			}

			/* Reset The temporary register. */
			uiRegVal = 0;
		}
	}

	return;
}

/******************************************************************************
 * @brief : Function For De Initializing GPIO Peripheral.
 * @fn : GPIO_DeInit(Gpio_RegDef_t* pGPIOx)
 *
 * @param[in] : pGPIOx
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_DeInit(Gpio_RegDef_t* pGPIOx)
{
	if(NULL != pGPIOx)
	{
		if(GPIOA == pGPIOx)
		{
			GPOIA_REG_RESET();
		}
		else if(GPIOB == pGPIOx)
		{
			GPOIB_REG_RESET();
		}
		else if(GPIOC == pGPIOx)
		{
			GPOIC_REG_RESET();
		}
		else if(GPIOD == pGPIOx)
		{
			GPOID_REG_RESET();
		}
		else if(GPIOE == pGPIOx)
		{
			GPOIE_REG_RESET();
		}
		else if(GPIOF == pGPIOx)
		{
			GPOIF_REG_RESET();
		}
		else if(GPIOG == pGPIOx)
		{
			GPOIG_REG_RESET();
		}
		else if(GPIOH == pGPIOx)
		{
			GPOIH_REG_RESET();
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For initializing and  De Initializing GPIO Peripheral
 * 			Clock.
 * @fn : GPIO_PeriClkCtrl(Gpio_RegDef_t* pGPIOx, uint8_t EnOrDi)
 *
 * @param[in] : pGPIOx
 * @param[in] : EnOrDi
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_PeriClkCtrl(Gpio_RegDef_t* pGPIOx, uint8_t EnOrDi)
{
	if(NULL != pGPIOx)
	{
		/* Enable The Clock For Corresponding Port. */
		if(ENABLE == EnOrDi)
		{
			if(GPIOA == pGPIOx)
			{
				GPOIA_PCLK_EN();
			}
			else if(GPIOB == pGPIOx)
			{
				GPOIB_PCLK_EN();
			}
			else if(GPIOC == pGPIOx)
			{
				GPOIC_PCLK_EN();
			}
			else if(GPIOD == pGPIOx)
			{
				GPOID_PCLK_EN();
			}
			else if(GPIOE == pGPIOx)
			{
				GPOIE_PCLK_EN();
			}
			else if(GPIOF == pGPIOx)
			{
				GPOIF_PCLK_EN();
			}
			else if(GPIOG == pGPIOx)
			{
				GPOIG_PCLK_EN();
			}
			else if(GPIOH == pGPIOx)
			{
				GPOIH_PCLK_EN();
			}
		}

		if(DISABLE == EnOrDi)
		{
			/* Disable The Clock For Corresponding Port. */
			if(GPIOA == pGPIOx)
			{
				GPOIA_PCLK_DI();
			}
			else if(GPIOB == pGPIOx)
			{
				GPOIB_PCLK_DI();
			}
			else if(GPIOC == pGPIOx)
			{
				GPOIC_PCLK_DI();
			}
			else if(GPIOD == pGPIOx)
			{
				GPOID_PCLK_DI();
			}
			else if(GPIOE == pGPIOx)
			{
				GPOIE_PCLK_DI();
			}
			else if(GPIOF == pGPIOx)
			{
				GPOIF_PCLK_DI();
			}
			else if(GPIOG == pGPIOx)
			{
				GPOIG_PCLK_DI();
			}
			else if(GPIOH == pGPIOx)
			{
				GPOIH_PCLK_DI();
			}
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For Writing value to GPIO Port.
 * @fn : GPIO_WriteToOpPort(Gpio_RegDef_t* pGPIOx, uint16_t Val)
 *
 * @param[in] : pGPIOx
 * @param[in] : Val
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_WriteToOpPort(Gpio_RegDef_t* pGPIOx, uint16_t Val)
{
	if(NULL != pGPIOx)
	{
		pGPIOx->ODR = Val;
	}
	return;
}

/******************************************************************************
 * @brief : Function For Toggling particular GPIO pin.
 * @fn : GPIO_TogglePin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum)
 *
 * @param[in] : pGPIOx
 * @param[in] : PinNum
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_TogglePin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum)
{
	if(NULL != pGPIOx
	&&(PinNum >= eGPIO_PIN_0 && PinNum < eGPIO_Pin_Max))
	{
		pGPIOx->ODR ^= (1 << PinNum);
	}
	return;
}

/******************************************************************************
 * @brief : Function For Write high or low to particular pin.
 * @fn : GPIO_WriteToOpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum,
					   eGPIOState eSetOrReset)
 *
 * @param[in] : pGPIOx
 * @param[in] : PinNum
 * @param[in] : eSetOrReset
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_WriteToOpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum,
					   eGPIOState eSetOrReset)
{
	if(NULL != pGPIOx
	&& (PinNum >= eGPIO_PIN_0 && PinNum < eGPIO_Pin_Max))
	{
		if(eGPIOReset == eSetOrReset)
		{
			pGPIOx->ODR &= ~(1 << PinNum);
		}
		else if(eGPIOSet == eSetOrReset)
		{
			pGPIOx->ODR |= (1 << PinNum);
		}
	}
	return;
}

/******************************************************************************
 * @brief : Function For Read from particular pin configured as input.
 * @fn : GPIO_ReadFromIpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum)
 *
 * @param[in] : pGPIOx
 * @param[in] : PinNum
 *
 * @param[out] : uint8_t.
 *
 * @return : None
 *
 *****************************************************************************/
uint8_t GPIO_ReadFromIpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum)
{
	/* Variable declaration. */
	uint8_t ucGpioPinVal = 0;

	/* Parameter validity check. */
	if((NULL != pGPIOx)
	&& (PinNum >= eGPIO_PIN_0 && PinNum < eGPIO_Pin_Max))
	{
		ucGpioPinVal = (uint8_t)((pGPIOx->IDR >> PinNum) & 0x00000001);
	}

	/* Return the value. */
	return ucGpioPinVal;
}

/******************************************************************************
 * @brief : Function For Read from particular pin configured as output.
 * @fn : GPIO_ReadFromOpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum)
 *
 * @param[in] : pGPIOx
 * @param[in] : PinNum
 *
 * @param[out] : uint8_t.
 *
 * @return : None
 *
 *****************************************************************************/
uint8_t GPIO_ReadFromOpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum)
{
	return 0;
}

/******************************************************************************
 * @brief : Function For Read from GPIO configured as output.
 * @fn : GPIO_ReadFromOpPort(Gpio_RegDef_t* pGPIOx)
 *
 * @param[in] : pGPIOx
 *
 * @param[out] : uint16_t.
 *
 * @return : None
 *
 *****************************************************************************/
uint16_t GPIO_ReadFromOpPort(Gpio_RegDef_t* pGPIOx)
{
	return 0;
}

/******************************************************************************
 * @brief : Function For Read from GPIO configured as Input.
 * @fn : GPIO_ReadFromOpPort(Gpio_RegDef_t* pGPIOx)
 *
 * @param[in] : pGPIOx
 *
 * @param[out] : uint16_t.
 *
 * @return : None
 *
 *****************************************************************************/
uint16_t GPIO_ReadFromIpPort(Gpio_RegDef_t* pGPIOx)
{
	/* Variable declaration. */
	uint16_t ucGpioPinVal = 0;

	/* Parameter validity check. */
	if(NULL != pGPIOx)
	{
		ucGpioPinVal = (uint16_t)(pGPIOx->IDR);
	}

	/* Return the value. */
	return ucGpioPinVal;
}

/******************************************************************************
 * @brief : Function to configure the IRQ.
 * @fn : GPIO_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi)
 *
 * @param[in] : IRQNum
 * @param[in] : EnOrDi
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi)
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
 * @brief : Function to configure the priority of the GPIO pin.
 * @fn : GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal)
 *
 * @param[in] : IRQNum
 * @param[in] : PriorityVal
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal)
{
	uint8_t iprx = 0;
	uint8_t iprxSection = 0;
	uint8_t shiftAmnt = 0;

	/* Find the priority register for corresponding irq number. */
	iprx = (IRQNum / 4);

	/* Find the priority register section for corresponding irq number. */
	iprxSection = (IRQNum % 4);

	shiftAmnt = (8 * iprxSection ) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( PriorityVal << shiftAmnt);
}

/******************************************************************************
 * @brief : Function to configure the priority of the GPIO pin.
 * @fn : GPIO_IRQHandling(eGPIOPin PinNum)
 *
 * @param[in] : IRQNum
 * @param[in] : PriorityVal
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void GPIO_IRQHandling(eGPIOPin PinNum)
{
	if(EXTI->EXTI_PR1 & ( 1 << PinNum))
	{
		/* Clear */
		EXTI->EXTI_PR1 |= ( 1 << PinNum);
	}
}

