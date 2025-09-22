/******************************************************************************
 *		Source File Containing SYSTICK related Informations and
 *		Drivers (API's) Related SYSTICK Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "systick.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
/* SYSTICK CSR Bit positions. */
#define SYSTICK_EN				(0x00)
#define SYSTICK_TICKINT			(0x01)
#define SYSTICK_CLKSRC			(0x02)
#define SYSTICK_CCNTFLAG		(0x10)

#define ONE_DAY_MS				(86400000)

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/
typedef struct _SysClk_
{
	long long uiMilliSec;
	short uhDay;
} SysClk;

/******************************************************************************
 * Global variable declaration.
 *****************************************************************************/
SysClk tSysClk;

/******************************************************************************
 * Function Definitions.
 *****************************************************************************/

/******************************************************************************
 * @brief : Function For Initializing SysTick Peripheral.
 * @fn : SysTickInit(void)
 *
 * @param[in] : None
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SysTickInit(void)
{
	/* Disable SysTick First*/
	*SYSTICK_CSR &= ~( 1 << SYSTICK_EN);

	/* Configure the load. */
	*SYSTICK_RVR |= SYSTICK_LOAD_VAL;

	/* Clear current value. */
	*SYSTICK_CVR = 0;

	/* Enable SysTick. */
	*SYSTICK_CSR |= (1 << SYSTICK_CLKSRC) | (1 << SYSTICK_TICKINT) |
			(1 << SYSTICK_EN);

	/* Enable SysTick IRQ. */
	*NVIC_ISER0 |= (1 << ((-1) & 0x1F));

	return;
}

/******************************************************************************
 * @brief :SysTick IRQ handler.
 * @fn : SysTickInit(void)
 *
 * @param[in] : None
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
void SysTick_Handler(void)
{
	tSysClk.uiMilliSec += 1;

	if(tSysClk.uiMilliSec > ONE_DAY_MS)
	{
		/* Reset the millisecond timer. */
		tSysClk.uiMilliSec = 0;

		/* Increment the day. */
		tSysClk.uhDay += 1;
	}

	return;
}

/******************************************************************************
 * @brief : Getter function to retrieve current millisecond value.
 * @fn : SysTickInit(void)
 *
 * @param[in] : None
 *
 * @param[out] : None.
 *
 * @return : uint32_t
 *
 *****************************************************************************/
uint32_t GetCurrentMs(void)
{
	/* return the global millisecond value. */
	return tSysClk.uiMilliSec;
}
