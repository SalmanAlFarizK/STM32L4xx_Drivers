/******************************************************************************
 *		Header File Containing SYSTICK related Informations and
 *		Drivers (API's) Related SYSTICK Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/

#ifndef INC_SYSTICK_H_
#define INC_SYSTICK_H_
#include "stm32l476xx.h"

#define SYSTICK_LOAD_VAL		(4000)
#define SYSTICK_CSR				((__vo uint32_t*)0xE000E010)
#define SYSTICK_RVR				((__vo uint32_t*)0xE000E014)
#define SYSTICK_CVR				((__vo uint32_t*)0xE000E018)
#define SYSTICK_CALIB			((__vo uint32_t*)0xE000E010)

/******************************************************************************
 * FUnction Declarations.
 *****************************************************************************/
void SysTickInit(void);
uint32_t GetCurrentMs(void);


#endif /* INC_SYSTICK_H_ */
