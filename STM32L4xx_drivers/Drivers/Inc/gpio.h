/******************************************************************************
 *		Header File Containing GPIO related Informations and
 *		Drivers (API's) Related GPIO Peripheral of STM32L476
 *		MCU.
 *****************************************************************************/
#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include <stdint.h>
#include "stm32l476xx.h"

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/
/*
 * @brief : Configuration Structure for GPIO Pin.
 */
typedef struct _GPIO_Config_t_
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdCtrl;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_Config_t;


/*
 * @brief : Handle Structure for GPIO Pin.
 */
typedef struct _GPIO_Handle_t_
{
	Gpio_RegDef_t* pGPIOx;
	GPIO_Config_t  GpioPinConfig;
} GPIO_Handle_t;

/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/
/**
 * Enum Representing GPIO States.
 */
typedef enum _eGPIOState_
{
	eGPIOReset = 0,
	eGPIOSet
} eGPIOState;

/**
 * Enum Representing GPIO Pin Numbers.
 */
typedef enum _eGPIOPin_
{
	eGPIO_PIN_0 = 0,
	eGPIO_PIN_1,
	eGPIO_PIN_2,
	eGPIO_PIN_3,
	eGPIO_PIN_4,
	eGPIO_PIN_5,
	eGPIO_PIN_6,
	eGPIO_PIN_7,
	eGPIO_PIN_8,
	eGPIO_PIN_9,
	eGPIO_PIN_10,
	eGPIO_PIN_11,
	eGPIO_PIN_12,
	eGPIO_PIN_13,
	eGPIO_PIN_14,
	eGPIO_PIN_15,
	eGPIO_Pin_Max
} eGPIOPin;

/**
 * Enum Representing GPIO Pin modes.
 */
typedef enum _eGPIO_Modes_
{
	eGPIO_IpMode = 0,
	eGPIO_OpMode,
	eGPIO_AfMode,
	eGPIO_AnalogMode,
	eGPIO_ITFeT,
	eGPIO_ITReT,
	eGPIO_ITReFeT,
	eGPIO_ModeMax
} eGPIO_Modes;

/**
 * Enum Representing GPIO Pin Output types.
 */
typedef enum _eGPIO_OpType_
{
	eGPIO_OpPushPull = 0,
	eGPIO_OpOpenDrain,
	eGPIO_OpTypeMax
} eGPIO_OpType;

/**
 * Enum Representing GPIO Pin Speed types.
 */
typedef enum _eGPIO_SpeedType_
{
	eGPIO_LowSpeed = 0,
	eGPIO_MediumSpeed,
	eGPIO_HighSpeed,
	eGPIO_VeryHighSpeed,
	eGPIO_SpeedTypeMax
} eGPIO_SpeedType;

/**
 * Enum Representing GPIO Pin pull up pull down resistor types.
 */
typedef enum _eGPIO_PuPdType_
{
	eGPIO_NoPuPd = 0,
	eGPIO_Pu,
	eGPIO_Pd,
	eGPIO_Reserved,
	eGPIO_PudPdMax
} eGPIO_PuPdType;

/**
 * Enum Representing GPIO Pin alternate function types.
 */
typedef enum _eGPIO_AFType_
{
	eGPIO_Af0 = 0,
	eGPIO_Af1,
	eGPIO_Af2,
	eGPIO_Af3,
	eGPIO_Af4,
	eGPIO_Af5,
	eGPIO_Af6,
	eGPIO_Af7,
	eGPIO_Af8,
	eGPIO_Af9,
	eGPIO_Af10,
	eGPIO_Af11,
	eGPIO_Af12,
	eGPIO_Af13,
	eGPIO_Af14,
	eGPIO_Af15,
	eGPIO_AfMax
} eGPIO_AfType;


/******************************************************************************
 * Function Declarations.
 *****************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t PriorityVal);
void GPIO_IRQHandling(eGPIOPin PinNum);
void GPIO_Init(GPIO_Handle_t* ptGpioHandle);
void GPIO_DeInit(Gpio_RegDef_t* pGPIOx);
void GPIO_PeriClkCtrl(Gpio_RegDef_t* pGPIOx, uint8_t EnOrDi);
void GPIO_WriteToOpPort(Gpio_RegDef_t* pGPIOx, uint16_t Val);
void GPIO_TogglePin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum);
void GPIO_WriteToOpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum,
							  eGPIOState eSetOrReset);

uint8_t GPIO_ReadFromIpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum);
uint8_t GPIO_ReadFromOpPin(Gpio_RegDef_t* pGPIOx, eGPIOPin PinNum);
uint16_t GPIO_ReadFromOpPort(Gpio_RegDef_t* pGPIOx);
uint16_t GPIO_ReadFromIpPort(Gpio_RegDef_t* pGPIOx);




#endif /* INC_GPIO_H_ */
