/******************************************************************************
 *		STM32L476 MCU Specific Details
 *****************************************************************************/
#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#include <stdint.h>

#define __vo		volatile

/******************************************************************************
 * 					Processor Specific Details.
 *****************************************************************************/
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)

#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4

/******************************************************************************
 * Common Macros.
 *****************************************************************************/
#define ENABLE			(0x01)
#define DISABLE			(0x00)

/******************************************************************************
 * @brief: Base addresses of Flash and SRAM memory addresses.
 *****************************************************************************/
#define FLASH_BASEADDR					(0x08000000U)
#define SRAM1_BASEADDR					(0x20000000U)
#define SRAM2_BASEADDR					(0x20030000U)
#define ROM_BASEADDR					(0x1FFF0000U)
#define SRAM							(SRAM1_BASEADDR)


/******************************************************************************
 * @brief: Base addresses of System buses.
 *****************************************************************************/
#define PERIPH_BASE							(0x40000000U)
#define APB1PERIPH_BASE						(PERIPH_BASE)
#define APB2PERIPH_BASE						(0x40010000U)
#define AHB1PERIPH_BASE						(0x40020000U)
#define AHB2PERIPH_BASE						(0x48000000U)


/******************************************************************************
 * @brief: Base addresses of peripherals residing in APB1 Bus.
 *****************************************************************************/
#define TIM2_BASEADDR				(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR				(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR				(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR				(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR				(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR				(APB1PERIPH_BASE + 0x1400)
#define RTC_BASEADDR				(APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR				(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR				(APB1PERIPH_BASE + 0x3000)
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0x5000)
#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASE + 0x5C00)
#define CRS_BASEADDR				(APB1PERIPH_BASE + 0x6000)
#define CAN1_BASEADDR				(APB1PERIPH_BASE + 0x6400)
#define PWR_BASEADDR				(APB1PERIPH_BASE + 0x7000)
#define DAC1_BASEADDR				(APB1PERIPH_BASE + 0x7400)
#define OPAMP_BASEADDR				(APB1PERIPH_BASE + 0x7800)

/******************************************************************************
 * @brief: Base addresses of peripherals residing in APB2 Bus.
 *****************************************************************************/
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x0000)
#define VREFBUF_BASEADDR			(APB2PERIPH_BASE + 0x0030)
#define COMP_BASEADDR				(APB2PERIPH_BASE + 0x0200)
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x0400)
#define FIREWALL_BASEADDR			(APB2PERIPH_BASE + 0x1C00)
#define TIM1_BASEADDR				(APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000)
#define TIM8_BASEADDR				(APB2PERIPH_BASE + 0x3400)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x3800)
#define TIM15_BASEADDR				(APB2PERIPH_BASE + 0x4000)
#define TIM16_BASEADDR				(APB2PERIPH_BASE + 0x4400)
#define TIM17_BASEADDR				(APB2PERIPH_BASE + 0x4800)


/******************************************************************************
 * @brief: Base addresses of peripherals residing in AHB1 Bus.
 *****************************************************************************/
#define DMA1_BASEADDR				(AHB1PERIPH_BASE + 0x0000)
#define DMAMUX1_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define DMA2_BASEADDR				(AHB1PERIPH_BASE + 0x0400)
#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x1000)
#define FLASHREG_BASEADDR			(AHB1PERIPH_BASE + 0x2000)
#define CRC_BASEADDR				(AHB1PERIPH_BASE + 0x3000)
#define TSC_BASEADDR				(AHB1PERIPH_BASE + 0x4000)
#define DMA2D_BASEADDR				(AHB1PERIPH_BASE + 0xB000)
#define GFXMMU_BASEADDR				(AHB1PERIPH_BASE + 0xC000)


/******************************************************************************
 * @brief: Base addresses of peripherals residing in AHB2 Bus.
 *****************************************************************************/
#define GPIOA_BASEADDR				(AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR				(AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR				(AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR				(AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR				(AHB2PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR				(AHB2PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR				(AHB2PERIPH_BASE + 0x1C00)


/******************************************************************************
 * 				Peripheral Register Definition Structures.
 *****************************************************************************/

/******************************************************************************
 * Section: GPIO
 * @brief : Register Definition Structure for GPIO.
 *****************************************************************************/
typedef struct _Gpio_RegDef_t_
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
	__vo uint32_t BRR;
} Gpio_RegDef_t;

/******************************************************************************
 * @brief: GPIO Peripheral definitions.
 *****************************************************************************/
#define GPIOA						((Gpio_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((Gpio_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((Gpio_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((Gpio_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((Gpio_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((Gpio_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG						((Gpio_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH						((Gpio_RegDef_t*)GPIOH_BASEADDR)

/******************************************************************************
 * Section: RCC
 * @brief : Register Definition Structure for RCC.
 *****************************************************************************/
typedef struct _Rcc_Regdef_t_
{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_ICSCR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_PLL_CFGR;
	__vo uint32_t RCC_PLLSAI1_CFGR;
	__vo uint32_t RCC_PLLSAI2_CFGR;
	__vo uint32_t RCC_CIER;
	__vo uint32_t RCC_CIFR;
	__vo uint32_t RCC_CICR;
	uint32_t Reserved0;
	__vo uint32_t RCC_AHB1RSTR;
	__vo uint32_t RCC_AHB2RSTR;
	__vo uint32_t RCC_AHB3RSTR;
	uint32_t Reserved1;
	__vo uint32_t RCC_APB1RSTR1;
	__vo uint32_t RCC_APB1RSTR2;
	__vo uint32_t RCC_APB2RSTR;
	uint32_t Reserved2;
	__vo uint32_t RCC_AHB1_ENR;
	__vo uint32_t RCC_AHB2_ENR;
	__vo uint32_t RCC_AHB3_ENR;
	uint32_t Reserved3;
	__vo uint32_t RCC_APB1ENR1;
	__vo uint32_t RCC_APB1ENR2;
	__vo uint32_t RCC_APB2ENR;
	uint32_t Reserved4;
	__vo uint32_t RCC_AHB1SMENR;
	__vo uint32_t RCC_AHB2SMENR;
	__vo uint32_t RCC_AHB3SMENR;
	uint32_t Reserved5;
	__vo uint32_t RCC_APB1SM_ENR1;
	__vo uint32_t RCC_APB1SM_ENR2;
	__vo uint32_t RCC_APB2SMENR;
	uint32_t Reserved6;
	__vo uint32_t RCC_CCIPR;
	uint32_t reserved7;
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
} Rcc_Regdef_t;

/******************************************************************************
 * @brief: RCC Peripheral definitions.
 *****************************************************************************/
#define RCC							((Rcc_Regdef_t*)RCC_BASEADDR)

/******************************************************************************
 * @brief: RCC Peripheral clock enable macros.
 *****************************************************************************/

/* GPIO Clock Enables / Disables. */

#define GPOIA_PCLK_EN()			(RCC->RCC_AHB2_ENR |= (1 << 0))
#define GPOIB_PCLK_EN()			(RCC->RCC_AHB2_ENR |= (1 << 1))
#define GPOIC_PCLK_EN()			(RCC->RCC_AHB2_ENR |= (1 << 2))
#define GPOID_PCLK_EN()			(RCC->RCC_AHB2_ENR |= (1 << 3))
#define GPOIE_PCLK_EN()			(RCC->RCC_AHB2_ENR |= (1 << 4))
#define GPOIF_PCLK_EN()			(RCC->RCC_AHB2_ENR |= (1 << 5))
#define GPOIG_PCLK_EN()			(RCC->RCC_AHB2_ENR |= (1 << 6))
#define GPOIH_PCLK_EN()			(RCC->RCC_AHB2_ENR |= (1 << 7))

#define GPOIA_PCLK_DI()			(RCC->RCC_AHB2_ENR &= ~(1 << 0))
#define GPOIB_PCLK_DI()			(RCC->RCC_AHB2_ENR &= ~(1 << 1))
#define GPOIC_PCLK_DI()			(RCC->RCC_AHB2_ENR &= ~(1 << 2))
#define GPOID_PCLK_DI()			(RCC->RCC_AHB2_ENR &= ~(1 << 3))
#define GPOIE_PCLK_DI()			(RCC->RCC_AHB2_ENR &= ~(1 << 4))
#define GPOIF_PCLK_DI()			(RCC->RCC_AHB2_ENR &= ~(1 << 5))
#define GPOIG_PCLK_DI()			(RCC->RCC_AHB2_ENR &= ~(1 << 6))
#define GPOIH_PCLK_DI()			(RCC->RCC_AHB2_ENR &= ~(1 << 7))

/* GPIO Register reset. */
#define GPOIA_REG_RESET()		do{(RCC->RCC_AHB2RSTR |= (1 << 0)); (RCC->RCC_AHB2RSTR &= ~(1 << 0));}while(0)
#define GPOIB_REG_RESET()		do{(RCC->RCC_AHB2RSTR |= (1 << 1)); (RCC->RCC_AHB2RSTR &= ~(1 << 1));}while(0)
#define GPOIC_REG_RESET()		do{(RCC->RCC_AHB2RSTR |= (1 << 2)); (RCC->RCC_AHB2RSTR &= ~(1 << 2));}while(0)
#define GPOID_REG_RESET()		do{(RCC->RCC_AHB2RSTR |= (1 << 3)); (RCC->RCC_AHB2RSTR &= ~(1 << 3));}while(0)
#define GPOIE_REG_RESET()		do{(RCC->RCC_AHB2RSTR |= (1 << 4)); (RCC->RCC_AHB2RSTR &= ~(1 << 4));}while(0)
#define GPOIF_REG_RESET()		do{(RCC->RCC_AHB2RSTR |= (1 << 5)); (RCC->RCC_AHB2RSTR &= ~(1 << 5));}while(0)
#define GPOIG_REG_RESET()		do{(RCC->RCC_AHB2RSTR |= (1 << 6)); (RCC->RCC_AHB2RSTR &= ~(1 << 6));}while(0)
#define GPOIH_REG_RESET()		do{(RCC->RCC_AHB2RSTR |= (1 << 7)); (RCC->RCC_AHB2RSTR &= ~(1 << 7));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 : \
										 (x == GPIOB) ? 1 : \
										 (x == GPIOC) ? 2 : \
										 (x == GPIOD) ? 3 : \
										 (x == GPIOE) ? 4 : \
										 (x == GPIOF) ? 5 : \
										 (x == GPIOG) ? 6 : \
										 (x == GPIOH) ? 7 : 0)

/******************************************************************************
 * Section: SYSCFG
 * @brief : Register Definition Structure for SYSCFG.
 *****************************************************************************/
typedef struct _SysCfg_Regdef_t_
{
	__vo uint32_t  SYSCFG_MEMRMP;
	__vo uint32_t  SYSCFG_CFGR1;
	__vo uint32_t  SYSCFG_EXTICR[4];
	__vo uint32_t  SYSCFG_SCSR;
	__vo uint32_t  SYSCFG_CFGR2;
	__vo uint32_t  SYSCFG_SWPR;
	__vo uint32_t  SYSCFG_SKR;
} SysCfg_Regdef_t;

/******************************************************************************
 * @brief: RCC Peripheral definitions.
 *****************************************************************************/
#define SYSCFG							((SysCfg_Regdef_t*)SYSCFG_BASEADDR)

/* SYSCFG Clock Enable / disable Macro */
#define SYSCFG_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 0))
#define SYSCFG_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 0))


/******************************************************************************
 * Section: EXTI
 * @brief : Register Definition Structure for EXTI.
 *****************************************************************************/
typedef struct _EXTI_Regdef_t_
{
	__vo uint32_t EXTI_IMR1;
	__vo uint32_t EXTI_EMR1;
	__vo uint32_t EXTI_RTSR1;
	__vo uint32_t EXTI_FTSR1;
	__vo uint32_t EXTI_SWIER1;
	__vo uint32_t EXTI_PR1;
	uint32_t Reserved0;
	uint32_t Reserved1;
	__vo uint32_t EXTI_IMR2;
	__vo uint32_t EXTI_EMR2;
	__vo uint32_t EXTI_RTSR2;
	__vo uint32_t EXTI_FTSR2;
	__vo uint32_t EXTI_SWIER2;
	__vo uint32_t EXTI_PR2;
} EXTI_Regdef_t;

/******************************************************************************
 * @brief: EXTI Peripheral definitions.
 *****************************************************************************/
#define EXTI							((EXTI_Regdef_t*)EXTI_BASEADDR)



/******************************************************************************
 * @brief: Interrupts availabke.
 *****************************************************************************/
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Cortex-M4 Non Maskable Interrupt                                */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M4 Hard Fault Interrupt                                  */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_PVM_IRQn                = 1,      /*!< PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection Interrupts    */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                                   */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                                   */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                                   */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                                   */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                                   */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                                   */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                                   */
  ADC1_2_IRQn                 = 18,     /*!< ADC1, ADC2 SAR global Interrupts                                  */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break interrupt and TIM15 global interrupt                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM16 global interrupt                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM17 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  DFSDM1_FLT3_IRQn            = 42,     /*!< DFSDM1 Filter 3 global Interrupt                                  */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                             */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt                            */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  ADC3_IRQn                   = 47,     /*!< ADC3 global  Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  DFSDM1_FLT0_IRQn            = 61,     /*!< DFSDM1 Filter 0 global Interrupt                                  */
  DFSDM1_FLT1_IRQn            = 62,     /*!< DFSDM1 Filter 1 global Interrupt                                  */
  DFSDM1_FLT2_IRQn            = 63,     /*!< DFSDM1 Filter 2 global Interrupt                                  */
  COMP_IRQn                   = 64,     /*!< COMP1 and COMP2 Interrupts                                        */
  LPTIM1_IRQn                 = 65,     /*!< LP TIM1 interrupt                                                 */
  LPTIM2_IRQn                 = 66,     /*!< LP TIM2 interrupt                                                 */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Channel6_IRQn          = 68,     /*!< DMA2 Channel 6 global interrupt                                   */
  DMA2_Channel7_IRQn          = 69,     /*!< DMA2 Channel 7 global interrupt                                   */
  LPUART1_IRQn                = 70,     /*!< LP UART1 interrupt                                                */
  QUADSPI_IRQn                = 71,     /*!< Quad SPI global interrupt                                         */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                   = 74,     /*!< Serial Audio Interface 1 global interrupt                         */
  SAI2_IRQn                   = 75,     /*!< Serial Audio Interface 2 global interrupt                         */
  SWPMI1_IRQn                 = 76,     /*!< Serial Wire Interface 1 global interrupt                          */
  TSC_IRQn                    = 77,     /*!< Touch Sense Controller global interrupt                           */
  LCD_IRQn                    = 78,     /*!< LCD global interrupt                                              */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
} IRQn_Type;






#endif /* INC_STM32L476XX_H_ */
