/******************************************************************************
 *		STM32L476 MCU Specific Details
 *****************************************************************************/
#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#include <stdint.h>

#define __vo		volatile

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

/******************************************************************************
 * Section: SYSCFG
 * @brief : Register Definition Structure for SYSCFG.
 *****************************************************************************/
typedef struct _SysCfg_Regdef_t_
{
	__vo uint32_t  SYSCFG_MEMRMP;
	__vo uint32_t  SYSCFG_CFGR1;
	__vo uint32_t  SYSCFG_EXTICR1;
	__vo uint32_t  SYSCFG_EXTICR2;
	__vo uint32_t  SYSCFG_EXTICR3;
	__vo uint32_t  SYSCFG_EXTICR4;
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



#endif /* INC_STM32L476XX_H_ */
