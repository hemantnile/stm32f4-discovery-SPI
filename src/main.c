/**
 ******************************************************************************
 * @file    Src/main.c 
 * @author  Hemant Nile
 * @version V1
 * @date    04-Feb-2017
 * @brief   Simple implementation of accessing LIS3DSH accelerometer on STM32F4 
	    Discovery board using SPI interface. Four LEDs present on the board 
	    lit up when board is tilted in their direction.
 
 ******************************************************************************
 */

#include "main.h"

static void Configure_LEDS(void);
static void Configure_SPI1(void);

int main(void)
{
	int8_t x, y, z;

// Configure peripherals
	Configure_LEDS();
	Configure_SPI1();

// Initiating accelerometer LIS3DSH
	SPI_Tx(0x20U, 0x67U); // CTRL_REG4: output data rate = 100Hz, continuous update, x,y,z enable
	SPI_Tx(0x24U, 0x48U); // CTRL_REG5: Anti-aliasing filter bandwidth=200Hz, full scale = 4g

// Infinite loop
	while (1)
	{
		x = SPI_Rx((uint8_t) 0x29U);
		y = SPI_Rx((uint8_t) 0x2BU);
		z = SPI_Rx((uint8_t) 0x2DU);

		if (x < -20) SetPin(GPIOD, PIN_12);
		else ResetPin(GPIOD, PIN_12);
		if (x > 20) SetPin(GPIOD, PIN_14);
		else ResetPin(GPIOD, PIN_14);
		if (y < -20) SetPin(GPIOD, PIN_15);
		else ResetPin(GPIOD, PIN_15);
		if (y > 20) SetPin(GPIOD, PIN_13);
		else ResetPin(GPIOD, PIN_13);
	}
}

/**  
 *   System Clock Configuration
 *   The system Clock is configured as follows in MAIN.H : 
 *
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 168000000
 *            HCLK(Hz)                       = 168000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 336
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 */
void SystemInit(void)
{
// FPU settings
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2)); /* set CP10 and CP11 Full Access */
#endif

// Configure the Vector Table location add offset address
#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE; /* Vector Table Relocation in Internal SRAM */
#else
	SCB->VTOR = FLASH_BASE; /* Vector Table Relocation in Internal FLASH */
#endif

// Enable PWR CLK and set voltage scale 1
	CLK_ENABLE(RCC->APB1ENR, RCC_APB1ENR_PWREN);
	SET_BIT(PWR->CR, PWR_CR_VOS);

// Flash latency, prefech, instruction cache , data cache
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);

// Enable HSE and wait for it	
	SET_BIT(RCC->CR, RCC_CR_HSEON);
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RCC_CR_HSERDY)
	{
	}
	WRITE_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE | PLL_M | PLL_N << 6 | ((PLL_P >> 1U) - 1U) << 16 | PLL_Q << 24);
// Enable PLL and wait for it	
	SET_BIT(RCC->CR, RCC_CR_PLLON);
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RCC_CR_PLLRDY)
	{
	}
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1 | RCC_CFGR_HPRE | RCC_CFGR_SW_PLL,
			APB2_PRE | APB1_PRE | AHB_PRE | RCC_CFGR_SW_PLL);
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
	{
	}
// Disable HSI	
	CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

static void Configure_LEDS(void)
{
	CLK_ENABLE(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
	MODIFY_REG(GPIOD->MODER, (uint32_t) 0xFF000000U, (uint32_t) 0x55000000U); // pins 12,13,14,15 as output
	MODIFY_REG(GPIOD->OTYPER, (uint32_t) 0xFF000000U, (uint32_t) 0x00000000U); // GPIOD->OTYPER - PUSH PULL 
	MODIFY_REG(GPIOD->OSPEEDR, (uint32_t) 0xFF000000U, (uint32_t) 0xFF000000U); // pins 12,13,14,15 very high speed 
	MODIFY_REG(GPIOD->PUPDR, (uint32_t) 0xFF000000U, (uint32_t) 0x00000000U); // GPIOD->PUPDR - NO PULL
}

static void Configure_SPI1(void)
{
	CLK_ENABLE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
	CLK_ENABLE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);

	MODIFY_REG(GPIOA->MODER, (uint32_t) 0x0000FC00U, (uint32_t) 0x0000A800U);
	MODIFY_REG(GPIOA->OTYPER, (uint32_t) 0x000000E0U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOA->OSPEEDR, (uint32_t) 0x0000FC00U, (uint32_t) 0x00005400U);
	MODIFY_REG(GPIOA->PUPDR, (uint32_t) 0x0000FC00U, (uint32_t) 0x00005400U);
	MODIFY_REG(GPIOA->AFR[0], (uint32_t) 0xFFF00000U, (uint32_t) 0x55500000U);

	MODIFY_REG(GPIOE->MODER, (uint32_t) 0x000000C0U, (uint32_t) 0x00000040U);
	MODIFY_REG(GPIOE->OTYPER, (uint32_t) 0x00000008U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOE->OSPEEDR, (uint32_t) 0x000000C0U, (uint32_t) 0x00000080U);
	MODIFY_REG(GPIOE->PUPDR, (uint32_t) 0x000000C0U, (uint32_t) 0x00000040U);

	CLK_ENABLE(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);

	WRITE_REG(SPI1->CR1, SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_BR_1);
	WRITE_REG(SPI1->CR2, SPI_CR2_ERRIE);

	NVIC_SetPriority(SPI1_IRQn, 0);
	NVIC_EnableIRQ(SPI1_IRQn);

	SetPin(GPIOE, PIN_3);
	SET_BIT(SPI1->CR1, SPI_CR1_SPE);
}

void SPI_Tx(uint8_t address, uint8_t data)
{
	uint8_t temp;
	ResetPin(GPIOE, PIN_3);
	while (READ_BIT(SPI1->SR, SPI_SR_TXE) == 0)
	{
	}
	*((__IO uint8_t *)&(SPI1->DR)) = address;
	while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == 0)
	{
	}
	temp = *((__IO uint8_t *)&(SPI1->DR));	// useless read
	while (READ_BIT(SPI1->SR, SPI_SR_TXE) == 0)
	{
	}
	*((__IO uint8_t *)&(SPI1->DR)) = data;
	while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == 0)
	{
	}
	temp = *((__IO uint8_t *)&(SPI1->DR));	// useless read
	while (READ_BIT(SPI1->SR, SPI_SR_BSY) == SPI_SR_BSY)
	{
	}
	SetPin(GPIOE, PIN_3);
}

uint8_t SPI_Rx(uint8_t address)
{
	uint8_t temp;
	ResetPin(GPIOE, PIN_3);
	address |= (uint8_t) 0x80U;
	while (READ_BIT(SPI1->SR, SPI_SR_TXE) == 0)
	{
	}
	*((__IO uint8_t *)&(SPI1->DR)) = address;
	while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == 0)
	{
	}
	temp = *((__IO uint8_t *)&(SPI1->DR));	// useless read
	while (READ_BIT(SPI1->SR, SPI_SR_TXE) == 0)
	{
	}
	*((__IO uint8_t *)&(SPI1->DR)) = (uint8_t)0x00U;
	while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == 0)
	{
	}
	temp = *((__IO uint8_t *)&(SPI1->DR));
	while (READ_BIT(SPI1->SR, SPI_SR_BSY) == SPI_SR_BSY)
	{
	}
	SetPin(GPIOE, PIN_3);
	return temp;
}

void SPI1_TransferError_Callback(void)
{
// Disable RXNE  Interrupt
	CLEAR_BIT(SPI1->CR2, SPI_CR2_RXNEIE);

// Disable TXE   Interrupt
	CLEAR_BIT(SPI1->CR2, SPI_CR2_TXEIE);

// All LEDs ON to indicate error occurs
	SetPin(GPIOD, PIN_12);
	SetPin(GPIOD, PIN_13);
	SetPin(GPIOD, PIN_14);
	SetPin(GPIOD, PIN_15);
}

void Error_Handler(void)
{
// User may add here some code to deal with this error
	while (1)
	{
	}
}
