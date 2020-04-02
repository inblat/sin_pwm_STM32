/*
 * main.h
 *
 *  Created on: 9 нояб. 2019 г.
 *      Author: gerovite
 */

#ifndef APP_MAIN_H_
#define APP_MAIN_H_


#include "stm32f4xx.h"

/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M      4
#define PLL_N      168

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      2

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      4

void clockConfig(void);

void clockConfig(void)
{
	RCC->CR |= (uint32_t)RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY)){};

    /* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    /* HCLK = SYSCLK / 1*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK / 1*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* PCLK1 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

    /* Configure the main PLL */
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till the main PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0){}

    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

    /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL){};
}


void TIM1Config(void);

void TIM1Config(void)
{

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

	// GPIOA
	GPIOA->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED7;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_0;
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH2_0 | GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH0_0;

	// GPIOB
	GPIOB->MODER |= GPIO_MODER_MODE1_1 | GPIO_MODER_MODE0_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED1 | GPIO_OSPEEDR_OSPEED0;
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL1_0 | GPIO_AFRL_AFRL0_0;


	// TIM1

	TIM1->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	TIM1->CR1 |= TIM_CR1_CMS; // Center-aligned mode selection
	//TIM1->CR1 |= TIM_CR1_DIR; // Direction

	TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // Output Compare 1 Mode
	TIM1->CCMR1 &=~ TIM_CCMR1_CC1S | TIM_CCMR1_CC2S; // Capture/Compare 1 Selection
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC1PE; // Output Compare 1 Preload enable
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
	TIM1->CCMR2 &=~ TIM_CCMR2_CC3S;
	TIM1->CCMR2 &=~ TIM_CCMR2_OC3PE; // Output Compare 3 Preload enable


	TIM1->CCER &=~ TIM_CCER_CC1P | TIM_CCER_CC1NP; //Capture/Compare 1 output Polarity
	TIM1->CCER &=~ TIM_CCER_CC2P | TIM_CCER_CC2NP; //Capture/Compare 2 output Polarity
	TIM1->CCER &=~ TIM_CCER_CC3P | TIM_CCER_CC3NP; //Capture/Compare 2 output Polarity
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE; //Capture/Compare 1 output enable
	TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE; //Capture/Compare 2 output enable
	TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE; //Capture/Compare 2 output enable

	TIM1->PSC = 10;
	TIM1->ARR = 1000;
	TIM1->CCR1 = 50;
	TIM1->CCR2 = 50;
	TIM1->CCR3 = 50;

	TIM1->BDTR |= TIM_BDTR_DTG_7 | TIM_BDTR_DTG_3; //DTG[0:7] bits (Dead-Time Generator set-up)
	TIM1->BDTR |= TIM_BDTR_MOE; //Main Output enable

	//TIM1->RCR = 1;

	TIM1->CR1 |= TIM_CR1_CKD_0;
	TIM1->CR1 |= TIM_CR1_CEN;
}

#endif /* APP_MAIN_H_ */
