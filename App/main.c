/*
 * main.c
 *
 *  Created on: 9 нояб. 2019 г.
 *      Author: gerovite
 */

#include "main.h"


#define PMSM_CW		0
#define PMSM_CCW	1

volatile uint8_t PMSM_SinTableIndex = 0;
volatile uint16_t PMSM_PWM = 500;
volatile uint8_t PMSM_MotorSpin = PMSM_CW;

static volatile uint16_t hTimebase_500us = 0;

// Prototypes
void TB_Wait(uint32_t time);

// Functions
void TB_Wait(uint32_t time)
{
	hTimebase_500us = time;    // delay = 'time' value * 5ms
	while (hTimebase_500us != 0){} // wait and do nothing!
}


// Sin table
#define PMSM_SINTABLESIZE	192
static const uint8_t PMSM_SINTABLE [PMSM_SINTABLESIZE][3] =
{
		{0,       0,      221},
		{8,       0,      225},
		{17,      0,      229},
		{25,      0,      232},
		{33,      0,      236},
		{42,      0,      239},
		{50,      0,      241},
		{58,      0,      244},
		{66,      0,      246},
		{74,      0,      248},
		{82,      0,      250},
		{90,      0,      252},
		{98,      0,      253},
		{105,     0,      254},
		{113,     0,      254},
		{120,     0,      255},
		{128,     0,      255},
		{135,     0,      255},
		{142,     0,      254},
		{149,     0,      254},
		{155,     0,      253},
		{162,     0,      252},
		{168,     0,      250},
		{174,     0,      248},
		{180,     0,      246},
		{186,     0,      244},
		{192,     0,      241},
		{197,     0,      239},
		{202,     0,      236},
		{207,     0,      232},
		{212,     0,      229},
		{217,     0,      225},
		{221,     0,      221},
		{225,     0,      217},
		{229,     0,      212},
		{232,     0,      207},
		{236,     0,      202},
		{239,     0,      197},
		{241,     0,      192},
		{244,     0,      186},
		{246,     0,      180},
		{248,     0,      174},
		{250,     0,      168},
		{252,     0,      162},
		{253,     0,      155},
		{254,     0,      149},
		{254,     0,      142},
		{255,     0,      135},
		{255,     0,      127},
		{255,     0,      120},
		{254,     0,      113},
		{254,     0,      105},
		{253,     0,      98},
		{252,     0,      90},
		{250,     0,      82},
		{248,     0,      74},
		{246,     0,      66},
		{244,     0,      58},
		{241,     0,      50},
		{239,     0,      42},
		{236,     0,      33},
		{232,     0,      25},
		{229,     0,      17},
		{225,     0,      8},
		{221,     0,      0},
		{225,     8,      0},
		{229,     17,     0},
		{232,     25,     0},
		{236,     33,     0},
		{239,     42,     0},
		{241,     50,     0},
		{244,     58,     0},
		{246,     66,     0},
		{248,     74,     0},
		{250,     82,     0},
		{252,     90,     0},
		{253,     98,     0},
		{254,     105,    0},
		{254,     113,    0},
		{255,     120,    0},
		{255,     127,    0},
		{255,     135,    0},
		{254,     142,    0},
		{254,     149,    0},
		{253,     155,    0},
		{252,     162,    0},
		{250,     168,    0},
		{248,     174,    0},
		{246,     180,    0},
		{244,     186,    0},
		{241,     192,    0},
		{239,     197,    0},
		{236,     202,    0},
		{232,     207,    0},
		{229,     212,    0},
		{225,     217,    0},
		{221,     221,    0},
		{217,     225,    0},
		{212,     229,    0},
		{207,     232,    0},
		{202,     236,    0},
		{197,     239,    0},
		{192,     241,    0},
		{186,     244,    0},
		{180,     246,    0},
		{174,     248,    0},
		{168,     250,    0},
		{162,     252,    0},
		{155,     253,    0},
		{149,     254,    0},
		{142,     254,    0},
		{135,     255,    0},
		{128,     255,    0},
		{120,     255,    0},
		{113,     254,    0},
		{105,     254,    0},
		{98,      253,    0},
		{90,      252,    0},
		{82,      250,    0},
		{74,      248,    0},
		{66,      246,    0},
		{58,      244,    0},
		{50,      241,    0},
		{42,      239,    0},
		{33,      236,    0},
		{25,      232,    0},
		{17,      229,    0},
		{8,       225,    0},
		{0,       221,    0},
		{0,       225,    8},
		{0,       229,    17},
		{0,       232,    25},
		{0,       236,    33},
		{0,       239,    42},
		{0,       241,    50},
		{0,       244,    58},
		{0,       246,    66},
		{0,       248,    74},
		{0,       250,    82},
		{0,       252,    90},
		{0,       253,    98},
		{0,       254,    105},
		{0,       254,    113},
		{0,       255,    120},
		{0,       255,    128},
		{0,       255,    135},
		{0,       254,    142},
		{0,       254,    149},
		{0,       253,    155},
		{0,       252,    162},
		{0,       250,    168},
		{0,       248,    174},
		{0,       246,    180},
		{0,       244,    186},
		{0,       241,    192},
		{0,       239,    197},
		{0,       236,    202},
		{0,       232,    207},
		{0,       229,    212},
		{0,       225,    217},
		{0,       221,    221},
		{0,       217,    225},
		{0,       212,    229},
		{0,       207,    232},
		{0,       202,    236},
		{0,       197,    239},
		{0,       192,    241},
		{0,       186,    244},
		{0,       180,    246},
		{0,       174,    248},
		{0,       168,    250},
		{0,       162,    252},
		{0,       155,    253},
		{0,       149,    254},
		{0,       142,    254},
		{0,       135,    255},
		{0,       128,    255},
		{0,       120,    255},
		{0,       113,    254},
		{0,       105,    254},
		{0,       98,     253},
		{0,       90,     252},
		{0,       82,     250},
		{0,       74,     248},
		{0,       66,     246},
		{0,       58,     244},
		{0,       50,     241},
		{0,       42,     239},
		{0,       33,     236},
		{0,       25,     232},
		{0,       17,     229},
		{0,       8,      225}
};

// Set PWM
void PMSM_SetPWM_UVW(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3)
{

		TIM1->CCR1 = PWM1;
		TIM1->CCR2 = PWM2;
		TIM1->CCR3 = PWM3;

}


int main(void)
{

	clockConfig();

	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/100);

	TIM1Config();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER6_0;

	while(1)
		{

			TB_Wait(500);
			GPIOA->ODR ^= GPIO_ODR_OD6;
		}

}

void SysTick_Handler(void)
{

	uint16_t PWM1, PWM2, PWM3;

	// Calculate PWM for 3-phase
			PWM1 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][0]/255);
			PWM2 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][1]/255);
			PWM3 = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][2]/255);


			if (PMSM_MotorSpin == PMSM_CW) {
				// Forward rotation
				PMSM_SetPWM_UVW(PWM1, PWM2, PWM3);
			}
			else {
				// Backward rotation
				PMSM_SetPWM_UVW(PWM1, PWM3, PWM2);
			}

			// Increment position in sine table
			PMSM_SinTableIndex++;
			if (PMSM_SinTableIndex > PMSM_SINTABLESIZE-1) {
				PMSM_SinTableIndex = 0;
			}



	if (hTimebase_500us != 0)
	{
		hTimebase_500us --;
	}
}
