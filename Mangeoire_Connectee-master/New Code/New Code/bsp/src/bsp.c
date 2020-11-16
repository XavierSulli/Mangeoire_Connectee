/*
 * bsp.c
 *
 *  Created on: 29 nov. 2018
 *      Author: Laurent
 */


#include <bsp.h>

/*
 * BSP_LED_Init()
 * Initialize LED pin  as  High-Speed Push-Pull Output
 * Set LED initial state to OFF
 */

void BSP_LED_Init()
{
	// Enable GPIOA clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure PA10 as output
	GPIOA->MODER &= ~GPIO_MODER_MODE10_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODE10_Pos);

	// Configure PA10 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_10;

	// Configure PA10 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED10_Pos);

	// Disable PA10 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10_Msk;

	// Set Initial State OFF
	GPIOA->BSRR = GPIO_BSRR_BR_10;



	// Configure PB7, PB14 as output
	GPIOA->MODER &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE12_Msk);
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODE11_Pos) | (0x01 <<GPIO_MODER_MODE12_Pos);

	// Configure PB7, PB14 as Push-Pull output
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_12);

	// Configure PB7, PB14 as High-Speed Output
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED11_Msk | GPIO_OSPEEDR_OSPEED12_Msk);
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED11_Pos) | (0x03 <<GPIO_OSPEEDR_OSPEED12_Pos);

	// Disable PB7, PB14 Pull-up/Pull-down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD11_Msk | GPIO_PUPDR_PUPD12_Msk);

	// Set Initial State OFF
	GPIOA->BSRR = GPIO_BSRR_BR_11;
	GPIOA->BSRR = GPIO_BSRR_BR_12;
}



/*
 * BSP_LED_On()
 * Turn ON LED on PC7
 */

void BSP_LED_On(uint8_t id)
{
	switch (id)
	{
		case 0:
		{
			GPIOA->BSRR = GPIO_BSRR_BS10;
			break;
		}

		case 1:
		{
			GPIOA->BSRR = GPIO_BSRR_BS11;
			break;
		}

		case 2:
		{
			GPIOA->BSRR = GPIO_BSRR_BS12;
			break;
		}
		case 3:
		{
			GPIOA->BSRR = GPIO_BSRR_BS10;
			GPIOA->BSRR = GPIO_BSRR_BS11;
			GPIOA->BSRR = GPIO_BSRR_BS12;
			break;
		}
	}
}

/*
 * BSP_LED_Off()
 * Turn OFF LED on PA5
 */

void BSP_LED_Off(uint8_t id)
{
	switch (id)
	{
		case 0:
		{
			GPIOA->BSRR = GPIO_BSRR_BR10;
			break;
		}

		case 1:
		{
			GPIOA->BSRR = GPIO_BSRR_BR11;
			break;
		}

		case 2:
		{
			GPIOA->BSRR = GPIO_BSRR_BR12;
			break;
		}
		case 3:
		{
			GPIOA->BSRR = GPIO_BSRR_BR10;
			GPIOA->BSRR = GPIO_BSRR_BR11;
			GPIOA->BSRR = GPIO_BSRR_BR12;
			break;
		}
	}
}

/*
 * BSP_LED_Toggle()
 * Toggle LED on PA5
 */

void BSP_LED_Toggle(uint8_t id)
{
	switch (id)
	{
		case 0:
		{
			GPIOA->ODR ^= GPIO_ODR_ODR_10;
			break;
		}

		case 1:
		{
			GPIOA->ODR ^= GPIO_ODR_ODR_11;
			break;
		}

		case 2:
		{
			GPIOA->ODR ^= GPIO_ODR_ODR_12;
			break;
		}
	}
}

/*
 * BSP_PB_Init()
 * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
 */

void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODE13_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD13_Msk;
}

/*
 * BSP_PB_GetState()
 * Returns the state of the button (0=released, 1=pressed)
 */

uint8_t	BSP_PB_GetState()
{
	uint8_t state;

	if ((GPIOC->IDR & GPIO_IDR_ID13) == GPIO_IDR_ID13)
	{
		state = 1;
	}
	else
	{
		state = 0;
	}

	return state;
}

/*
 * BSP_Console_Init()
 * LPUART1 @ 115200 Full Duplex
 * 1 start - 8-bit - 1 stop
 * TX -> PG7 (AF8)
 * RX -> PG8 (AF8)
 */

void BSP_Console_Init()
{
	// Enable GPIOG clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;

	// Enable VDDIO2 (for use GPIOG)
	PWR->CR2 |= PWR_CR2_IOSV;

	// Configure PG7 and PG8 as Alternate function
	GPIOG->MODER &= ~(GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk);
	GPIOG->MODER |=  (0x02 <<GPIO_MODER_MODE7_Pos) | (0x02 <<GPIO_MODER_MODE8_Pos);

	// Setup output Push-Pull type
	GPIOG->OTYPER &= ~(GPIO_OTYPER_ODR_7 | GPIO_OTYPER_ODR_8);

	// Setup Speed to high-speed
	GPIOG->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8);
	GPIOG->OSPEEDR |=  (0x02 <<GPIO_OSPEEDR_OSPEED7_Pos) | (0x02 <<GPIO_OSPEEDR_OSPEED7_Pos);

	// Set PG7 and PG8 to AF8 (LPUART1)
	GPIOG->AFR[0] &= ~(0xF0000000);
	GPIOG->AFR[0] |=  (0x80000000);
	GPIOG->AFR[1] &= ~(0x0000000F);
	GPIOG->AFR[1] |=  (0x00000008);

	// Select SYSCLK (80MHz) as clock source
	RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL;
	RCC->CCIPR |=  0x01 <<RCC_CCIPR_LPUART1SEL_Pos;

	// Enable LPUART1 clock
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

	// Clear LPUART1 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	LPUART1->CR1 = 0x00000000;
	LPUART1->CR2 = 0x00000000;
	LPUART1->CR3 = 0x00000000;

	// Baud Rate = 115200 = 256*80MHz / BRR
	LPUART1->CR1 &= ~USART_CR1_OVER8;
	LPUART1->BRR = 0x02B672;

	// Enable both Transmitter and Receiver
	LPUART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable LPUART1
	LPUART1->CR1 |= USART_CR1_UE;
}

void BSP_ADC_Init()
{
	// Enable GPIOA clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure pin PA0 as analog
	GPIOA->MODER &= ~GPIO_MODER_MODE0_Msk;
	GPIOC->MODER |= (0x03 <<GPIO_MODER_MODE0_Pos);



	// Enable ADC clock
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

	// Reset ADC configuration
	ADC2->CR 	= 0x00000000;
	ADC2->CFGR  = 0x00000000;
	ADC2->CFGR2 = 0x00000000;
	ADC2->SQR1  = 0x00000000;
	ADC2->SQR2  = 0x00000000;
	ADC2->SQR3  = 0x00000000;
	ADC2->SQR4  = 0x00000000;

	// Enable continuous conversion mode
	ADC2->CFGR |= ADC_CFGR_CONT;

	// 12-bit resolution
	ADC2->CFGR |= (0x00 <<ADC_CFGR_RES_Pos);

	// Select PCLK/2 as ADC clock
	RCC->CCIPR |= (0x03 << RCC_CCIPR_ADCSEL_Pos);

	// Set sampling time to 24.5 ADC clock cycles channel 5 of PA0
	ADC2->SMPR1 |= (0x03 << ADC_SMPR1_SMP5_Pos) ;

	// Select channel & number of conversion
	ADC2->SQR1 &= ~ADC_SQR1_L; // reset value of ADC
	ADC2->SQR1 |= (0x00 << ADC_SQR1_L_Pos);// set to 1 conversion
	ADC2->SQR1 &= ~ADC_SQR1_SQ1;// reset of SQ1
	ADC2->SQR1 |= (0x05 << ADC_SQR1_SQ1_Pos);// channel 5 selection

	// Enable ADC
	ADC2->ISR |= ADC_ISR_ADRDY;
	ADC2->CR |= ADC_CR_ADEN;

}


/*
 * BSP_PC4_Init()
 * Initialize SD_PWR pin (PC4) as a High-Speed Push-Pull output
 *
 */

void BSP_PC4_Init()
{
	// Enable GPIOC clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC4 as output
	GPIOC->MODER &= ~GPIO_MODER_MODE4_Msk;
	GPIOC->MODER |= (0x01 <<GPIO_MODER_MODE4_Pos);

	// Configure PC4 as Push-Pull output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_4;

	// Configure PC4 as High-Speed Output
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_Msk;
	GPIOC->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED4_Pos);

	// Disable PC4 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD4_Msk;

	// Set Initial State OFF
	GPIOC->BSRR |= GPIO_BSRR_BR_4;
}

void BSP_PC4_On()
{
GPIOC->BSRR |= GPIO_BSRR_BS4;

}

void BSP_PC4_Off()
{
GPIOC->BSRR |= GPIO_BSRR_BR4;

}
/*
 * BSP_PC2_Init()
 * Initialize SD_PWR pin (PC2) as a High-Speed Push-Pull output
 *
 */

void BSP_PC2_Init()
{
	// Enable GPIOC clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC2 as output
	GPIOC->MODER &= ~GPIO_MODER_MODE2_Msk;
	GPIOC->MODER |= (0x01 <<GPIO_MODER_MODE2_Pos);

	// Configure PC2 as Push-Pull output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_2;

	// Configure PC2 as High-Speed Output
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_Msk;
	GPIOC->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED2_Pos);

	// Disable PC2 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD2_Msk;

	// Set Initial State OFF
	GPIOC->BSRR |= GPIO_BSRR_BR_2;
}

void BSP_PC2_On()
{
GPIOC->BSRR |= GPIO_BSRR_BS2;

}

void BSP_PC2_Off()
{
GPIOC->BSRR |= GPIO_BSRR_BR2;

}

/*
 * BSP_PC3_Init()
 * Initialize CAM_PWR pin (PC3) as a High-Speed Push-Pull output
 *
 */

void BSP_PC3_Init()
{
	// Enable GPIOC clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC3 as output
	GPIOC->MODER &= ~GPIO_MODER_MODE3_Msk;
	GPIOC->MODER |= (0x01 <<GPIO_MODER_MODE3_Pos);

	// Configure PC3 as Push-Pull output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_3;

	// Configure PC3 as High-Speed Output
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3_Msk;
	GPIOC->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED3_Pos);

	// Disable PC3 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD3_Msk;

	// Set Initial State OFF
	GPIOC->BSRR |= GPIO_BSRR_BR_3;
}

void BSP_PC3_On()
{
GPIOC->BSRR |= GPIO_BSRR_BS3;

}

void BSP_PC3_Off()
{
GPIOC->BSRR |= GPIO_BSRR_BR3;

}
/*
 * BSP_PC5_Init()
 * Initialize BLE_PWR pin (PC5) as a High-Speed Push-Pull output
 *
 */

void BSP_PC5_Init()
{
	// Enable GPIOC clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC5 as output
	GPIOC->MODER &= ~GPIO_MODER_MODE5_Msk;
	GPIOC->MODER |= (0x01 <<GPIO_MODER_MODE5_Pos);

	// Configure PC5 as Push-Pull output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_5;

	// Configure PC5 as High-Speed Output
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_Msk;
	GPIOC->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED5_Pos);

	// Disable PC5 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD5_Msk;

	// Set Initial State OFF
	GPIOC->BSRR |= GPIO_BSRR_BR_5;
}

void BSP_PC5_On()
{
GPIOC->BSRR |= GPIO_BSRR_BS5;

}

void BSP_PC5_Off()
{
GPIOC->BSRR |= GPIO_BSRR_BR5;

}

/*
 * BSP_PC5_Init()
 * Initialize BLE_PWR pin (PC5) as a High-Speed Push-Pull output
 *
 */

void BSP_PB1_Init()
{
	// Enable GPIOC clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PB1 as output
	GPIOB->MODER &= ~GPIO_MODER_MODE1_Msk;
	GPIOB->MODER |= (0x01 <<GPIO_MODER_MODE1_Pos);

	// Configure PB1 as Push-Pull output
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_1;

	// Configure PC5 as High-Speed Output
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
	GPIOB->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED1_Pos);

	// Disable PC5 Pull-up/Pull-down
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;

	// Set Initial State OFF
	GPIOB->BSRR |= GPIO_BSRR_BR_1;
}

void BSP_PB1_On()
{
GPIOB->BSRR |= GPIO_BSRR_BS1;

}

void BSP_PB1_Off()
{
GPIOB->BSRR |= GPIO_BSRR_BR1;

}

/*
 * BSP_PA1_Init()
 * Initialize WEIGHING_PWR pin (PA1) as a High-Speed Push-Pull output
 *
 */

void BSP_PA1_Init()
{
	// Enable GPIOA clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure PA1 as output
	GPIOA->MODER &= ~GPIO_MODER_MODE1_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODE1_Pos);

	// Configure PA1 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_1;

	// Configure PA1 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED1_Pos);

	// Disable PA1 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;

	// Set Initial State OFF
	GPIOA->BSRR |= GPIO_BSRR_BR_1;
}

void BSP_PA1_On()
{
GPIOA->BSRR |= GPIO_BSRR_BS1;

}

void BSP_PA1_Off()
{
GPIOA->BSRR |= GPIO_BSRR_BR1;

}

/*
 * BSP_PA9_Init()
 * Initialize gps_reset pin (PA9) as a High-Speed Push-Pull output
 *
 */

void BSP_PA9_Init()
{
	// Enable GPIOA clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure PA9 as output
	GPIOA->MODER &= ~GPIO_MODER_MODE9_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODE9_Pos);

	// Configure PA9 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;

	// Configure PA9 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEED9_Pos);

	// Disable PA9 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk;

	// Set Initial State OFF
	GPIOA->BSRR |= GPIO_BSRR_BR_9;
}

void BSP_PA9_On()
{
GPIOA->BSRR |= GPIO_BSRR_BS9;

}

void BSP_PA9_Off()
{
GPIOA->BSRR |= GPIO_BSRR_BR9;

}


void BSP_NVIC_Init()
{


	// Set priority level 1 for RTC interrupt
	//NVIC_SetPriority(RTC_Alarm_IRQn, 1);

	// Enable RTC interrupts
	//NVIC_EnableIRQ(RTC_Alarm_IRQn);

	// Set  priority 2 for LPUART1 interrupts
	//NVIC_SetPriority(LPUART1_IRQn, 2);

	// Enable EXTI LPUART1 interrupts
	//NVIC_EnableIRQ(LPUART1_IRQn);
}
