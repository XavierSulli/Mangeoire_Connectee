/*
 * delay.c
 *
 *  Created on: 6 août 2017
 *      Author: Laurent
 */

#include "delay.h"

volatile uint32_t uwTick;


/*
 *  Basic delay functions
 */

void DELAY_TIM_init(void)
{
	// Enable TIM6 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

	// Reset TIM6 configuration
	TIM6->CR1 = 0x0000;
	TIM6->CR2 = 0x0000;

	// Set TIM6 prescaler
	// Fck = 80MHz -> /80 = 1MHz counting frequency -> 1us
	TIM6->PSC = (uint16_t) 80 -1;

	// Set ARR to maximum value
	TIM6->ARR = (uint16_t) 0xFFFF;
}

void DELAY_TIM2_init(void)
{
	// Enable TIM2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Reset TIM2 configuration
	TIM2->CR1 = 0x0000;
	TIM2->CR2 = 0x0000;

	// Set TIM2 prescaler
	// Fck = 80MHz -> /8000 = 10KHz counting frequency -> 0.1ms
	TIM2->PSC = (uint16_t) 8000 -1;

	// Set TIM2 auto-reload register for 1ms
	TIM2->ARR = (uint16_t) 10 -1;

	// Enable auto-reload preload
	TIM2->CR1 |= TIM_CR1_ARPE;

	// Enable Interrupt upon Update Event
	TIM2->DIER |= TIM_DIER_UIE;

	TIM2->EGR|= TIM_EGR_UG;
	// Start TIM2 counter
	TIM2->CR1 |= TIM_CR1_CEN;
}


/*
 * timer_delay_us(uint16_t us)
 * waits here for us
 */

void DELAY_TIM_us(uint16_t us)
{
	// Resets TIM6 counter
	TIM6->EGR |= TIM_EGR_UG;

	// Start TIM6 counter
	TIM6->CR1 |= TIM_CR1_CEN;

	// Wait until TIM6 counter reaches delay
	while(TIM6->CNT < us);

	// Stop TIM6 counter
	TIM6->CR1 &= ~TIM_CR1_CEN;
}

void DELAY_TIM_ms(uint16_t ms)
{
	uint16_t i = 0;
	for(i=0; i<ms; i++)
	{
		DELAY_TIM_us(1000);
	}
}

void delay_ms(uint32_t delay)
{
	uint32_t	i;
	for(i=0; i<(delay*6100); i++);		// Tuned for ms @
}

void delay_us(uint32_t delay)
{
	uint32_t	i;
	for(i=0; i<(delay*6); i++);			// Tuned for µs
}


/*
 * Systick based delay functions
 */

void BSP_IncTick(void)
{
  uwTick++;
}

uint32_t BSP_GetTick(void)
{
  return uwTick;
}

void BSP_Delay(uint32_t Delay)
{
  uint32_t tickstart = BSP_GetTick();
  uint32_t wait = Delay;

  /* Add a period to guaranty minimum wait */
  if (wait < 10000)
  {
    wait++;
  }

  while((BSP_GetTick() - tickstart) < wait)
  {
  }
}

void BSP_SuspendTick(void)
{
  /* Disable SysTick Interrupt */
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

void BSP_ResumeTick(void)
{
  /* Enable SysTick Interrupt */
  SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
}

void BSP_SuspendTIM2(void)
{
	// stop TIM2 counter
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

void BSP_ResumeTIM2(void)
{
	// Start TIM2 counter
	TIM2->CR1 |= TIM_CR1_CEN;
}


