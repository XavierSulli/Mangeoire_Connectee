/*
 * snapshot.c
 *
 *  Created on: 14 déc. 2018
 *      Author: alexandre.ferroni
 */

#include "snapshot.h"
//PC4 can be used to take a snapshot
//PC4 as input with an external pull-up resistor

void init_snap()
{
	// GPIOC clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~(GPIO_MODER_MODE13_Msk);
	// GPIO PUPDR : nothing
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13_Msk);

}

void snap_irq_init()
{
	// Enable SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Select Port C as interrupt source for EXTI line 13
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13_Msk;
	SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC;

	// Disable Rising / Enable Falling trigger
	EXTI->RTSR1 &= ~EXTI_RTSR1_RT13;
	EXTI->FTSR1 |= EXTI_FTSR1_FT13;

	// Enable EXTI line 13 interrupt;
	NVIC_SetPriority(EXTI15_10_IRQn, 4);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Pending interrupt flag on line 4
	EXTI->PR1 |= EXTI_PR1_PIF13;
	// Interrupt request from Line 4 is not masked
	EXTI->IMR1 |= EXTI_IMR1_IM13;
}

void snap_irq_enable()
{
	// Pending interrupt flag on line 4
	EXTI->PR1 |= EXTI_PR1_PIF13;
	// Interrupt request from Line 4 is not masked
	EXTI->IMR1 |= EXTI_IMR1_IM13;

	// Enable EXTI line 4 interrupt;
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
