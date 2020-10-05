/*
 * delay.h
 *
 *  Created on: 6 août 2017
 *      Author: Laurent
 */

#ifndef BSP_INC_DELAY_H_
#define BSP_INC_DELAY_H_

#include "stm32l4xx.h"
/*
 * Timer delays
 */

void DELAY_TIM_init(void);
void DELAY_TIM2_init(void);
void DELAY_TIM_us(uint16_t us);
void DELAY_TIM_ms(uint16_t ms);

void		BSP_IncTick			(void);
uint32_t 	BSP_GetTick			(void);

void 		BSP_SuspendTick		(void);
void 		BSP_ResumeTick		(void);

void BSP_SuspendTIM2            (void);
void BSP_ResumeTIM2             (void);


void delay_ms	(uint32_t delay);
void delay_us	(uint32_t delay);

#endif /* BSP_INC_DELAY_H_ */
