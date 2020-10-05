/*
 * bsp.h
 *
 *  Created on: 29 nov. 2018
 *      Author: Laurent
 */

#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "stm32l4xx.h"

/*
 * LED driver functions
 */

#define	BLUE	0
#define WHITE	1
#define	RED		2
#define ALL		3

#define UART_BUFFER_SIZE		23

void	BSP_LED_Init	(void);
void	BSP_LED_On		(uint8_t id);
void	BSP_LED_Off		(uint8_t id);
void	BSP_LED_Toggle	(uint8_t id);


/*
 * Push-Button driver functions
 */

void	BSP_PB_Init		(void);
uint8_t	BSP_PB_GetState	(void);


/*
 * Debug Console driver functions
 */

void	BSP_Console_Init	(void);
void    BSP_PC4_Init        (void);
void    BSP_PC4_On          (void);
void    BSP_PC4_Off         (void);
void    BSP_PC2_Init        (void);
void    BSP_PC2_On          (void);
void    BSP_PC2_Off         (void);
void    BSP_PC5_Init        (void);
void    BSP_PC5_On          (void);
void    BSP_PC5_Off         (void);
void    BSP_PB1_Init        (void);
void    BSP_PB1_On          (void);
void    BSP_PB1_Off         (void);
void    BSP_PC3_Init        (void);
void    BSP_PC3_On          (void);
void    BSP_PC3_Off         (void);
void    BSP_PA1_Init        (void);
void    BSP_PA1_On          (void);
void    BSP_PA1_Off         (void);
void    BSP_PA9_Init        (void);
void    BSP_PA9_On          (void);
void    BSP_PA9_Off         (void);
void    BSP_ADC_Init(void);

void BSP_NVIC_Init				(void);

#endif /* BSP_INC_BSP_H_ */
