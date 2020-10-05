/*
 * dcmi.h
 *
 *  Created on: 12 oct. 2018
 *      Author: alexandre.ferroni
 */

#ifndef BSP_INC_DCMI_H_
#define BSP_INC_DCMI_H_

#include "main.h"

void dcmi_init();
void dcmi_gpio_config();
void dcmi_it_en();

void exti_sync_init(void);
void exti_sync_start(void);

#endif /* BSP_INC_DCMI_H_ */
