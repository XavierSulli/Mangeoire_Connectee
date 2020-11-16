/*
 * main.h
 *
 *  Created on: 13 oct. 2020
 *      Author: Redouuu
 */

#ifndef BSP_INC_MAIN_H_
#define BSP_INC_MAIN_H_

//#define buffer_size 10000
#define buffer_HQ_size 40000

#include <stdint.h>
#include <inttypes.h>

#include "dcmi.h"
#include "sccb.h"
#include "ov2640.h"
#include "delay.h"
#include "dma2_dcmi.h"
#include "snapshot.h"

// Data structure for time & date


/* Global functions */

int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);

#endif /* BSP_INC_MAIN_H_ */
