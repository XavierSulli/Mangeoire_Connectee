/*
 * main.h
 *
 *  Created on: 29 nov. 2018
 *      Author: Laurent
 */

#ifndef APP_INC_MAIN_H_
#define APP_INC_MAIN_H_

//#define buffer_size 10000
#define buffer_HQ_size 40000


#include <stdint.h>
#include <inttypes.h>

#include "stm32l496xx.h" //symbols didn't work

#include "dcmi.h"
#include "sccb.h"
#include "ov2640.h"
#include "delay.h"
#include "dma2_dcmi.h"
#include "snapshot.h"

// FreeRTOS headers
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"

/* Global functions */

int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);


#endif /* APP_INC_MAIN_H_ */
