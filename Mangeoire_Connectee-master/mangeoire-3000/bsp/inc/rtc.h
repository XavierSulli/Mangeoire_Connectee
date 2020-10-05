/*
 * rtc.h
 *
 *  Created on: 20 mai 2018
 *      Author: Laurent
 */

#ifndef BSP_INC_RTC_H_
#define BSP_INC_RTC_H_


// Data structure for time & date

typedef struct
{
	uint8_t 	day;
	uint8_t 	month;
	uint8_t		year;
	uint8_t 	hours;
	uint8_t 	minutes;
	uint8_t		seconds;
} rtcc_t;


// Public functions

void 		BSP_RTC_Set				(rtcc_t *pnow);
void 		BSP_RTC_Get				(rtcc_t *pnow);

uint8_t		BSP_RTC_WaitForSynchro	(void);
uint32_t 	BSP_RTC_GetFatTime		(void);

void 		BSP_RTC_EXTI_Init		(void);
void 		BSP_RTC_SetAlarmA		(rtcc_t *palarm);
void 		BSP_RTC_SetAlarmA2		(rtcc_t *palarm);

// Public functions

void BSP_RTC_Clock_Config	(void);
void BSP_RTC_EXTI_Init		(void);

//void BSP_RTC_SetTime		(time_t *ptime);
//void BSP_RTC_GetTime		(time_t *ptime);


#endif /* BSP_INC_RTC_H_ */
