/*
 * rtc.c
 *
 *  Created on: 20 mai 2018
 *      Author: Laurent
 */

#include "stm32l4xx.h"
#include "rtc.h"

// Private functions
static uint8_t byte2bcd	(uint8_t byte);
static uint8_t bcd2byte	(uint8_t bcd);

// Global variables
rtcc_t 		now, alarm;


/*
 * BSP_RTC_Set(...)
 *
 * Set RTC date and time
 * param: 	pointers to date and time data structures
 * retval: 	none
 */

void BSP_RTC_Set(rtcc_t *pnow)
{
	uint32_t	bcddate, bcdtime;

	// Convert time into BCD format
	bcdtime = 	( (byte2bcd(pnow->hours))   <<16U) |
				( (byte2bcd(pnow->minutes)) <<8U)  |
				( (byte2bcd(pnow->seconds)) <<0U);

	// Convert date into BCD format
	bcddate = 	( (byte2bcd(pnow->year )) <<16U) |
				( (byte2bcd(pnow->month)) <<8U)  |
				( (byte2bcd(pnow->day  )) <<0U);

	bcddate |=  (0x03 <<13U); 	// It's always Sunday (don't care) !

	// Disable RTC registers writing protection
	PWR->CR1 |= PWR_CR1_DBP;

	// Enter key
	*(__IO uint8_t *)&RTC->WPR = 0xCA;
	*(__IO uint8_t *)&RTC->WPR = 0x53;

	// Enter INIT mode
	RTC->ISR |= RTC_ISR_INIT;

	// Wait for INIT mode ready flag
	while((RTC->ISR & RTC_ISR_INITF) == RESET);

	// Setup prescalers for 1s RTC clock
	RTC->PRER = 0x007F00FF;

	// Setup date & time
	RTC->TR = bcdtime;
	RTC->DR = bcddate;

	// Exit INIT mode
	RTC->ISR &= ~RTC_ISR_INIT;

	// Disable Write access for RTC registers
	RTC->WPR = 0xFF;
}


/*
 * BSP_RTC_Get(...)
 *
 * Retreive current RTC date & time in binary format
 */

void BSP_RTC_Get(rtcc_t *pnow)
{
	uint32_t time, date;

	time = RTC->TR;
	date = RTC->DR;

	pnow->year = 	bcd2byte( (date & (RTC_DR_YT_Msk  | RTC_DR_YU_Msk )) >>RTC_DR_YU_Pos );
	pnow->month = 	bcd2byte( (date & (RTC_DR_MT_Msk  | RTC_DR_MU_Msk )) >>RTC_DR_MU_Pos );
	pnow->day = 	bcd2byte( (date & (RTC_DR_DT_Msk  | RTC_DR_DU_Msk )) >>RTC_DR_DU_Pos );

	pnow->hours = 	bcd2byte( (time & (RTC_TR_HT_Msk  | RTC_TR_HU_Msk )) >>RTC_TR_HU_Pos );
	pnow->minutes = bcd2byte( (time & (RTC_TR_MNT_Msk | RTC_TR_MNU_Msk)) >>RTC_TR_MNU_Pos);
	pnow->seconds = bcd2byte( (time & (RTC_TR_ST_Msk  | RTC_TR_SU_Msk )) >>RTC_TR_SU_Pos );
}


/*
 * Wait for RTC synchronization
 */

uint8_t BSP_RTC_WaitForSynchro(void)
{
	uint32_t	timeout;

	// Enter key
	*(__IO uint8_t *)&RTC->WPR = 0xCA;
	*(__IO uint8_t *)&RTC->WPR = 0x53;

	// Clear RSF flag
	RTC->ISR &= (uint32_t)RTC_ISR_RSF_Msk;

	// Disable Write access for RTC registers
	RTC->WPR = 0xFF;

	timeout = 10000000;

	// Wait until synchronization is done
	while( ((RTC->ISR & RTC_ISR_RSF_Msk) == 0) && (timeout > 0) ) timeout--;

	if (timeout ==0) return 1;
	else return 0;
}


/*
 * BSP_RTC_SetAlarmA(time_t *ptime)
 */

void BSP_RTC_SetAlarmA(rtcc_t *palarm)
{
	uint32_t bcdtime;

	bcdtime = ( (byte2bcd(palarm->hours))   <<16U) |
			  ( (byte2bcd(palarm->minutes)) <<8U)  |
			  ( (byte2bcd(palarm->seconds)) <<0U);

	// Enable Write access for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// Disable Alarm A
	RTC->CR &= ~RTC_CR_ALRAE;

	// Set Alarm time and discard date matching
	RTC->ALRMAR = RTC_ALRMAR_MSK4 | bcdtime;

	// Enable Alarm A and associated interrupt
	RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE;

	// Disable Write access for RTC registers
	RTC->WPR = 0xFF;
}

void BSP_RTC_SetAlarmA2(rtcc_t *palarm)
{
	uint32_t bcdtime;

	bcdtime = ( (byte2bcd(palarm->hours))   <<16U) |
			  ( (byte2bcd(palarm->minutes)) <<8U)  |
			  ( (byte2bcd(palarm->seconds)) <<0U);

	bcdtime =bcdtime+byte2bcd(70);

	while((bcdtime&0x000000ff)>=byte2bcd(60)){
		bcdtime=bcdtime-byte2bcd(60)+(1<<8U);
	}
	while((bcdtime&0x0000ff00)>=(byte2bcd(60)<<8U)){
		bcdtime=bcdtime-(byte2bcd(60)<<8U)+(1<<16U);
	}
	while((bcdtime&0x00ff0000)>=(byte2bcd(24)<<16U)){
		bcdtime=bcdtime-(byte2bcd(24)<<16U);
	}


	// Enable Write access for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// Disable Alarm A
	RTC->CR &= ~RTC_CR_ALRAE;

	// Set Alarm time and discard date matching
	RTC->ALRMAR = RTC_ALRMAR_MSK4 | bcdtime;

	// Enable Alarm A and associated interrupt
	RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE;

	// Disable Write access for RTC registers
	RTC->WPR = 0xFF;
}


/*
 * BSP_RTC_EXTI_Init()
 * Enable RTC (EXTI18) rising edge Interrupt
 */

void BSP_RTC_EXTI_Init()
{
	// Enable EXTI line 18
	EXTI->IMR1 |= EXTI_IMR1_IM18;

	// Enable Rising / Disable Falling trigger
	EXTI->RTSR1 |= EXTI_RTSR1_RT18;
	EXTI->FTSR1 &= ~EXTI_FTSR1_FT18;
}


/*
 * Compute FatFs compliant timestamp
 */

uint32_t BSP_RTC_GetFatTime()
{
	uint32_t	timestamp;

	uint8_t		day, month, year;
	uint8_t		hours, minutes, seconds;

	timestamp = 0x00000000;

	day		= (((RTC->DR & RTC_DR_DT_Msk ) >>RTC_DR_DT_Pos ) * 10) + (((RTC->DR & RTC_DR_DU_Msk ) >>RTC_DR_DU_Pos ));
	month   = (((RTC->DR & RTC_DR_MT_Msk ) >>RTC_DR_MT_Pos ) * 10) + (((RTC->DR & RTC_DR_MU_Msk ) >>RTC_DR_MU_Pos ));
	year	= (((RTC->DR & RTC_DR_YT_Msk ) >>RTC_DR_YT_Pos ) * 10) + (((RTC->DR & RTC_DR_YU_Msk ) >>RTC_DR_YU_Pos ));

	timestamp |= ((uint8_t)(2000 + year - 1980) <<25U);			// Year
	timestamp |= ((uint8_t) month 				<<21U);			// Month
	timestamp |= ((uint8_t) day					<<16U);			// Day

	hours   = (((RTC->TR & RTC_TR_HT_Msk ) >>RTC_TR_HT_Pos ) * 10) + (((RTC->TR & RTC_TR_HU_Msk ) >>RTC_TR_HU_Pos ));
	minutes = (((RTC->TR & RTC_TR_MNT_Msk) >>RTC_TR_MNT_Pos) * 10) + (((RTC->TR & RTC_TR_MNU_Msk) >>RTC_TR_MNU_Pos));
	seconds = (((RTC->TR & RTC_TR_ST_Msk ) >>RTC_TR_ST_Pos ) * 10) + (((RTC->TR & RTC_TR_SU_Msk ) >>RTC_TR_SU_Pos ));

	timestamp |= ((uint8_t)hours 	<<11U);			// Hours
	timestamp |= ((uint8_t)minutes 	<<05U);			// Minutes
	timestamp |= ((uint8_t)seconds 	<<00U);			// Seconds

	return timestamp;
}


/*
 * Converts 2 digit Decimal to BCD format
 * param: 	Byte to be converted.
 * retval: BCD Converted byte
 */

static uint8_t byte2bcd(uint8_t byte)
{
  uint8_t bcdhigh = 0;

  while (byte >= 10)
  {
    bcdhigh++;
    byte -= 10;
  }

  return  ((uint8_t)(bcdhigh << 4) | byte);
}


/*
 * Convert from 2 digit BCD to Binary
 * param  BCD value to be converted.
 * retval Converted word
 */

static uint8_t bcd2byte(uint8_t bcd)
{
  uint8_t tmp = 0;
  tmp = ((uint8_t)(bcd & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (bcd & (uint8_t)0x0F));
}



