/**
 ******************************************************************************
 * File Name bluenrg_conf.h
 * @author   CL
 * @version  V1.0.0
 * @date     05-Mar-2018
 * @brief 
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLUENRG_CONF_H
#define __BLUENRG_CONF_H

#include "stm32l4xx.h"
#include <string.h>

#define DEBUG      			 		1			// Print messages from files at user level
#define PRINT_CSV_FORMAT      		0			// Print the data travelling over the SPI in the .csv format for the GUI
#define HCI_READ_PACKET_SIZE      	128			// Number of Bytes reserved for HCI Read Packet
#define HCI_MAX_PAYLOAD_SIZE      	128			// Number of Bytes reserved for HCI Max Payload
#define SCAN_P      				16384		// Scan Interval: time interval from when the Controller started its last scan until it begins the subsequent scan 	(for a number N, Time = N x 0.625 msec)
#define SCAN_L      				16384		// Scan Window: amount of time for the duration of the LE scan 														(for a number N, Time = N x 0.625 msec)
#define SUPERV_TIMEOUT      		60			// Supervision Timeout for the LE Link 																				(for a number N, Time = N x 10 msec)
#define CONN_P1      				40			// Minimum Connection Period 																						(for a number N, Time = N x 1.25 msec)
#define CONN_P2      				40			// Maximum Connection Period 																						(for a number N, Time = N x 1.25 msec)
#define CONN_L1      				2000		// Minimum Connection Length 																						(for a number N, Time = N x 0.625 msec)
#define CONN_L2      				2000		// Maximum Connection Length 																						(for a number N, Time = N x 0.625 msec)
#define ADV_DATA_TYPE      			ADV_IND		// Advertising Type
#define ADV_INTERV_MIN      		2048		// Minimum Advertising Interval 																					(for a number N, Time = N x 0.625 msec)
#define ADV_INTERV_MAX      		4096		// Maximum Advertising Interval 																					(for a number N, Time = N x 0.625 msec)
#define L2CAP_INTERV_MIN      		9			// Minimum Connection Event Interval 																				(for a number N, Time = N x 1.25 msec)
#define L2CAP_INTERV_MAX      		20			// Maximum Connection Event Interval 																				(for a number N, Time = N x 1.25 msec)
#define L2CAP_TIMEOUT_MULTIPLIER	600			// Timeout Multiplier 																								(for a number N, Time = N x 10 msec)
#define HCI_DEFAULT_TIMEOUT_MS    	1000

#define BLUENRG_memcpy                memcpy
#define BLUENRG_memset                memset
  

#endif /*__BLUENRG_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
