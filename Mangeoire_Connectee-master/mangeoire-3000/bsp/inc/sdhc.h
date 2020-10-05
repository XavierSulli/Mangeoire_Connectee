/*
 * sdhc.h
 *
 *  Created on: 1 déc. 2018
 *      Author: Laurent
 */

#ifndef BSP_INC_SDHC_H_
#define BSP_INC_SDHC_H_


#include <stm32l4xx.h>
#include "main.h"


/* SDMMC Commands Defines */

#define		CMD0_GO_IDLE_STATE				0
#define		CMD2_ALL_SEND_CID				2
#define		CMD3_SEND_RELATIVE_ADDR			3
#define		CMD7_SELECT_DESELECT_CARD		7
#define		CMD8_SEND_IF_COND				8
#define		CMD9_SEND_CSD					9
#define		CMD12_STOP_TRANSMISSION			12
#define		CMD13_SEND_STATUS				13
#define		CMD16_SET_BLOCKLEN				16
#define		CMD17_READ_SINGLE_BLOCK			17
#define		CMD18_READ_MULTIPLE_BLOCK		18
#define		CMD23_SET_BLOCK_COUNT			23
#define		CMD24_WRITE_SINGLE_BLOCK		24
#define		CMD25_WRITE_MULTIPLE_BLOCK		25
#define		CMD55_APP_CMD					55
#define		CMD58_READ_OCR					58

#define		ACMD6_SET_BUS_WIDTH				6
#define		ACMD13_SD_STATUS				13
#define		ACMD23_SET_WR_BLK_ERASE_COUNT	23
#define		ACMD41_SD_SEND_OP_COND			41


typedef struct
{
	uint8_t	 command_index;
	uint32_t arg;
	uint8_t	 crc;
	uint8_t  resp_length;
	uint8_t	 resp[6];
} sdhc_command_t;


/* Results of SDMMC Functions */
typedef enum
{	SD_OK 		= 0,
	SD_TIMEOUT 	= 1,
	SD_ERROR 	= 2
} SDRESULT;


/* Functions */

void 		BSP_SDHC_SPI_Init			(void);

SDRESULT 	BSP_SDHC_Init				(void);
SDRESULT    BSP_SDHC_Read				(uint8_t *buffer, uint32_t addr, uint32_t nb);
SDRESULT    BSP_SDHC_Write				(uint8_t *buffer, uint32_t addr, uint32_t nb);


SDRESULT	BSP_SDHC_SendCommand		(sdhc_command_t *sdc);



#endif /* BSP_INC_SDHC_H_ */
