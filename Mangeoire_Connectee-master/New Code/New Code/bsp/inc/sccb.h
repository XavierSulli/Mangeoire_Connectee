/*
 * sccb.h
 *
 *  Created on: 15 oct. 2018
 *      Author: alexandre.ferroni
 */

#ifndef BSP_INC_SCCB_H_
#define BSP_INC_SCCB_H_

#include "main.h"

#define I2C_TIMEOUT_MAX		1000
#define I2C3_TIMING			0xFF10B3FF//0xA010B3FF//0x00F0DBF7 //0x20303E5D  //10khz

#define SCCB_ADDRESS			 0x30 //may be 0x60    //0x77 //testbmp180
#define SCCB_SLAVE_ADDRESS7      0xFE //own address 1

//may not be used
#define SCCB_WRITE_ADDRESS	0x60
#define SCCB_READ_ADDRESS	0x61

void sccb_mco();
void sccb_init();
uint8_t sccb_ReadNReg(uint8_t addr, uint8_t *buffer);
uint8_t sccb_WriteNReg(uint8_t addr, uint8_t data);


#endif /* BSP_INC_SCCB_H_ */
