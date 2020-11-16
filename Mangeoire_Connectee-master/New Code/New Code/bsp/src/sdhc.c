/*
 * sdhc.c
 *
 *  Created on: 1 déc. 2018
 *      Author: Laurent
 */

#include "sdhc.h"
#include "delay.h"


/*
 * Configure SPI
 *
 * SPI2_SCK  -> PB13 (AF5)
 * SPI2_MISO -> PB14 (AF5)
 * SPI2_MOSI -> PB15 (AF5)
 *
 * CS		 -> PB12
 */

void BSP_SDHC_SPI_Init()
{
	// Enable GPIOB clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Configure PB13, PB14, PB15 as Alternate function
	GPIOB->MODER &= ~(GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE15);
	GPIOB->MODER |=  (0x02 << 26U) | (0x02 << 28U) | (0x02 << 30U);

	// Set PB13, PB14, PB15 to AF5 (SPI2)
	GPIOB->AFR[1] &= ~(0xFFF00000);
	GPIOB->AFR[1] |=  (0x55500000);

	// Configure PB12 as output
	GPIOB->MODER &= ~(GPIO_MODER_MODE12);
	GPIOB->MODER |=  (0x01 << 24U);

	// Push-Pull outputs
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_12 | GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_15);

	// High-Speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15);
	GPIOB->OSPEEDR |=  (0x03 << 24U) | (0x03 << 26U) | (0x03 << 28U) | (0x03 << 30U);

	// Disable Pull-up/Pull-down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD12_Msk | GPIO_PUPDR_PUPD13_Msk | GPIO_PUPDR_PUPD14_Msk | GPIO_PUPDR_PUPD15_Msk);

	GPIOB->MODER &= ~(GPIO_MODER_MODE5);
	GPIOB->MODER |= (0x01 <<GPIO_MODER_MODE5_Pos);

	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_5);

	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5);
	GPIOB->OSPEEDR |=  (0x03 << GPIO_OSPEEDR_OSPEED5_Pos);

	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);

	GPIOB->BSRR = GPIO_BSRR_BS_5;

	delay_ms(100);

	// PA4 (CS) default state is high
	GPIOB->BSRR = GPIO_BSRR_BS_12;

	// Enable SPI2 clock
	RCC -> APB1ENR1 |= RCC_APB1ENR1_SPI2EN;

	// Configure SPI

	// - mode 0 (CPOL = 0, CPHA = 0)
	// - Full-Duplex, no CRC
	// - 8-bit data frame
	// - MSB first
	SPI2->CR1 = 0x0000;

	// - Master Mode
	// - Software Slave Management
	SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

	// - 250kHz (BR = /256)
	SPI2->CR1 |= (0x07U <<SPI_CR1_BR_Pos);

	// Set FIFO Reception threshold to 8-bit
	SPI2->CR2 |= SPI_CR2_FRXTH;

	// Set data size to 8-bit
	SPI2->CR2 &= ~SPI_CR2_DS;
	SPI2->CR2 |= (0x7U << SPI_CR2_DS_Pos);

}


/*
 * BSP_SDHC_Init()
 *
 * Performs SD Card initialization to work with SPI
 */

SDRESULT BSP_SDHC_Init()
{
	sdhc_command_t	sdc;

	uint8_t	i;

	uint8_t	trial, success;

	// Generate more than 74 clock cycles, with both DI and CS high

	GPIOB->BSRR = GPIO_BSRR_BS_12;	// CS high

	// Enable SPI2
	SPI2->CR1 |= SPI_CR1_SPE;

	for (i=0; i<12; i++)
	{
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)0xFF;
	}

	// Disable SPI2 after BSY=0 and TXE=1
	while( (SPI2->SR & (SPI_SR_BSY | SPI_SR_TXE)) != SPI_SR_TXE);
	SPI2->CR1 |= SPI_CR1_SPE;

	delay_ms(1);

	// Send CMD0 (CRC = 0x95)

	my_printf("Sending CMD0... ");
	LPUART1->TDR = '*';

	sdc.command_index 	= CMD0_GO_IDLE_STATE;
	sdc.arg 			= 0x00000000;
	sdc.crc				= 0x95;
	sdc.resp_length		= 1;

	GPIOB->BSRR = GPIO_BSRR_BR_12;					// set CS (low)
	BSP_SDHC_SendCommand(&sdc);
	GPIOB->BSRR = GPIO_BSRR_BS_12;					// set CS (high)

	for (i=0; i<sdc.resp_length; i++) my_printf("[%02x]", sdc.resp[i]);
	my_printf("\r\n");

	// Send CMD8 (CRC = 0x87)

	my_printf("Sending CMD8... ");
	LPUART1->TDR = '#';

	sdc.command_index 	= CMD8_SEND_IF_COND;
	sdc.arg 			= 0x000001AA;
	sdc.crc				= 0x87;
	sdc.resp_length		= 5;

	GPIOB->BSRR = GPIO_BSRR_BR_12;					// set CS (low)
	BSP_SDHC_SendCommand(&sdc);
	GPIOB->BSRR = GPIO_BSRR_BS_12;					// set CS (high)

	for (i=0; i<sdc.resp_length; i++) my_printf("[%02x]", sdc.resp[i]);
	my_printf("\r\n");

	// Initialize SDCARD

	my_printf("Start Init process... \r\n");

	trial = 8;
	success = 0;

	while( (success == 0) && (trial > 0) )
	{
		// Send CMD55 prior to any ACMD command

		my_printf("Sending CMD55... ");
		LPUART1->TDR = '%';

		sdc.command_index 	= CMD55_APP_CMD;
		sdc.arg 			= 0x00000000;
		sdc.crc				= 0x00;
		sdc.resp_length		= 1;

		GPIOB->BSRR = GPIO_BSRR_BR_12;					// set CS (low)
		BSP_SDHC_SendCommand(&sdc);
		GPIOB->BSRR = GPIO_BSRR_BS_12;					// set CS (high)

		for (i=0; i<sdc.resp_length; i++) my_printf("[%02x]", sdc.resp[i]);
		my_printf("\r\n");


		// Send ACMD41

		my_printf("Sending ACMD41... ");

		sdc.command_index 	= ACMD41_SD_SEND_OP_COND;
		sdc.arg 			= 0x40000000;
		sdc.crc				= 0x00;
		sdc.resp_length		= 1;

		GPIOB->BSRR = GPIO_BSRR_BR_12;					// set CS (low)
		BSP_SDHC_SendCommand(&sdc);
		GPIOB->BSRR = GPIO_BSRR_BS_12;					// set CS (high)

		for (i=0; i<sdc.resp_length; i++) my_printf("[%02x]", sdc.resp[i]);
		my_printf("\r\n");

		if (sdc.resp[0] == 0x00) success = 1;

		else
		{
			delay_ms(100);
			trial--;
		}
	}

	// Change the SPI speed now to 0.625MHz (BR = /64)
	my_printf("Speed-up SPI now\r\n");
	SPI2->CR1 &= ~(SPI_CR1_BR);
	SPI2->CR1 |= (0x04<<SPI_CR1_BR_Pos);

	// Send CMD58
	my_printf("Sending CMD58... ");

	sdc.command_index 	= CMD58_READ_OCR;
	sdc.arg 			= 0x00000000;
	sdc.crc				= 0x00;
	sdc.resp_length		= 5;

	GPIOB->BSRR = GPIO_BSRR_BR_12;					// set CS (low)
	BSP_SDHC_SendCommand(&sdc);
	GPIOB->BSRR = GPIO_BSRR_BS_12;					// set CS (high)

	for (i=0; i<sdc.resp_length; i++) my_printf("[%02x]", sdc.resp[i]);

	// Test for CCS (Card Capacity Status) bit[30] in OCR register
	if((sdc.resp[1] & 0x40) != 0x40)
	{
		my_printf("\r\n\nSD Card Initialization failed\r\n");
		return SD_ERROR;
	}

	else
	{
		my_printf("\r\n\nSD Card Initialization completed\r\n\n");
		return SD_OK;
	}
}


/*
 *	BSP_SDHC_SendCommand(sdhc_command_t *sdc)
 *
 *	Sends a single SD command and record response
 */

SDRESULT BSP_SDHC_SendCommand(sdhc_command_t *sdc)
{
	uint8_t				i, r, t;

	// Enable SPI2
	SPI2->CR1 |= SPI_CR1_SPE;

	// Empty RX data and read SR register to clear OVR if necessary
	r = *(__IO uint8_t *)&SPI2->DR;
	*(__IO uint16_t *)&SPI2->SR;

	// Byte #1 is command index
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0x40 | sdc->command_index);

	// Byte #2 is argument[3]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((sdc->arg & 0xFF000000) >>24U);

	// Get Response from Byte #1
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #3 is argument[2]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((sdc->arg & 0x00FF0000) >>16U);

	// Get Response from Byte #2
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #4 is argument[1]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((sdc->arg & 0x0000FF00) >>8U);

	// Get Response from Byte #3
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #5 is argument[0]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((sdc->arg & 0x000000FF) >>0U);

	// Get Response from Byte #4
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #6 is CRC
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)(sdc->crc);

	// Get Response from Byte #5
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Wait for response to come

	r = 0xFF;
	t = 8;

	while ( (r==0xFF) && (t>0) )
	{
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)0xFF;

		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		r = *(__IO uint8_t *)&SPI2->DR;

		t--;
	}

	if (t==0) return SD_TIMEOUT;

	// Record response

	sdc->resp[0] = r;
	i = 1;

	if (sdc->resp_length > 1)
	{
		while (i<(sdc->resp_length -1))
		{
			while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
			*(__IO uint8_t *)&SPI2->DR = (uint8_t)0xFF;

			while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
			sdc->resp[i] = *(__IO uint8_t *)&SPI2->DR;
			i++;
		}
	}

	// Read last data

	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	sdc->resp[i] = *(__IO uint8_t *)&SPI2->DR;

	// Disable SPI2 after BSY=0 and TXE=1
	while( (SPI2->SR & (SPI_SR_BSY | SPI_SR_TXE)) != SPI_SR_TXE);
	SPI2->CR1 |= SPI_CR1_SPE;

	return SD_OK;
}



/*
 * BSP_SDHC_Read (uint8_t *buffer, uint32_t addr, uint32_t nb)
 *
 * Reads a 512-byte sector at addr into buffer
 */


SDRESULT BSP_SDHC_Read (uint8_t *buffer, uint32_t addr, uint32_t nb)
{
	uint8_t			r;
	uint32_t		i;
	uint32_t		t;


	if (nb >1 )
	{
		my_printf("Multiple blocks not yet supported\r\n");
		return SD_ERROR;
	}

	while ( (USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART2->TDR = 'R';

	// For single block transfer
	GPIOB->BSRR = GPIO_BSRR_BR_12;					// set CS (low)

	// Enable SPI2
	SPI2->CR1 |= SPI_CR1_SPE;

	// Empty RX data and read SR register to clear OVR if necessary
	r = *(__IO uint8_t *)&SPI2->DR;
	*(__IO uint16_t *)&SPI2->SR;


	// Byte #1 is command index --> CMD17
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0x40 | CMD17_READ_SINGLE_BLOCK);

	// Byte #2 is addr[3]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((addr & 0xFF000000) >>24U);

	// Get Response from Byte #1
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #3 is addr[2]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((addr & 0x00FF0000) >>16U);

	// Get Response from Byte #2
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #4 is addr[1]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((addr & 0x0000FF00) >>8U);

	// Get Response from Byte #3
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #5 is addr[0]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((addr & 0x000000FF) >>0U);

	// Get Response from Byte #4
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #6 is CRC
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0x00);

	// Get Response from Byte #5
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Wait for response to come

	r = 0xFF;
	t = 512;

	while ( (r!=0xFE) && (t>0) )
	{
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)0xFF;

		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		r = *(__IO uint8_t *)&SPI2->DR;

		t--;
	}

	if (t==0) return SD_TIMEOUT;

	// Record response

	i = 0;

	while (i<(512-1))
	{
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)0xFF;

		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		buffer[i] = *(__IO uint8_t *)&SPI2->DR;
		i++;
	}

	// Read last data

	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	buffer[i] = *(__IO uint8_t *)&SPI2->DR;

	// Disable SPI2 after BSY=0 and TXE=1
	while( (SPI2->SR & (SPI_SR_BSY | SPI_SR_TXE)) != SPI_SR_TXE);
	SPI2->CR1 |= SPI_CR1_SPE;

	GPIOB->BSRR = GPIO_BSRR_BS_12;					// set CS (high)

	while ( (USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART2->TDR = '*';

	return SD_OK;
}


/*
 * BSP_SDHC_Write(uint8_t *buffer, uint32_t addr, uint32_t nb)
 *
 * Writes buffer into a 512-byte sector at addr
 *
 */

SDRESULT BSP_SDHC_Write(uint8_t *buffer, uint32_t addr, uint32_t nb)
{
	uint8_t			r;
	uint32_t		i;
	uint32_t		t;

	if (nb >1 )
	{
		my_printf("Multiple blocks not yet supported\r\n");
		return SD_ERROR;
	}

	while ( (USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART2->TDR = 'W';

	// For single block transfer
	GPIOB->BSRR = GPIO_BSRR_BR_12;					// set CS (low)

	// Enable SPI2
	SPI2->CR1 |= SPI_CR1_SPE;

	// Empty RX data and read SR register to clear OVR if necessary
	r = *(__IO uint8_t *)&SPI2->DR;
	*(__IO uint16_t *)&SPI2->SR;


	// Byte #1 is command index --> CMD24
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0x40 | CMD24_WRITE_SINGLE_BLOCK);

	// Byte #2 is addr[3]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((addr & 0xFF000000) >>24U);

	// Get Response from Byte #1
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #3 is addr[2]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((addr & 0x00FF0000) >>16U);

	// Get Response from Byte #2
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #4 is addr[1]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((addr & 0x0000FF00) >>8U);

	// Get Response from Byte #3
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #5 is addr[0]
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)((addr & 0x000000FF) >>0U);

	// Get Response from Byte #4
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Byte #6 is CRC
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0x00);

	// Get Response from Byte #5
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;


	// Send 4 dummy bytes
	for (i=0; i<4; i++)
	{
		// Dummy byte
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0xFF);

		// Get Response
		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		r = *(__IO uint8_t *)&SPI2->DR;
	}


	// Send data token
	while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
	*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0xFE);

	// Get Response
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;


	// Sends 512-byte data buffer
	for(i=0; i<512; i++)
	{
		// Send data
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)buffer[i];

		// Get Response
		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		r = *(__IO uint8_t *)&SPI2->DR;
	}

	// Sends fake 2-byte CRC
	for(i=0; i<2; i++)
	{
		// Dummy byte
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0x55);

		// Get Response
		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		r = *(__IO uint8_t *)&SPI2->DR;
	}

	// Wait for response to come
	r = 0xFF;
	t = 512;

	while ( (r==0xFF) && (t>0) )
	{
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)0xFF;

		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		r = *(__IO uint8_t *)&SPI2->DR;

		t--;
	}

	if (t==0) return SD_TIMEOUT;


	// Sends additional 4 dummy bytes
	for (i=0; i<4; i++)
	{
		// Dummy byte
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)(0xFF);

		// Get Response
		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		r = *(__IO uint8_t *)&SPI2->DR;
	}


	// Wait until busy time, sending clocks
	r = 0x00;
	t = 0x00FFFFFF;

	while ( (r==0x00) && (t>0) )
	{
		while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
		*(__IO uint8_t *)&SPI2->DR = (uint8_t)0xFF;

		while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		r = *(__IO uint8_t *)&SPI2->DR;

		t--;
	}

	if (t==0) return SD_TIMEOUT;

	// Read last data
	while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
	r = *(__IO uint8_t *)&SPI2->DR;

	// Disable SPI2 after BSY=0 and TXE=1
	while( (SPI2->SR & (SPI_SR_BSY | SPI_SR_TXE)) != SPI_SR_TXE);
	SPI2->CR1 |= SPI_CR1_SPE;

	GPIOB->BSRR = GPIO_BSRR_BS_12;					// set CS (high)

	while ( (USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART2->TDR = '*';

	return SD_OK;
}




