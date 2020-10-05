/*
 * uart.c
 *
 *  Created on: 13 nov. 2018
 *      Author: alexandre.ferroni
 */

//section 40
//PA2 TX : PA3 :RX

#include "uart.h"
#include "bsp.h"

#define MAX 850
// Global variable
uint8_t		uart_buffer[UART_BUFFER_SIZE];	// Buffer to store 8-bit LPUART1 RX incoming data
extern char buffer[MAX];


void uart_init(){
	// GPIOA clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure PB10 and PB11 as Alternate function
	/*GPIOB->MODER &= ~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk);
	GPIOB->MODER |= (0x02 <<GPIO_MODER_MODE10_Pos) | (0x02 <<GPIO_MODER_MODE11_Pos);*/
	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODE2_Pos) | (0x02 <<GPIO_MODER_MODE3_Pos);

	//GPIOB->PUPDR |= (0x01 <<22U);
	GPIOA->PUPDR |= (0x01 <<4U);

	// Set PB10 and PB11 to AF8 (LPUART1)
	/*GPIOB->AFR[1] &= ~(0x0000FF00);
	GPIOB->AFR[1] |=  (0x00008800);*/
	// Set PA2 and PA3 to AF8 (LPUART1)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00008800);

	// Enable LPUART1 clock
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

	// Clear LPUART1 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	LPUART1->CR1 = 0x00000000;
	LPUART1->CR2 = 0x00000000;
	LPUART1->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CCIPR &= ~(RCC_CCIPR_LPUART1SEL_Msk);
	// RCC->CCIPR |= (0x1 << RCC_CCIPR_LPUART1SEL_Pos);

	//Baud rate = 115200
	//Fck = 48*10^6
	//Tx/Rx baud =	256*fck/LPUARTDIV
	//LPUARTDIV is coded on the LPUART_BRR register
	LPUART1->BRR = 0x2B671; //for 80Mhz

	//Baud rate = 921600
	//Fck = 48*10^6
	//Tx/Rx baud =	256*fck/LPUARTDIV
	//LPUARTDIV is coded on the LPUART_BRR register
	//LPUART1->BRR = 0x56CE; //for 80Mhz

	//Baud rate = 9600 //il faut modifier la clock de 48 à 24 pour pouvoir avoir un baud rate de 9600
	//Fck = 48*10^6
	//Tx/Rx baud =	256*fck/LPUARTDIV
	//LPUARTDIV is coded on the LPUART_BRR register
	//LPUART1->BRR = 0x9C400;

	// Enable DM2 Clock
	//RCC->AHB1ENR = RCC_AHB1ENR_DMA2EN;

	// DMA1 channel mapping (Channel 7 on Request 4)
	//DMA2_CSELR->CSELR &= ~0x0F000000;
	//DMA2_CSELR->CSELR |=  0x04000000;

	// Set priority level to high
	//DMA2_Channel7->CCR |= 0x02 <<12U;

	// Set memory data size to 8-bits
	//DMA2_Channel7->CCR |= 0x00 <<10U;

	// Set peripheral data size to 8-bits
	//DMA2_Channel7->CCR |= 0x00 <<8U;

	// Enable memory increment
	//DMA2_Channel7->CCR |= DMA_CCR_MINC;

	// Disable peripheral increment
	//DMA2_Channel7->CCR &= ~DMA_CCR_PINC;

	// Enable circular mode
	//DMA2_Channel7->CCR |= DMA_CCR_CIRC;

	// Peripheral -> Memory
	//DMA2_Channel7->CCR &= ~DMA_CCR_DIR;

	// Number of data to be transfered
	//DMA2_Channel7->CNDTR = (uint32_t) MAX;

	// Peripheral address
	//DMA2_Channel7->CPAR = (uint32_t) &(LPUART1->RDR);

	// Memory buffer address
	//DMA2_Channel7->CMAR = (uint32_t) &uart_buffer[0]; // à changer
	//DMA2_Channel7->CMAR = (uint32_t) &buffer;

	// Enable TC interrupt
	//DMA2_Channel7->CCR |= DMA_CCR_TCIE;

	// Enable DMA2 Channel7
	//DMA2_Channel7->CCR |= DMA_CCR_EN;

	//DMAR: DMA enable receiver
	//LPUART1->CR3 |= (0x1 << USART_CR3_DMAR_Pos);

	//RXNEIE: RXNE interrupt enable
	//LPUART1->CR1 |= (0x1 << USART_CR1_RXNEIE_Pos);

	// Enable both Transmitter and Receiver
	//LPUART1->CR1 |= USART_CR1_TE;
	LPUART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable LPUART1
	LPUART1->CR1 |= USART_CR1_UE;

}

void uart_gps_init(){
	// GPIOB clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Configure PB10 and PB11 as Alternate function
	GPIOB->MODER &= ~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk);
	GPIOB->MODER |= (0x02 <<GPIO_MODER_MODE10_Pos) | (0x02 <<GPIO_MODER_MODE11_Pos);
	// Configure PA2 and PA3 as Alternate function
	//GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
	//GPIOA->MODER |= (0x02 <<GPIO_MODER_MODE2_Pos) | (0x02 <<GPIO_MODER_MODE3_Pos);

	//GPIOB->PUPDR |= (0x01 <<22U);
	//GPIOA->PUPDR |= (0x01 <<4U);

	// Set PB10 and PB11 to AF7 (USART3)
	GPIOB->AFR[1] &= ~(0x0000FF00);
	GPIOB->AFR[1] |=  (0x00007700);
	// Set PA2 and PA3 to AF8 (LPUART1)
	//GPIOA->AFR[0] &= ~(0x0000FF00);
	//GPIOA->AFR[0] |=  (0x00008800);

	// Enable USART3 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

	// Clear USART3 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART3->CR1 = 0x00000000;
	USART3->CR2 = 0x00000000;
	USART3->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CCIPR &= ~(RCC_CCIPR_USART3SEL_Msk);
	// RCC->CCIPR |= (0x1 << RCC_CCIPR_LPUART1SEL_Pos);

	//Baud rate = 115200
	//Fck = 48*10^6
	//Tx/Rx baud =	256*fck/LPUARTDIV
	//LPUARTDIV is coded on the LPUART_BRR register
	USART3->BRR = 0x208D; //for 80Mhz

	//Baud rate = 921600
	//Fck = 48*10^6
	//Tx/Rx baud =	256*fck/LPUARTDIV
	//LPUARTDIV is coded on the LPUART_BRR register
	//LPUART1->BRR = 0x56CE; //for 80Mhz

	//Baud rate = 9600 //il faut modifier la clock de 48 à 24 pour pouvoir avoir un baud rate de 9600
	//Fck = 48*10^6
	//Tx/Rx baud =	256*fck/LPUARTDIV
	//LPUARTDIV is coded on the LPUART_BRR register
	//LPUART1->BRR = 0x9C400;

	// Enable both Transmitter and Receiver
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
	// Enable DMA1 Clock
	RCC->AHB1ENR = RCC_AHB1ENR_DMA1EN;

	// DMA1 channel mapping (Channel 7 on Request 4)
	DMA1_CSELR->CSELR &= ~0x00000F00;
	DMA1_CSELR->CSELR |=  0x00000200;

	// Set priority level to high
	DMA1_Channel3->CCR |= 0x02 <<12U;

	// Set memory data size to 8-bits
	DMA1_Channel3->CCR |= 0x00 <<10U;

	// Set peripheral data size to 8-bits
	DMA1_Channel3->CCR |= 0x00 <<8U;

	// Enable memory increment
	DMA1_Channel3->CCR |= DMA_CCR_MINC;

	// Disable peripheral increment
	DMA1_Channel3->CCR &= ~DMA_CCR_PINC;

	// Enable circular mode
	DMA1_Channel3->CCR |= DMA_CCR_CIRC;

	// Peripheral -> Memory
	DMA1_Channel3->CCR &= ~DMA_CCR_DIR;

	// Number of data to be transfered
	DMA1_Channel3->CNDTR = (uint32_t) MAX;

	// Peripheral address
	DMA1_Channel3->CPAR = (uint32_t) &(USART3->RDR);

	// Memory buffer address
	//DMA2_Channel7->CMAR = (uint32_t) &uart_buffer[0]; // à changer
	DMA1_Channel3->CMAR = (uint32_t) &buffer;

	// Enable TC interrupt
	DMA1_Channel3->CCR |= DMA_CCR_TCIE;

	// Enable DMA2 Channel7
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	//DMAR: DMA enable receiver
	USART3->CR3 |= (0x1 << USART_CR3_DMAR_Pos);

	//RXNEIE: RXNE interrupt enable
	USART3->CR1 |= (0x1 << USART_CR1_RXNEIE_Pos);


	//LPUART1->CR1 |= USART_CR1_TE;


	// Enable LPUART1
	USART3->CR1 |= USART_CR1_UE;

}

//          /*!< LP UART1 interrupt

/*
 *  Bluetooth test : OK 115200
 *  AT+BAUD°    °1->C
 *  8---------115200
 *  B---------921600
 *	C---------1382400
 *  HC-06
 *  https://www.olimex.com/Products/Components/RF/BLUETOOTH-SERIAL-HC-06/resources/hc06.pdf
 *
 * 	uint8_t	sent0 = 0;
	uint8_t	sent1 = 0;
	uint8_t	sent2 = 0;
	uint8_t	sent3 = 0;
	uint8_t	sent4 = 0;
	uint8_t	sent5 = 0;
	uint8_t	sent6 = 0;
	uint8_t	sent7 = 0;
	uint8_t	sent8 = 0;
	uint8_t	sent9 = 0;
	uint8_t	sent10 = 0;
	uint8_t	sent11 = 0;

	/////////
	uart_init();		// Init UART
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'A';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'T';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = '+';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'V';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'E';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'R';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'S';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'I';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'O';
	while((LPUART1->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	LPUART1->TDR = 'N';

	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent0 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent1 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent2 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent3 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent4 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent5 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent6 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent7 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent8 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent9 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent10 = LPUART1->RDR;
	while((LPUART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
	sent11 = LPUART1->RDR;
	*/
