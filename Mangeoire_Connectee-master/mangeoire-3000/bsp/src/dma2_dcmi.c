/*
 * dma2d.c
 *
 *  Created on: 30 oct. 2018
 *      Author: alexandre.ferroni
 */

#include <dma2_dcmi.h>

//extern uint32_t	  dcmi_dma_buffer[buffer_size];

//section 11.6.6
//erreur page 358 MA[31:0]: peripheral address

void dma2_dcmi_init(){
	//p341

	//DMA2 clock enable in run mode
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// Reset DMA2 Channel 5 configuration
	DMA2_Channel5->CCR = 0x00000000;

	//memory-to-memory mode disable

	//priority level : very-high
	DMA2_Channel5->CCR |= (0x3 << DMA_CCR_PL_Pos);

	//memory size // 32 bits
	DMA2_Channel5->CCR |= (0x2 << DMA_CCR_MSIZE_Pos);

	//peripheral size // 32 bits
	DMA2_Channel5->CCR |= (0x2 << DMA_CCR_PSIZE_Pos);

	//memory increment mode enabled
	DMA2_Channel5->CCR |= (0x1 << DMA_CCR_MINC_Pos);

	//peripheral increment mode disabled
	DMA2_Channel5->CCR &= ~DMA_CCR_PINC_Msk;

	// circular mode disabled
	DMA2_Channel5->CCR &= ~DMA_CCR_CIRC_Msk;
	//DMA2_Channel5->CCR |= (0x1 << DMA_CCR_CIRC_Pos); //test

	// number of data to transfer
	DMA2_Channel5->CNDTR &= ~DMA_CNDTR_NDT_Msk;
	DMA2_Channel5->CNDTR |= (10000 << DMA_CNDTR_NDT_Pos); //9600*2 = 160*120

	// Peripheral is DCMI DR
	DMA2_Channel5->CPAR = (uint32_t)&DCMI->DR;

	// Memory is dcmi_dma_buffer
	//DMA2_Channel5->CMAR = (uint32_t)dcmi_dma_buffer;

	//Channel 5 DCMI
	DMA2_CSELR->CSELR |= (0x4 << DMA_CSELR_C5S_Pos);

	// Enable DMA2 Channel 5
	//DMA2_Channel5->CCR |= DMA_CCR_EN;

}
