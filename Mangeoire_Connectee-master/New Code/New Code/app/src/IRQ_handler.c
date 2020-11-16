/*
 * IRQ_handler.c
 *
 *  Created on: 23 nov. 2018
 *      Author: alexandre.ferroni
 */

#include "main.h"
#include "bsp.h"
#include "hci_tl.h"

extern uint8_t	frame_irq;
extern uint8_t 	snap_irq;
extern uint8_t 	uart_img_irq;
extern uint8_t 	uart_rx_irq;
extern uint32_t nbr_data_uart; //number of data to send
extern uint8_t 	cap_start;
extern uint8_t	process_rtc_sync;
extern uint8_t	command_irq;
extern uint8_t	uart_buffer[UART_BUFFER_SIZE];
extern uint8_t	dma_buffer_id;
//extern xSemaphoreHandle xGPSSem;

#define MAX 500

/*
 * This function handles DCMI interrupts
 */
void DCMI_IRQHandler()
{
	// Test for  pending interrupt
	if ((DCMI->RISR & DCMI_RIS_FRAME_RIS) == DCMI_RIS_FRAME_RIS)
	{
		// clear the capture complete flag
		DCMI->ICR |= (0x1 << DCMI_ICR_FRAME_ISC_Pos);

		// Disable DMA2 Channel 5
		DMA2_Channel5->CCR &= ~DMA_CCR_EN;

		nbr_data_uart = 10000 - DMA2_Channel5->CNDTR;
		nbr_data_uart = nbr_data_uart << 2; //4 X because 32 -> 8 bits

		// change the frame_capt
		frame_irq = 1;
	}
}

///*
// * This function handles SNAP interrupts
// */
//void EXTI15_10_IRQHandler()
//{
//	// Test for line 4 pending interrupt
//	if (EXTI->PR1 & EXTI_PR1_PIF13_Msk)
//	{
//		// Clear pending bit 4 by writing a '1'
//		// This bit is cleared by writing a ‘1’ to the bit
//		EXTI->PR1 |= EXTI_PR1_PIF13;
//
//		// Disable EXTI line 4;
//		//EXTI->IMR1 &= ~(EXTI_IMR1_IM4_Pos);
//
//		// Do what you need
//		NVIC_DisableIRQ(EXTI15_10_IRQn);
//
//		// A Snapshot request has occurred
//		snap_irq = 1;
//	}
//}

/**
  * @brief This function handles EXTI line12 interrupt.
  */
void EXTI15_10_IRQHandler(void)
{
	if ((EXTI->PR1 & EXTI_PR1_PIF12) == EXTI_PR1_PIF12)
	{
//		while ( (USART1->ISR & USART_ISR_TC) != USART_ISR_TC);
//		USART1->TDR = '!';

		// Clear pending bit by writing a '1'
		EXTI->PR1 |= EXTI_PR1_PIF12;

		// Call HCI callback
		hci_tl_lowlevel_isr();
	}
}


/*
 * This function handles DMA2_Channel6 interrupts (images -> UART)
 */
void DMA2_Channel6_IRQHandler()
{
	// Wait until transmission is complete
	if((DMA2->ISR & DMA_ISR_TCIF6) == DMA_ISR_TCIF6)
	{
		// Clear dma2 register
		DMA2->IFCR |= (0x1 << DMA_IFCR_CTCIF6_Pos);

		NVIC_DisableIRQ(DMA2_Channel6_IRQn);

		// Disable DMA2 Channel 6
		DMA2_Channel6->CCR &= ~DMA_CCR_EN;

		// Image transmission is complete
		uart_img_irq = 1;
	}
}

/*
 * This function handles DMA2_Channel6 interrupts (UART -> RTC)
 */
void DMA2_Channel7_IRQHandler()
{
//	// Wait until reception is complete
//	if((DMA2->ISR & DMA_ISR_TCIF7) == DMA_ISR_TCIF7)
//	{
//		// Clear dma2 register
//		DMA2->IFCR |= (0x1 << DMA_IFCR_CTCIF7_Pos);
//
//		NVIC_DisableIRQ(DMA2_Channel7_IRQn);
//
//		// Disable DMA2 Channel 6
//		DMA2_Channel7->CCR &= ~DMA_CCR_EN;
//		my_printf("%c",uart_buffer[2]);
//		// Testing for commands
//		if(uart_buffer[0] == '$')
//		{
//			command_irq = 1;
//		}
//		else
//		{
//			// Image transmission is complete
//			process_rtc_sync = 1;
//		}
//		DMA2_Channel7->CCR |= DMA_CCR_EN;
//	}
	if((DMA2->ISR & DMA_ISR_TCIF7) == DMA_ISR_TCIF7)
	{
				// Clear dma2 register
				DMA2->IFCR |= (0x1 << DMA_IFCR_CTCIF7_Pos);

			//	NVIC_DisableIRQ(DMA2_Channel7_IRQn);

					//process_rtc_sync=1;


	}

}

/*
 * This function handles LPUART1 interrupts (UART RX)
 */
/*
void LPUART1_IRQHandler()
{
	uint8_t sent = 0;
	//if((LPUART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	//{
	//read data and clear RXNE flag
	sent = LPUART1->RDR;

	//NVIC_DisableIRQ(LPUART1_IRQn);
	//if ((sent == 0x31)||(sent == 0x10)||(sent == 0x15)||(sent == 0x20))exit = 0;

	uart_rx_irq = 1;
	//}
}
*/
void TIM2_IRQHandler()
{
	TIM2->SR &= ~TIM_SR_UIF;
	BSP_IncTick();

}

/*
 * This function handles Vsync interrupts
 */
void EXTI9_5_IRQHandler()
{
	// Test for line 7 pending interrupt
	if (EXTI->PR1 & EXTI_PR1_PIF7_Msk)
	{
		// Clear pending bit 7 by writing a '1'
		// This bit is cleared by writing a ‘1’ to the bit
		EXTI->PR1 |= EXTI_PR1_PIF7;

		// Disable EXTI line 7;
		EXTI->IMR1 &= ~(EXTI_IMR1_IM7_Pos);

		// Do what you need
		NVIC_DisableIRQ(EXTI9_5_IRQn);

		// Capture has started
		cap_start = 1;
	}
}

/*
 * This function handles USART2 interrupts
 */

extern uint8_t	console_rx_byte;
extern uint8_t	console_rx_irq;
//extern uint8_t	timeout;
extern uint8_t i;
extern uint8_t buffert;


void LPUART1_IRQHandler()
{
	// Test for RXNE pending interrupt
	if ((LPUART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{
		// RXNE flags automatically clears when reading RDR.
		//timeout=timeout-1;
		// Store incoming byte
		buffert= LPUART1->RDR;
		i++;
	}

}

/**
  * @brief  This functions handles DMA1_Stream3 HT and TC interrupts (GPS stream)
  * @param  None
  * @retval None -> Releases the xGPSSem semaphore (wakes GPS task up)
  */

void DMA1_Channel3_IRQHandler(void)
{
	//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// Test for Channel 3 Half Transfer
	if ((DMA1->ISR & DMA_ISR_HTIF3) == DMA_ISR_HTIF3)
	{
		// Clear the interrupt pending bit
		DMA1->IFCR |= DMA_IFCR_CHTIF3;

		// Flag the first DMA half-buffer
		dma_buffer_id = 0;

		// Release GPS semaphore
	    //xSemaphoreGiveFromISR(xGPSSem, &xHigherPriorityTaskWoken);

	    // Perform a context switch to the waiting task
	    //portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	// Test for Channel 3 Transfer Complete
	if ((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3)
	{
		// Clear the interrupt pending bit
		DMA1->IFCR |= DMA_IFCR_CTCIF3;

		// Flag the second DMA half-buffer
		dma_buffer_id = 1;

		// Release GPS semaphore
	    //xSemaphoreGiveFromISR(xGPSSem, &xHigherPriorityTaskWoken);

	    // Perform a context switch to the waiting task
	    //portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}


/*
 * This function handles RTC interrupts
 */

extern uint8_t	rtc_irq;
/*
void RTC_Alarm_IRQHandler()
{


}
*/
