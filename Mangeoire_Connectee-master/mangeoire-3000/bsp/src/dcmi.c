/*
 * dcmi.c
 *
 *  Created on: 12 oct. 2018
 *      Author: alexandre.ferroni
 */

#include "dcmi.h"

/*
 section 20 reference manual
 DCMI : PINOUTS
 Compatible avec une version LQFP64

 DCMI_HSYNC : PA4  V AF10  --
 DCMI_VSYNC : PB7 V AF10   --
 DCMI_PIXCLK : PA6 V  AF4  --
 DCMI_D0 : PC6 V AF10 	   --		//PA9 V AF5
 DCMI_D1 : PC7 V AF10	   --		//PA10 V AF5
 DCMI_D2 : PC8 V AF10	   --
 DCMI_D3 : PC9 V AF4	   --
 DCMI_D4 : PC11 V AF10
 DCMI_D5 : PB6 V AF10	   --
 DCMI_D6 : PB8 V AF10  	   --
 DCMI_D7 : PB9 V AF10 	   --

 */

void dcmi_init()
{

	// GPIOA clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// GPIOB clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	// GPIOC clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	//DCMI clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;


	dcmi_gpio_config();

	//DCMI configuration

	DCMI->CR = 0x00000000;
	//Line select mode : 0 Interface captures all received lines
	//Byte select mode : 00 Interface captures all received data
	//Extended data mode : 00 : interface captures 8-bit data on every pixel clock
	//Frame capture rate control : 00 : all frames are captured
	//Vertical synchronization polarity active high by default on the camera
	DCMI->CR |= (0x0 << DCMI_CR_VSPOL_Pos); //test 0
	//Horizontal synchronization polarity active high by default on the camera
	DCMI->CR |= (0x0 << DCMI_CR_HSPOL_Pos); //test 0
	//Pixel clock polarity falling edge active by default on the camera
	DCMI->CR |= (0x1 << DCMI_CR_PCKPOL_Pos); //test rising edge
	// Embedded synch select : 0 : Hardware synch data capture


	//JPEG format : 1 : used for JPEG data transfers //test ov2640
	DCMI->CR |= (0x1 << DCMI_CR_JPEG_Pos); //test 0
	//Crop feature : 0 : The full image is captured
	//Capture mode : 0-Continuous grab mode ; 1-snapshot mode
	DCMI->CR |= (0x1 << DCMI_CR_CM_Pos);
	//USED in others functions
	//Enable : 1 : DCMI enabled
	//Capture enable : 1 : capture enabled

}

void dcmi_it_en(){

	// Capture complete interrupt enable
	DCMI->IER |= (0x1 << DCMI_IER_FRAME_IE_Pos);

	// Set priority level 1 for DCMI interrupt
	NVIC_SetPriority(DCMI_IRQn, 3);

	// Enable DCMI interrupts
	NVIC_EnableIRQ(DCMI_IRQn);

}

void dcmi_gpio_config(){
	//Configuration according to cubeMX
	//Mode : AF
	/*
	 DCMI_HSYNC : PA4  V AF10  --
	 DCMI_VSYNC : PB7 V AF10   --
	 DCMI_PIXCLK : PA6 V  AF4  --
	 DCMI_D0 : PC6 V AF10 	   --		//PA9 V AF5
	 DCMI_D1 : PC7 V AF10	   --		//PA10 V AF5
	 DCMI_D2 : PC8 V AF10	   --
	 DCMI_D3 : PC9 V AF4	   --
	 DCMI_D4 : PC11 V AF10
	 DCMI_D5 : PB6 V AF10	   --
	 DCMI_D6 : PB8 V AF10  	   --
	 DCMI_D7 : PB9 V AF10 	   --
	*/
	//PUPD : No 00
	//OSPEED : Low	00
	//Reset values are ok

	// Configure PA4 and PA6 as AF
	GPIOA->MODER &= ~(GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE6_Msk);
	GPIOA->MODER |= (0x2 <<GPIO_MODER_MODE4_Pos) | (0x2 <<GPIO_MODER_MODE6_Pos);
	// Configure PB6, PB7, PB8 and PB9 as AF
	GPIOB->MODER &= ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk);
	GPIOB->MODER |= (0x2 <<GPIO_MODER_MODE6_Pos) | (0x2 <<GPIO_MODER_MODE7_Pos) | (0x2 <<GPIO_MODER_MODE8_Pos) | (0x2 <<GPIO_MODER_MODE9_Pos);
	// Configure PC6, PC7, PC8, PC9 and PC11 as AF
	GPIOC->MODER &= ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE11_Msk);
	GPIOC->MODER |= (0x2 <<GPIO_MODER_MODE6_Pos) | (0x2 <<GPIO_MODER_MODE7_Pos) | (0x2 <<GPIO_MODER_MODE8_Pos) | (0x2 <<GPIO_MODER_MODE9_Pos) | (0x2 <<GPIO_MODER_MODE11_Pos);


	// Set to the wanted AF
	//GPIOA
	GPIOA->AFR[0] &= ~(0x0F0F0000);
	GPIOA->AFR[0] |=  (0x040A0000);
	//GPIOB low
	GPIOB->AFR[0] &= ~(0xFF000000);
	GPIOB->AFR[0] |=  (0xAA000000);
	//GPIOB high
	GPIOB->AFR[1] &= ~(0x000000FF);
	GPIOB->AFR[1] |=  (0x000000AA);
	//GPIOC low
	GPIOC->AFR[0] &= ~(0xFF000000);
	GPIOC->AFR[0] |=  (0xAA000000);
	//GPIOC high
	GPIOC->AFR[1] &= ~(0x0000F0FF);
	GPIOC->AFR[1] |=  (0x0000A04A);

}

void exti_sync_init(void)
{
	//DCMI_VSYNC : PB7 V AF10   --

	// Enable SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Select Port B as interrupt source for EXTI line 7
	SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI7_Msk;
	SYSCFG->EXTICR[1] |=  SYSCFG_EXTICR2_EXTI7_PB;

	// Enable Rising / Disable Falling trigger
	EXTI->RTSR1 |= EXTI_RTSR1_RT7;
	EXTI->FTSR1 &= ~EXTI_FTSR1_FT7;

}

void exti_sync_start(void)
{

	// Pending interrupt flag on line 7
	EXTI->PR1 |= EXTI_PR1_PIF7;
	// Interrupt request from Line 7 is not masked
	EXTI->IMR1 |= EXTI_IMR1_IM7;

	// Enable EXTI line 7 interrupt;
	NVIC_EnableIRQ(EXTI9_5_IRQn);

}
