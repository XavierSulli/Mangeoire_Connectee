/**
  ******************************************************************************
  * @file  : hci_tl_interface.c
  * @brief : This file provides the implementation for all functions prototypes 
  *          for the STM32 BlueNRG-MS HCI Transport Layer interface
  ******************************************************************************
  *
  * COPYRIGHT 2019 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//#include "RTE_Components.h"

#include "stm32l4xx.h"
#include "delay.h"
#include "bsp.h"

#include "hci_tl.h"



#define HEADER_SIZE       5U
#define MAX_BUFFER_SIZE   255U
#define TIMEOUT_DURATION  15U

// EXTI_HandleTypeDef hexti0;

/******************** IO Operation and BUS services ***************************/

/**
 * @brief  Initializes the peripherals communication with the BlueNRG
 *         Expansion Board (via SPI, I2C, USART, ...)
 *
 * @param  void* Pointer to configuration struct 
 * @retval int32_t Status
 */



int32_t HCI_TL_SPI_Init(void* pConf)
{
	// Start GPIOF clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// PB2 -> Bluetooth INT (SPI IRQ) pin

	// PB2 as input
	GPIOB->MODER &= ~(GPIO_MODER_MODER2);

	// Pull-Down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	GPIOB->PUPDR |=  (0x02 <<GPIO_PUPDR_PUPD2_Pos);


	// Enable SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Configure PF12 as EXTI12
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR1_EXTI2);
	SYSCFG->EXTICR[3] |=  (SYSCFG_EXTICR1_EXTI2_PF);

	// Unmask EXTI12
	EXTI->IMR1 |= EXTI_IMR1_IM2;

	// Enable rising trigger
	EXTI->RTSR1 |=   EXTI_RTSR1_RT2;
	EXTI->FTSR1 &= ~(EXTI_FTSR1_FT2);


	// PB0 ->	CS    pin
	// PB1  -> 	RESET pin


	// Start GPIOD clock
	//RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	// Configure PB0, PB1 as push-pull outputs

	GPIOB->MODER  &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);		// output
	GPIOB->MODER  |=  (0x01 <<0U) | (0x01 <<2U);

	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);			// Push-pull
	GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1);	    // No Pull resistor
	GPIOB->OSPEEDR |= (0x03 <<0U) | (0x03 <<2U);					// High-speed

	// Set initial state
	GPIOB->BSRR = GPIO_BSRR_BS0;	// CS    is high
	GPIOB->BSRR = GPIO_BSRR_BS1;	// RESET is high

	// SPI1
	// SCLK -> PB3 (AF5)
	// MISO -> PB4 (AF5)
	// MOSI -> PB5 (AF5)

	// Start GPIOA clock
	//RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Configure PB3, PB4, PB5 as AF mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
	GPIOB->MODER |=  (0x02 <<6U) | (0x02 <<8U) | (0x02 <<10U);

	// Set to Very High-Speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5);
	GPIOB->OSPEEDR |=  (0x03 <<6U) | (0x03 <<8U) | (0x03 <<10U);

	// Connect to SPI1 (AF5)
	GPIOB->AFR[0] &= ~(0x00FFF000);
	GPIOB->AFR[0] |=  (0x00555000);

	GPIOB->AFR[1] &= ~(0x00000000);
	GPIOB->AFR[1] |=  (0x00000000);


	// Enable SPI1 Clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// Configure SPI
	// Default config -> Full-duplex, no CRC, MSB first,...
	SPI1->CR1 = 0x0000;

	// Set the baudrate to /16 -> 5MHz
	SPI1->CR1 |= 0x03 <<3;

	// Set software management of NSS
	SPI1->CR1 |= SPI_CR1_SSM;

	// Set as master (SSI must be high)
	SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI;

	// 8-bit data transfer
	SPI1->CR2 = 0x0000;

	// Set RXNE after 8-bit have been received
	SPI1->CR2 |= SPI_CR2_FRXTH;

	// Enable SPI1
	SPI1->CR1 |= SPI_CR1_SPE;
    
	return 0;
}

/**
 * @brief  DeInitializes the peripherals communication with the BlueNRG
 *         Expansion Board (via SPI, I2C, USART, ...)
 *
 * @param  None
 * @retval int32_t 0
 */
int32_t HCI_TL_SPI_DeInit(void)
{
//  HAL_GPIO_DeInit(HCI_TL_SPI_EXTI_PORT, HCI_TL_SPI_EXTI_PIN);
//  HAL_GPIO_DeInit(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN);
//  HAL_GPIO_DeInit(HCI_TL_RST_PORT, HCI_TL_RST_PIN);
  return 0;
}

/**
 * @brief Reset BlueNRG module.
 *
 * @param  None
 * @retval int32_t 0
 */
int32_t HCI_TL_SPI_Reset(void)
{
	GPIOB->BSRR = GPIO_BSRR_BR1;	// RESET (PB1) is low
	delay_ms(5);

	GPIOB->BSRR = GPIO_BSRR_BS1;	// RESET (PB1) is high
	delay_ms(5);


//  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_RESET);
//  HAL_Delay(5);
//  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_SET);
//  HAL_Delay(5);
  return 0;
}  






int32_t BSP_SPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length)
{
	uint16_t	n;

	n = Length;

	while(n>0)
	{
		// Wait here until TXE is set (transmit buffer empty)
		while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);

		// Send a single byte
		*(__IO uint8_t *)&SPI1->DR = *pTxData;

		// Wait here until RXNE is set (data is received)
 		while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE);

		// Read incoming data
		*pRxData = *(__IO uint8_t *)&SPI1->DR;

		pTxData++;
		pRxData++;
		n--;
	}

	return 0;
}







/**
 * @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
 *
 * @param  buffer : Buffer where data from SPI are stored
 * @param  size   : Buffer size
 * @retval int32_t: Number of read bytes
 */
int32_t HCI_TL_SPI_Receive(uint8_t* buffer, uint16_t size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  uint8_t char_ff = 0xff;
  volatile uint8_t read_char;

  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  /* CS reset */
  GPIOB->BSRR = GPIO_BSRR_BR0;	// CS    is low
  // HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_RESET);

  /* Read the header */
  BSP_SPI1_SendRecv(header_master, header_slave, HEADER_SIZE);

  if(header_slave[0] == 0x02)
  {
    /* device is ready */
    byte_count = (header_slave[4] << 8)| header_slave[3];

    if(byte_count > 0) {

      /* avoid to read more data that size of the buffer */

      if (byte_count > size){
        byte_count = size;
      }

      for(len = 0; len < byte_count; len++)
      {
        BSP_SPI1_SendRecv(&char_ff, (uint8_t*)&read_char, 1);
        buffer[len] = read_char;
      }
    }
  }

  /* Release CS line */
  GPIOB->BSRR = GPIO_BSRR_BS0;	// CS    is high
  // HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_SET);

  return len;

	return 0;
}

/**
 * @brief  Writes data from local buffer to SPI.
 *
 * @param  buffer : data buffer to be written
 * @param  size   : size of first data buffer to be written
 * @retval int32_t: Number of read bytes
 */
int32_t HCI_TL_SPI_Send(uint8_t* buffer, uint16_t size)
{  
  int32_t result;

  uint8_t header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  static uint8_t read_char_buf[MAX_BUFFER_SIZE];
  uint32_t tickstart = BSP_GetTick();

  do
  {
    result = 0;

    /* CS reset */
    GPIOB->BSRR = GPIO_BSRR_BR0;	// CS    is low
    // HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_RESET);

    /* Read header */
    BSP_SPI1_SendRecv(header_master, header_slave, HEADER_SIZE);

    if(header_slave[0] == 0x02)
    {
      /* SPI is ready */
      if(header_slave[1] >= size)
      {
        BSP_SPI1_SendRecv(buffer, read_char_buf, size);
      }
      else
      {
        /* Buffer is too small */
        result = -2;
      }
    } else {
      /* SPI is not ready */
      result = -1;
    }

    /* Release CS line */
    GPIOB->BSRR = GPIO_BSRR_BS0;	// CS    is high
    // HAL_GPIO_WritePin(HCI_TL_SPI_CS_PORT, HCI_TL_SPI_CS_PIN, GPIO_PIN_SET);

    if((BSP_GetTick() - tickstart) > TIMEOUT_DURATION)
    {
      result = -3;
      break;
    }
  } while (result < 0);

  return result;
}

/**
 * @brief  Reports if the BlueNRG has data for the host micro.
 *
 * @param  None
 * @retval int32_t: 1 if data are present, 0 otherwise
 */
static int32_t IsDataAvailable(void)
{
	if ((GPIOF->IDR & GPIO_IDR_ID12) != RESET)
	{
			return 1;
	}

	else return 0;
} 

/***************************** hci_tl_interface main functions *****************************/
/**
 * @brief  Register hci_tl_interface IO bus services
 *
 * @param  None
 * @retval None
 */ 
void hci_tl_lowlevel_init(void)
{
  tHciIO fops;

  /* Register IO bus services */
  fops.Init    = HCI_TL_SPI_Init;
  fops.DeInit  = HCI_TL_SPI_DeInit;
  fops.Send    = HCI_TL_SPI_Send;
  fops.Receive = HCI_TL_SPI_Receive;
  fops.Reset   = HCI_TL_SPI_Reset;
  fops.GetTick = BSP_GetTick;

  hci_register_io_bus (&fops);

  /* Register event irq handler */
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(3, 1, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief HCI Transport Layer Low Level Interrupt Service Routine
  *
  * @param  None
  * @retval None
  */
void hci_tl_lowlevel_isr(void)
{
  /* Call hci_notify_asynch_evt() */
  while(IsDataAvailable())
  {
    hci_notify_asynch_evt(NULL);
  }

  /* USER CODE BEGIN hci_tl_lowlevel_isr */

  /* USER CODE END hci_tl_lowlevel_isr */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
