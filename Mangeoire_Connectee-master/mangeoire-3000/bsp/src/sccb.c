/*
 * sccb.c
 *
 *  Created on: 15 oct. 2018
 *      Author: alexandre.ferroni
 */

#include "sccb.h"

/*
SCCB Init
//I2C pins
//I2C3_SCL PC0
//I2C3_SDA PC1
 */

//now 20Mhz
void sccb_mco(){
	// PA8 -> AF0
	// Use PA8 as MCO output

	RCC->CFGR &= ~(RCC_CFGR_MCOSEL_Msk);
	RCC->CFGR |=  (0x01 << RCC_CFGR_MCOSEL_Pos);

	// Set MCO prescaler to /4 -> 20MHz
	RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOPRE_DIV4;

	// PA8 -> AF0
	// GPIOA clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure PA8 as AF
	GPIOA->MODER &= ~(GPIO_MODER_MODE8_Msk);
	GPIOA->MODER |= (0x02 << GPIO_MODER_MODE8_Pos);

	// Set to AF0 (MCO output)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000000);

	// Update System core clock
	SystemCoreClockUpdate();

}


void sccb_init(){
	/*
	SCCB Init
	//I2C pins
	//I2C3_SCL PC0
	//I2C3_SDA PC1
	 */

	// Enable I2C3 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;

	// GPIOC clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC0, PC1 as AF
	GPIOC->MODER &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk) ;
	GPIOC->MODER |= (0x02 <<GPIO_MODER_MODE0_Pos) | (0x02 <<GPIO_MODER_MODE1_Pos);

	// Setup Open-Drain
	GPIOC->OTYPER |= (GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1);

	// Output speed : Very-High
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk | GPIO_OSPEEDR_OSPEED1_Msk);
	GPIOC->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED0_Pos)| (0x03 << GPIO_OSPEEDR_OSPEED1_Pos);

	// Setup pull-up might be needed
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1);
	//GPIOC->PUPDR |= (0x01 << 00) | (0x01 << 02);

	// Connect to I2C3 (AF4)
	GPIOC->AFR[0] &= ~(0x000000FF);
	GPIOC->AFR[0] |=   0x00000044;

	// I2C2 Configuration
	I2C3->CR1 = 0x00000000;

	// Timing tuning
	I2C3->TIMINGR = I2C3_TIMING;

}


/*
 * Reads a certain register in I2C slave
 * PARAM:
 *   addr = slave register address
 *   *buffer = pointer to data return buffer
 */
uint8_t sccb_ReadNReg(uint8_t addr, uint8_t *buffer)
{
	uint32_t	timeout;
	timeout = 100000;

	// Enable I2C3
	I2C3->CR1 |= I2C_CR1_PE;

/***********************************************************************************/
	// I2C3 Configuration

	// Reset the Control Register 2
	I2C3->CR2 = 0x00000000;
	// Set the slave address
	I2C3->CR2 |= (SCCB_ADDRESS <<1U);
	// Transfer NBYTES is one-byte by default
	I2C3->CR2 |= (0x1 <<16U);
	// Generate an automatic end condition after NBYTES have been transferred
	I2C3->CR2 |= I2C_CR2_AUTOEND;
	// Set I2C in Write mode
	I2C3->CR2 &= ~(I2C_CR2_RD_WRN);

/***********************************************************************************/

	// Start transaction with slave address SCCB_READ_ADDRESS in write mode
	I2C3->CR2 |= I2C_CR2_START;

	// TXIS should be 1 (meaning I2C is waiting for a byte in TXDR)
	while ( ((I2C3->ISR & (I2C_ISR_TXIS)) != I2C_ISR_TXIS) && (timeout > 0) )
	{
		timeout --;
	}
	if (timeout == 0) return 2;

	// Send sensor register address (and automatic STOP)
	I2C3->TXDR = (addr);

/***********************************************************************************/
	while ( ((I2C3->ISR & (I2C_ISR_TXE)) != I2C_ISR_TXE) && (timeout > 0) )
		{
			timeout --;
		}
		if (timeout == 0) return 3;

	// Set I2C in Read mode
	I2C3->CR2 |= I2C_CR2_RD_WRN; // Read Transfer 1

	// Transfer NBYTES = nbytes
	//I2C3->CR2 &= ~I2C_CR2_NBYTES;
	I2C3->CR2 |= (1 << 16U);

/********************************************************************************************/

	// Start transaction with slave address  in read mode, read N bytes, generate STOP)
	I2C3->CR2 |= I2C_CR2_START;

	while ( ((I2C3->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) && (timeout > 0) )
	{
		timeout--;
	}
	if (timeout == 0) return 5;

	*buffer = I2C3->RXDR;


	// This flag indicates that a communication is in progress on the bus
	// It is cleared by hardware when a STOP condition is detected -> automatic end
	while ( ((I2C3->ISR & (I2C_ISR_BUSY) ) == I2C_ISR_BUSY) && (timeout > 0) )
	{
		timeout --;
	}
	if (timeout == 0) return 6;


	// Disable I2C3
	I2C3->CR1 &= ~I2C_CR1_PE;

	return 0;
}


/*
 * Writes a certain register in I2C slave
 * PARAM:
 *   addr = slave register address
 *   data = data to send to the slave
 */
uint8_t sccb_WriteNReg(uint8_t addr, uint8_t data)
{
	uint32_t	timeout;
	timeout = 100000;

	// Enable I2C3
	I2C3->CR1 |= I2C_CR1_PE;

/***********************************************************************************/
	// I2C3 Configuration

	// Reset the Control Register 2
	I2C3->CR2 = 0x00000000;
	// Set the slave address
	I2C3->CR2 |= (SCCB_ADDRESS <<1U);
	// Generate an automatic end condition after NBYTES have been transferred
	I2C3->CR2 |= I2C_CR2_AUTOEND;
	// Set I2C in Write mode
	I2C3->CR2 &= ~(I2C_CR2_RD_WRN);
	// Transfer NBYTES = 2
	I2C3->CR2 |= (2 << 16U);

/***********************************************************************************/

	// Start transaction with slave address in write mode
	I2C3->CR2 |= I2C_CR2_START;

	// TXIS should be 1 (meaning I2C is waiting for a byte in TXDR)
	while ( ((I2C3->ISR & (I2C_ISR_TXIS)) != I2C_ISR_TXIS) && (timeout > 0) )	{
		timeout --;
	}
	if (timeout == 0) return 2;

	// Send sensor register address
	I2C3->TXDR = (addr);

	// I2C is waiting that the byte is transfered
	while ( ((I2C3->ISR & (I2C_ISR_TXE)) != I2C_ISR_TXE) && (timeout > 0) )
		{
			timeout --;
		}
	if (timeout == 0) return 3;

	// Send sensor data
	I2C3->TXDR = (data);

	// I2C is waiting that the byte is transfered
	while ( ((I2C3->ISR & (I2C_ISR_TXE)) != I2C_ISR_TXE) && (timeout > 0) )
		{
			timeout --;
		}
	if (timeout == 0) return 4;

	// This flag indicates that a communication is in progress on the bus
	// It is cleared by hardware when a STOP condition is detected -> automatic end
	while ( ((I2C3->ISR & (I2C_ISR_BUSY) ) == I2C_ISR_BUSY) && (timeout > 0) )
			{
				timeout --;
			}
	if (timeout == 0) return 5;

	// Disable I2C3
	I2C3->CR1 &= ~I2C_CR1_PE;

	return 0;
}

/*
 * void SCL_high(){GPIOC->ODR |= (0x1U << GPIO_ODR_OD0_Pos);}
void SCL_low(){GPIOC->ODR &= ~(GPIO_ODR_OD0_Msk);}
void SDA_high(){GPIOC->ODR |= (0x1U << GPIO_ODR_OD1_Pos);}
void SDA_low(){GPIOC->ODR &= ~(GPIO_ODR_OD1_Msk);}

void sccb_pwr_en(){
	// GPIOC clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC0, PC1 as output mode
	GPIOC->MODER &= ~(GPIO_MODER_MODE3_Msk);
	GPIOC->MODER |= (0x01 <<GPIO_MODER_MODE2_Pos);

	// Setup Open-Drain
	GPIOC->OTYPER |= (GPIO_OTYPER_OT_3);

	// Output speed : Very-High
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3_Msk);
	GPIOC->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED2_Pos);

	// No pull
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	//GPIOC->PUPDR |= (0x02 << 06);

	// Power on the camera
	GPIOC->ODR &= ~(GPIO_ODR_OD3_Msk);
	delay_us(100);
}

void sccb_reset(){
	// GPIOC clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Configure PC0, PC1 as output mode
	GPIOC->MODER &= ~(GPIO_MODER_MODE2_Msk);
	GPIOC->MODER |= (0x01 <<GPIO_MODER_MODE2_Pos);

	// Setup Open-Drain
	GPIOC->OTYPER |= (GPIO_OTYPER_OT_2);

	// Output speed : Very-High
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2_Msk);
	GPIOC->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED2_Pos);

	// Setup pull-up
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	GPIOC->PUPDR |= (0x01 << 04);

	// Reset the camera
	GPIOC->ODR &= ~(GPIO_ODR_OD2_Msk);
	delay_us(100);
	GPIOC->ODR |= (0x1U << GPIO_ODR_OD2_Pos);
	delay_us(100);

}

void sccb_start(){
	sccb_init_output();
	SDA_high();
	SCL_high();
	delay_us(20);
	// SCL and SDA should be HIGH
	SDA_low();
	delay_us(2);
	//SCL_low();
	//delay_us(100);
}

void sccb_stop() {
	sccb_init_output();
	SCL_low();
    delay_us(2);

	SDA_low();
    delay_us(2);

	SCL_high();
    delay_us(2);

	SDA_high();
    delay_us(20);
}

void sccb_nack(){
	sccb_init_output();

	SDA_high();
	delay_us(50);

	SCL_high();
	delay_us(50);

	SCL_low();
	delay_us(50);

	SDA_low();
	delay_us(50);

}

void sccb_write(uint8_t m_data) {

	//front decs
	//actu sda (OUTPUT)
	//wait 1us
	//front montant wait 1us

	//front desc
	// actu sda (input)
	// Fron mont -> strobe SDA



	uint8_t j;
	uint8_t res;
	sccb_init_output();

	for( j = 0; j < 8; j++){

		SCL_low();
		delay_ns();
		if( (m_data<<j) & 0x80)
		{
			SDA_high();
		}
		else
		{
			SDA_low();
		}
		delay_us(2);
		SCL_high();
		delay_us(2);


	}
	SCL_low();
	delay_ns();
	sccb_init_input();
	delay_us(2);
	SCL_high();
	if((GPIOC->IDR & GPIO_IDR_ID1) == GPIO_IDR_ID1)res=1;
	else res=0;
	delay_us(2);

}

uint8_t sccb_read(void) {
	uint8_t read, j;
	read=0x00;

	sccb_init_input();
	for(j=8; j>0; j--) {
		SCL_low();
		delay_us(2);
		SCL_high();
		read <<= 1;
		if((GPIOC->IDR & GPIO_IDR_ID1) == GPIO_IDR_ID1) {
			read |= 1;
		}

		delay_us(2);
	}

	return(read);
}

void sccb_init_gpio(){
	// GPIOC clock enable in run mode
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// Setup Open-Drain
	GPIOC->OTYPER |= (GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1);

	// Output speed : Very-High
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk | GPIO_OSPEEDR_OSPEED1_Msk);
	GPIOC->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED0_Pos)| (0x03 << GPIO_OSPEEDR_OSPEED1_Pos);
	// Setup pull-up
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1);
	//GPIOC->PUPDR |= (0x01 << 00) | (0x01 << 02);

}

void sccb_init_output()
{
	//Pin configuration for I2C_3 pins

		//PC0 = I2C3_SCL
		//PC1 = I2C3_SDA

		// Configure PC0, PC1 as output mode
		GPIOC->MODER &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk) ;
		GPIOC->MODER |= (0x01 <<GPIO_MODER_MODE0_Pos) | (0x01 <<GPIO_MODER_MODE1_Pos);



}

void sccb_init_input()
{
	//Pin configuration for I2C_3 pins

		//PC0 = I2C3_SCL as output
		//PC1 = I2C3_SDA as input
		// Configure PC0, PC1 as input mode
		GPIOC->MODER &= ~(GPIO_MODER_MODE1_Msk) ;
		//GPIOC->MODER |= (0x00 <<GPIO_MODER_MODE1_Pos);

		// Setup no pull
		//GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
		//GPIOC->PUPDR |= (0x01 << 02);
}
 */
