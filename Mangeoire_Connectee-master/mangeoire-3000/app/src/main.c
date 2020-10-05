

/*
 * main.c
 *
 *  Created on: 29 nov. 2018
 *      Author: Mathieu
 *      CoAuthor: El famoso Quentin !!!
 */

#include "stm32l4xx.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "bsp.h"
#include "sdhc.h"
#include "ff.h"
#include "delay.h"
#include "uart.h"
#include "rtc.h"



#define	 NULL	((void *)0)
#define precision 1000000
#define MAX 850
#define buffer_size 10000

#define GPS_RX_HALFBUFFER_SIZE 425

/*
 * Local Static Functions
 */

static uint8_t SystemClock_Config	(void);

int8_t 	my_strcmp(const uint8_t *s1, const uint8_t *s2);
void blestate     (void *pvParameters);
//void defaultstate     (void *pvParameters);
void gpsstate     (void *pvParameters);
void pinitstate       (void *pvParameters);
void initstate        (void *pvParameters);
//void freshstate       (void *pvParameters);
void takepicstate     (void *pvParameters);
//void sendspicstate    (void *pvParameters);
//void presetuprtcstate (void *pvParameters);
void setuprtcstate    (void *pvParameters);
void shutdownstate    (void *pvParameters);

// global variables

			uint32_t		dcmi_dma_buffer1[buffer_size];
			uint32_t		dcmi_dma_buffer2[buffer_size];

			uint32_t		dcmi_dma_buffer_HQ[buffer_HQ_size];

			uint8_t		dma_buffer_id;

			uint8_t			frame_irq;
			uint8_t			command_irq;
			uint8_t 		snap_irq;
			uint8_t 		uart_img_irq;
			uint8_t 		uart_rx_irq;
			uint8_t	 		cap_start;
			uint8_t		g_latitude_dd_int, g_longitude_dd_int;
			uint32_t	g_latitude_dd_dec, g_longitude_dd_dec;


			uint8_t	        console_rx_byte;
			uint8_t	        console_rx_irq = 0;

			uint8_t         buffert;
			uint8_t         buffer[MAX];

volatile	uint8_t			process_rtc_sync;					// LPUART1 RX DMA TC for RTC Sync messages

extern 		uint8_t			uart_buffer[UART_BUFFER_SIZE];		// Buffer to store 8-bit LPUART1 RX incoming data

extern		rtcc_t			now;

uint32_t nbr_data_uart; //number of data to send
uint32_t prev_data_uart;//number of data to send, used if snap requested

FRESULT		fresult;
FATFS		fs;
FATFS		*pfs;
FIL			myfile;
uint32_t	free_clust, total_sect, free_sect;

DIR			home_dir;
FILINFO		fno;
uint8_t		filename[16];		// Example : "REC_000.IMA" (+\0)

uint16_t	file_id;			// Beware! Up to 999 (file id) -> 16-bit required
uint16_t	exist_id;
uint16_t	file_count;
uint8_t		cmp;

uint8_t		txtbuf[32];
uint8_t		logString[83];

// Loop and index counters
uint8_t     led;
uint8_t     debug;
uint32_t	i;
uint32_t 	state = 0;
uint16_t	timeout;

xSemaphoreHandle xSem;
xSemaphoreHandle xSem1;
xSemaphoreHandle xSem2;
xSemaphoreHandle xSem3;
xSemaphoreHandle xSem4;
xSemaphoreHandle xSem5;
xSemaphoreHandle xSem6;
xSemaphoreHandle xSem7;
xSemaphoreHandle xSem8;
xSemaphoreHandle xGPSSem;

/*
 * Project Entry Point
 */

int main()
{


	// Configure System Clock for 80MHz from 4MHz MSI (with LSE calibration)
	while (SystemClock_Config() != 0);

	DELAY_TIM_init();
	DELAY_TIM_ms(100);

	// Make-sure NVIC Priority Grouping is 0 (16 priority levels, no sub-priority)
	NVIC_SetPriorityGrouping((uint32_t) 0);


	BSP_LED_Init();


	// Initialize Debug Console
	//BSP_Console_Init();
	uart_init();


	// Create Semaphore object (this is not a 'give')
	xSem =  xSemaphoreCreateBinary();
	xSem1 = xSemaphoreCreateBinary();
	xSem2 = xSemaphoreCreateBinary();
	xSem3 = xSemaphoreCreateBinary();
	xSem4 = xSemaphoreCreateBinary();
	xSem5 = xSemaphoreCreateBinary();
	xSem6 = xSemaphoreCreateBinary();
	xSem7 = xSemaphoreCreateBinary();
	xSem8 = xSemaphoreCreateBinary();
	xGPSSem =  xSemaphoreCreateBinary();

	// Create Tasks
	xTaskCreate(pinitstate,      "pinitstat", 256, NULL, 1, NULL);
	xTaskCreate(gpsstate,      "gpstat", 512, NULL, 5, NULL);
	xTaskCreate(blestate,      "bletat", 256, NULL, 3, NULL);
	xTaskCreate(initstate,        "initstat", 256, NULL, 4, NULL);
	xTaskCreate(shutdownstate,  "shtdwnstat", 256, NULL, 8, NULL);
	xTaskCreate(takepicstate,    "tkpicstat", 256, NULL, 2, NULL);




	my_printf("\033[2J");
	my_printf("\033[0;0H");
	my_printf("\033[37m");
	my_printf("\r\nConsole Ready!\r\n");
	my_printf("SYSCLK = %d Hz\r\n", SystemCoreClock);



	// Welcome message
	my_printf("\r\n");
	my_printf("---------------------------------------\r\n");
	my_printf("-           Mangeoire 3000            -\r\n");
	my_printf("-  Polytech MEA -  Montpellier 2018   -\r\n");
	my_printf("- Firmware version 1.1.5 (16/07/2020) -\r\n");
	my_printf("---------------------------------------\r\n");
	my_printf("\r\n");




	DELAY_TIM_ms(100);

	// Start the Scheduler
	vTaskStartScheduler();




	// Loop forever
	while(1)
	{

	}
}



void pinitstate(void *pvParameters){
	my_printf(" Was a wake-up from SHUTDOWN mode ?\r\n");
	if((RTC->ISR & RTC_ISR_ALRAF) == RTC_ISR_ALRAF){
		init_snap();		//Init pin for snap
		snap_irq_init();	//Init snap's interrupts

		// Enable DMA1 Channel6 TC interrupt (USART2_RX)
		NVIC_SetPriority(DMA2_Channel7_IRQn, 7);
		NVIC_EnableIRQ(DMA2_Channel7_IRQn);

		// Clear reset flags (for next reset)
		RCC->CSR |= RCC_CSR_RMVF;


		// Wait for stabilization
		vTaskDelay(100);

		my_printf("---------------------------------------------> Entering State #2 \r\n");
		xSemaphoreGive(xSem4);

	}
	else {
	if((RTC->ISR & RTC_ISR_INITS_Msk) == RTC_ISR_INITS){
		init_snap();		//Init pin for snap
	snap_irq_init();	//Init snap's interrupts

	// Enable DMA1 Channel6 TC interrupt (USART2_RX)
	NVIC_SetPriority(DMA2_Channel7_IRQn, 7);
	NVIC_EnableIRQ(DMA2_Channel7_IRQn);

	// Clear reset flags (for next reset)
	RCC->CSR |= RCC_CSR_RMVF;


	// Wait for stabilization
	vTaskDelay(100);

	my_printf("---------------------------------------------> Entering State #2 \r\n");
	xSemaphoreGive(xSem2);
	}
	else{
		// Enable DMA1 Channel6 TC interrupt (USART2_RX)
		NVIC_SetPriority(DMA2_Channel7_IRQn, 7);
		NVIC_EnableIRQ(DMA2_Channel7_IRQn);

		// Clear reset flags (for next reset)
		RCC->CSR |= RCC_CSR_RMVF;


		// Wait for stabilization
		vTaskDelay(100);

		xSemaphoreGive(xSem1);
	}
	}


	vTaskSuspend( NULL );
	while(1);
}


void initstate(void *pvParameters){

	while(1){

		xSemaphoreTake(xSem1, portMAX_DELAY);
		my_printf("initstate\r\n");
				timeout = 100;
				while(timeout)
				{
					timeout--;
					if(command_irq == 1)
					{
						if(uart_buffer[1] == 'S')
						{
							debug = 1;
							state = -1;
							command_irq = 0;
							my_printf("---------------------------------------------> Entering Default State\r\n");
							NVIC_SetPriority(DMA2_Channel7_IRQn, 7);
							NVIC_EnableIRQ(DMA2_Channel7_IRQn);
						}
					}
					vTaskDelay(10);
				}
				my_printf("Freshstate \r\n");
						// Perform LED and Systick test
						my_printf("Performing LED test...");

						for (i=0; i<40; i++)
						{
							BSP_LED_Toggle(BLUE);
							delay_ms(50);
							BSP_LED_Toggle(WHITE);
							delay_ms(50);
							BSP_LED_Toggle(RED);
							delay_ms(50);
						}

						my_printf("done !\r\n");
						BSP_PC4_Init();



						vTaskDelay(10);
						// Initialize SPI
		  				BSP_SDHC_SPI_Init();

						// Perform SD card test
						my_printf("Performing SD card test...\r\n");

						my_printf("\n\r");
						timeout = 3;

						do
						{
							// Try SD card reading
							fresult = f_mount(&fs, "0:", 1);
							my_printf("%d",fresult);
							my_printf("\n\r");

							if (fresult == FR_OK)
							{
								pfs = &fs;
								fresult = f_getfree("0:", &free_clust, &pfs);
								total_sect = (fs.n_fatent - 2) * fs.csize;
								free_sect  = free_clust * fs.csize;
								my_printf("- Total drive space = %d kB\r\n", total_sect / 2);
								my_printf("- Avail drive space = %d kB\r\n", free_sect / 2);
								my_printf("- Number of free clusters = %d\r\n", free_clust);

								fresult = f_open((FIL *)&myfile, (TCHAR *)"0:/essai.txt", (BYTE) FA_OPEN_EXISTING | FA_READ);
								fresult = f_read(&myfile, txtbuf, 10, NULL);
								f_close(&myfile);

								my_printf("\r\n\nReading from SD Card : ");
								my_printf("%s\r\n\n", txtbuf);
								for(int c = 0; c < 32; c++){
									my_printf("[%02x]",txtbuf[c]);
								}
								my_printf("\r\n\n");

								my_printf("Writing to SD Card");

								fresult = f_open((FIL *)&myfile, (TCHAR *)"0:/welcome.txt", (BYTE) FA_CREATE_ALWAYS | FA_WRITE);
								fresult = f_write(&myfile, (const void*)"Welcome to the real world", 26, NULL);
								f_close(&myfile);
							}
							else
							{
								my_printf("\033[31m");
								my_printf("...Failed !\r\n");
								BSP_LED_On(RED);
								my_printf("Make sure SD card is in place and reset board\r\n");
								my_printf("\033[37m");
								timeout--;
								// Catch application here until manual reset is applied
							}
						} while(timeout != 0 && fresult != FR_OK);

						if(timeout == 0) while(1);

						my_printf("...done !\r\n");

						my_printf("---------------------------------------------> Entering State #4\r\n");
						//xSemaphoreGive(xSem4);

						my_printf("Enabling DMA for RTC Sync Messages\r\n");

						// Enable DMA1 Channel6 TC interrupt (USART2_RX -> RTC initialization)
						NVIC_SetPriority(DMA2_Channel7_IRQn, 7);
						NVIC_EnableIRQ(DMA2_Channel7_IRQn);

						// Reset interrupt flag
						process_rtc_sync = 0;

						if(!debug)
						{
						my_printf("test_time!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
						vTaskDelay(5000);

						my_printf("---------------------------------------------> Entering State #5\r\n");
						BSP_LED_Off(ALL);
						xSemaphoreGive(xSem5);
						}
			}
}




void takepicstate(void *pvParameters)
{
	while(1){
		xSemaphoreTake(xSem2, portMAX_DELAY);
		my_printf("takepicstate \r\n");
				my_printf("Prise de la photo en cours\r\n");
				// Allows some time for power to stabilize

				delay_us(100);
				BSP_PC3_Init();
				DELAY_TIM_init();	//Init Timer6

				sccb_mco(); 		// 12Mhz master clock out
				sccb_init();	 	// I²C init

				dma2_dcmi_init(); 	// DMA config : DCMI
				dcmi_init();		// DCMI Init

				exti_sync_init(); 	// exti init (vsync)  Cap_start

				OV2640_Reset();		// Cam Software Reset
				DELAY_TIM_us(100);  // Wait for 100us

				OV2640_JPEGConfig();	// JPEG config
				OV2640_LQ(); 			// 160*120 by default

				DELAY_TIM_us(100);

				dcmi_it_en();		// Interrupt Init

				//Enable : 1 -> DCMI enabled
				DCMI->CR |= (0x1 << DCMI_CR_ENABLE_Pos);

				////////////////////////////////First Frame
				// Disable DMA2 Channel 5
				DMA2_Channel5->CCR &= ~DMA_CCR_EN;
				// Memory is dcmi_dma_buffer
				DMA2_Channel5->CMAR = (uint32_t)dcmi_dma_buffer1;
				// number of data to transfer
				DMA2_Channel5->CNDTR &= ~DMA_CNDTR_NDT_Msk;
				DMA2_Channel5->CNDTR |= (buffer_size << DMA_CNDTR_NDT_Pos);
				// Enable DMA2 Channel 5
				DMA2_Channel5->CCR |= DMA_CCR_EN;
				//Capture enable : 1 : capture enabled
				DCMI->CR |= (0x1 << DCMI_CR_CAPTURE_Pos);

				while(!frame_irq); // FIRST FRAME CAPTURED
				frame_irq = 0;

				NVIC_SetPriority(DMA2_Channel6_IRQn, 2); //transfer complete

				uint8_t status = 0;

				cap_start = 0;

				////////////////////////////////////dcmi
				// Disable DMA2 Channel 5
				DMA2_Channel5->CCR &= ~DMA_CCR_EN;
				if(status == 0)
				{
					// Memory is dcmi_dma_buffer2
					DMA2_Channel5->CMAR = (uint32_t)dcmi_dma_buffer2;
				}
				else
				{
					// Memory is dcmi_dma_buffer1
					DMA2_Channel5->CMAR = (uint32_t)dcmi_dma_buffer1;
				}
				// reset buffer
				DMA2_Channel5->CNDTR &= ~DMA_CNDTR_NDT_Msk;
				DMA2_Channel5->CNDTR |= (buffer_size << DMA_CNDTR_NDT_Pos);
				// Enable DMA2 Channel 5
				DMA2_Channel5->CCR |= DMA_CCR_EN;

				//Capture enable : 1 : capture enabled
				DCMI->CR |= (0x1 << DCMI_CR_CAPTURE_Pos);
				exti_sync_start(); //enable start of capture interrupt
				while(!cap_start); // wait for start of capture
				cap_start = 0;

				while(frame_irq != 1);
				frame_irq = 0;

				if(!status)
				{
					status = 1;
				}
				else
				{
					status = 0;
				}

				OV2640_HQ();			// High Quality
				vTaskDelay(800);  	// Wait for 800ms -> cam automatic gain cell

				// Disable DMA2 Channel 5
				DMA2_Channel5->CCR &= ~DMA_CCR_EN;
				DMA2_Channel5->CMAR = (uint32_t)dcmi_dma_buffer_HQ;
				// reset buffer
				DMA2_Channel5->CNDTR &= ~DMA_CNDTR_NDT_Msk;
				DMA2_Channel5->CNDTR |= (buffer_HQ_size << DMA_CNDTR_NDT_Pos);
				// Enable DMA2 Channel 5
				DMA2_Channel5->CCR |= DMA_CCR_EN;

				//Capture enable : 1 : capture enabled
				DCMI->CR |= (0x1 << DCMI_CR_CAPTURE_Pos);
				while(!frame_irq);
				frame_irq = 0;

				OV2640_LQ();		// Low Quality
				snap_irq_enable();
				DMA2_Channel5->CCR &= ~DMA_CCR_EN;
				my_printf("---------------------------------------------> Entering State #3\r\n");
				my_printf("sendspicstate\r\n");
				BSP_PC4_Init();
				BSP_SDHC_SPI_Init();
									// Test if RTC is not yet initialized
									if ((RTC->ISR & RTC_ISR_INITS_Msk) == 1)
									{
										my_printf("RTC is not initalized\r\n");
										my_printf("Trying fake init...");

										now.year    = (uint8_t) 20;
										now.month   = (uint8_t) 5;
										now.day		= (uint8_t) 1;
										now.hours  	= (uint8_t) 0;
										now.minutes = (uint8_t) 0;
										now.seconds = (uint8_t) 0;

										// Perform RTC update
										BSP_RTC_Set(&now);

										// Wait until shadow registers are copied
										if (BSP_RTC_WaitForSynchro() == 0)
										{
											if ((RTC->ISR & RTC_ISR_INITS_Msk) != 0)
											{
												my_printf(" done !\r\n");
											}
											else my_printf("failed !\r\n");
										}
										else my_printf("failed !\r\n");
									}

									// Mount volume
									fresult = f_mount(&fs, "0:", 1);

									if (fresult == FR_OK)
									{
										// Retrieve next available file index
										file_id = 0;
										file_count = 0;

										// Read first directory item
										f_opendir(&home_dir, "0:/");
										f_readdir (&home_dir, &fno);

										while ( fno.fname[0] != 0 )
										{
											// Make sure this is a "OISEAU_####.JPG" file
											my_sprintf((char *)filename, (char *)fno.fname);
											filename[4] = '#';
											filename[5] = '#';
											filename[6] = '#';
											filename[7] = '#';
											cmp = my_strcmp((uint8_t *)filename, (uint8_t *)"CAP_####.JPG");

											if (cmp == 0)
											{
												file_count++;

												// Retrieve file index
												exist_id = ((fno.fname[4] - '0') * 1000) + ((fno.fname[5] - '0') * 100) + ((fno.fname[6] - '0') * 10) +((fno.fname[7] - '0'));

												// Update current index
												if (exist_id > file_id) file_id = exist_id;
											}

											// Read next directory item
											f_readdir (&home_dir, &fno);
										}
									}
									else
									{
										my_printf("\033[31m");
										my_printf("...Failed !\r\n");
										BSP_LED_On(RED);
										my_printf("Make sure SD card is in place and reset board\r\n");
										my_printf("\033[37m");
										// Catch application here until manual reset is applied
										while(1);
									}

									// Short report
									my_printf("Found %d files with last index = %d\r\n", file_count, file_id);

									// Increment file ID
									file_id++;
									my_sprintf((char *)filename, (char *)"CAP_%04d.JPG", file_id);
									my_printf("Current file    %s... \r\n", (char*)filename);

									my_printf("Writing logs to SD Card\r\n");

									BSP_RTC_Get(&now);
									BSP_ADC_Init();
									BSP_PA1_Init();

									ADC2->CR |= ADC_CR_ADVREGEN;
									while ((ADC2->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY);
									ADC2->ISR |= ADC_ISR_ADRDY;
									ADC2->CR |= ADC_CR_ADSTART;

									while ((ADC2->ISR & ADC_ISR_EOC) != ADC_ISR_EOC);
									float convert, result_in_volts, poids;

									convert = ADC2->DR;
									result_in_volts = 3.3*(convert/(float)4096.0);
									poids = -202*result_in_volts + 357;

									int entierV = result_in_volts;
									int decimalV = (result_in_volts-entierV)*precision;
									int entierP = poids;
									int decimalP = (poids-entierP)*precision;

									my_printf("ADC value = %d,%d | poids = %d,%d \r\n", entierV,decimalV,entierP,decimalP);

									//GPIOB->BSRR = GPIO_BSRR_BR_4;

									// Construction de la ligne de log à ajouter
									my_sprintf((char *)logString,(char *)"[%02d/%02d/%02d- %02d:%02d:%02d] - Capture numero %04d - fichier : %s - poids : %03dg\r\n",now.day,now.month,now.year,now.hours,now.minutes,now.seconds,file_id,filename, poids);

									fresult = f_open((FIL *)&myfile, (TCHAR *)"logs.txt", (BYTE) FA_OPEN_APPEND | FA_WRITE);

									if (fresult == FR_OK)
									{
										my_printf("Logs ready !\r\n", (char*)filename);
									}
									else
									{
										// File creation failed
										my_printf("Logs opening failed\r\n");
										BSP_LED_On(RED);

										// Catch application here until Watchdog resets
										while(1);
									}

									my_printf((char*)logString);
									fresult = f_write(&myfile, (const void*)logString, 84, NULL);
									fresult = f_close(&myfile);

									if (fresult == FR_OK)
									{
										my_printf("Logs closed successfully\r\n");
									}
									else
									{
										my_printf("Failed to close logs\r\n");
									}

									fresult = f_open((FIL *)&myfile, (TCHAR *)filename, (BYTE) FA_CREATE_ALWAYS | FA_WRITE);

									if (fresult == FR_OK)
									{
										my_printf("Image file ready !\r\n", (char*)filename);
									}

									else
									{
										// File creation failed
										my_printf("Image opening failed\r\n");
										BSP_LED_On(RED);

										// Catch application here until Watchdog resets
										while(1);
									}

									uint16_t nb_cluster = 0;
									nb_cluster = 40000/128;
									for(uint32_t c = 0; c < nb_cluster; c++){
										fresult = f_write(&myfile, &dcmi_dma_buffer_HQ[c*128], 512, NULL);
									}
									fresult = f_write(&myfile, &dcmi_dma_buffer_HQ[39936], 128, NULL);
									fresult = f_close(&myfile);

									if (fresult == FR_OK)
									{
										my_printf("Image closed successfully\r\n");
									}
									else
									{
										my_printf("Failed to close Image\r\n");
									}

									fresult = f_stat((TCHAR *)filename, &fno);
									switch (fresult) {

									    case FR_OK:
									    	BSP_RTC_Get(&now);
									    	fno.fdate = (WORD)(((now.year - 1980) * 512U) | now.month * 32U | now.day);
									    	fno.ftime = (WORD)(now.hours * 2048U | now.minutes * 32U | now.seconds / 2U);
									    	f_utime((TCHAR *)filename, &fno);
									        break;

									    case FR_NO_FILE:
									        my_printf("No files found\r\n");
									        break;

									    default:
									        my_printf("An error occured. (%d)\n", fresult);
									}

									my_printf("Done\r\n");
									BSP_PC4_Off();
									BSP_PA1_Off();
									BSP_PC3_Off();
									xSemaphoreGive(xSem4);
									if(debug)
									{
										xSemaphoreGive(xSem);
										my_printf("---------------------------------------------> Entering Default State\r\n");
									}


				}
			}


void gpsstate     (void *pvParameters)
{


	uint16_t		bufIndex;
		uint8_t		nmea_byte;

		uint8_t		nmea_sentence[128];
		uint8_t		sentence_length;

		uint8_t		nmea_field[24][16];

		uint16_t	i,j,k;
		uint8_t		a,b,c;


		uint8_t		nmea_CRC[2];

		uint8_t		local_CRC;
		uint8_t		received_CRC;

		uint8_t		parser_state = 0;
		int16_t	    error_count = 0, prev_error_count = -1;

		uint8_t		hours, minutes, seconds,day, month, year;

		uint8_t		latitude_dd_int, longitude_dd_int;
		uint32_t	latitude_dd_dec, longitude_dd_dec;

		uint8_t		speed_int;			// 0 to 255 knots
		uint8_t		speed_dec;			// 00 to 99 (2 decimals)

		uint16_t	heading_int;		// 0 to 359
		uint8_t		heading_dec;		// 00 to 99 (2 decimals)

		uint8_t		sat_in_view;




		// Enable DMA2 Stream 2 Interrupt
		NVIC_SetPriority(DMA1_Channel3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+4);
		NVIC_EnableIRQ(DMA1_Channel3_IRQn);

		// Take the semaphore once to make sure it is empty
		xSemaphoreTake(xSem5, 0);
		xSemaphoreTake(xGPSSem, 0);

		// Start GPS Stream
		uart_gps_init();
		vTaskDelay(1000);
		BSP_PC2_Init();
		BSP_PA9_Init();
		BSP_PA9_On();


		while(1)
		{
			// wait for NMEA semaphore
			xSemaphoreTake(xGPSSem, portMAX_DELAY);
			xSemaphoreTake(xSem5, portMAX_DELAY);


			my_printf("setuprtcstate\r\n");
				//Get RTC info
				BSP_RTC_Get(&now);
				my_printf("%02d/%02d/%02d - %02d:%02d:%02d\r\n", now.day, now.month, now.year, now.hours, now.minutes, now.seconds);

			// Parse incoming NMEA buffer
			// --------------------------

			bufIndex = 0;

			while(bufIndex<GPS_RX_HALFBUFFER_SIZE)
			{
				nmea_byte = buffer[bufIndex + (GPS_RX_HALFBUFFER_SIZE*dma_buffer_id)];
				//my_printf("%c", nmea_byte);

				switch(parser_state)
				{
					// STATE 0 : Wait for start of NMEA sentence

					case 0 :
					{
						if (nmea_byte == '$')
						{
							// Start byte detected, initialize the parser

							for (i=0; i<128; i++)
							{
								nmea_sentence[i]=0;
							}

							i = 0;				// Index into nmea_sentence
							local_CRC = 0;		// Initialize local CRC

							parser_state = 1; 	// -> Go to next state
						}
						break;
					}

					// STATE 1 : NMEA sentence has started, store following bytes until end of sentence is found

					case 1 :
					{
						if (nmea_byte != '*')
						{
							nmea_sentence[i] = nmea_byte;		// one line sentence
							local_CRC ^= nmea_byte;
							i++;

							if (i>=256)
							{
								// Message too long, there should be a problem...
								parser_state = 0;
								error_count++;
							}
						}

						else
						{
							// End of sentence detected

							sentence_length = i;
							j=0;

							parser_state = 2; 	// -> Go to next state
						}
						break;
					}

					// STATE 2 : End of NMEA sentence, check CRC

					case 2 :
					{
						if ((nmea_byte == '\r') || (nmea_byte == '\n'))
						{
							// Convert received CRC from ASCII to Hex

							if (nmea_CRC[0] >= 65) received_CRC = (nmea_CRC[0]-55) * 16;
							else received_CRC = (nmea_CRC[0]-48) * 16;
							if (nmea_CRC[1] >= 65) received_CRC = received_CRC + (nmea_CRC[1]-55);
							else received_CRC = received_CRC + (nmea_CRC[1]-48);

							// Compare both CRC

							if (local_CRC == received_CRC)
							{
								// CRC OK !
								// Initialize field array

								for (j=0; j<24; j++)
								{
									for (k=0; k<16; k++)
									{
										nmea_field[j][k] = 0;
									}
								}

								parser_state = 3; // Go to next state
							}

							else
							{
								// CRC error
								parser_state = 0;
								error_count++;
							}
						}

						else
						{
							// Collect 2 CRC bytes

							nmea_CRC[j] = nmea_byte;
							j++;

							if (j>2)
							{
								// No EOL found after 2 bytes... there is a problem...
								parser_state = 0;
								error_count++;
							}

						}
						break;
					}

					// STATE 3 : CRC OK -> parse sentence

					case 3 :
					{
						j=0;
						k=0;

						// Populate NMEA fields

						for (i=0; i<sentence_length; i++)
						{

							if (nmea_sentence[i] != ',')
							{
								nmea_field[j][k] = nmea_sentence[i];
								k++;
							}

							else
							{
								k=0;
								j++;
							}
						}


						// Process $GPGSA messages

						if ((nmea_sentence[2]=='G') && (nmea_sentence[3]=='S') && (nmea_sentence[4]=='A')&& ((nmea_sentence[8]=='3')|| (nmea_sentence[8]=='2'))&& (a==0))
						{
							a=1;
							b=0;
							c=0;
						}


						// Process $GPRMC messages

						if ((nmea_sentence[2]=='R') && (nmea_sentence[3]=='M') && (nmea_sentence[4]=='C'))
						{
							b=1;

							//vTracePrintF(user_event_channel, "GPRMC");

							// nmea_sentence[sentence_length] = 0;
							// my_printf(nmea_sentence);
							// my_printf("\r\n");


							// Decode time

							if ((nmea_field[1][0]>=0x30) && (nmea_field[1][0]<=0x39))
							{
								hours =   (nmea_field[1][0]-48) * 10 + (nmea_field[1][1]-48);
								minutes = (nmea_field[1][2]-48) * 10 + (nmea_field[1][3]-48);
								seconds = (nmea_field[1][4]-48) * 10 + (nmea_field[1][5]-48);
							}
							else
							{
								hours =   0;
								minutes = 0;
								seconds = 0;
							}

							if ((nmea_field[1][0]>=0x30) && (nmea_field[1][0]<=0x39))
							{
								day   = (nmea_field[1][0]-48) * 10 + (nmea_field[1][1]-48);
								month = (nmea_field[1][2]-48) * 10 + (nmea_field[1][3]-48);
								year  = (nmea_field[1][4]-48) * 10 + (nmea_field[1][5]-48);
							}
							else
							{
								day =   0;
								month = 0;
								year = 0;
							}





							// Decode position and convert to decimal degrees

							if (nmea_field[3][0]!=0)
							{
								latitude_dd_int = (nmea_field[3][0]-48) * 10 + (nmea_field[3][1]-48);

								latitude_dd_dec = 	(nmea_field[3][2]-48) * 1000000 +
													(nmea_field[3][3]-48) * 100000 +
													(nmea_field[3][5]-48) * 10000 +
													(nmea_field[3][6]-48) * 1000 +
													(nmea_field[3][7]-48) * 100 +
													(nmea_field[3][8]-48) * 10 +
													(nmea_field[3][9]-48) * 1 ;

								latitude_dd_dec = latitude_dd_dec / 60;
							}

							else
							{
								latitude_dd_int = 0;
								latitude_dd_dec = 0;
							}

							if (nmea_field[5][0]!=0)
							{
								longitude_dd_int = (nmea_field[5][0]-48) * 100 + (nmea_field[5][1]-48) * 10 + (nmea_field[5][2]-48);

								longitude_dd_dec = 	(nmea_field[5][3] -48) * 1000000 +
													(nmea_field[5][4] -48) * 100000 +
													(nmea_field[5][6] -48) * 10000 +
													(nmea_field[5][7] -48) * 1000 +
													(nmea_field[5][8] -48) * 100 +
													(nmea_field[5][9] -48) * 10 +
													(nmea_field[5][10]-48) * 1 ;

								longitude_dd_dec = longitude_dd_dec / 60;
							}

							else
							{
								longitude_dd_int = 0;
								longitude_dd_dec = 0;
							}




							// Decode Speed over ground

							if (nmea_field[7][0]!=0)
							{
								i = 0;
								while (nmea_field[7][i] != '.') i++;

								j = i;
								k = 1;
								speed_int = 0;

								while (j>0)
								{
									speed_int += (nmea_field[7][j-1]-48) * k;
									k *= 10;
									j--;
								}

								speed_dec = 	(nmea_field[7][i+1]-48 ) * 10
											   +(nmea_field[7][i+2]-48 ) * 1;
							}
							else
							{
								speed_int = 0;
								speed_dec = 0;
							}




							// Decode Heading

							if (nmea_field[8][0]!=0)
							{
								i = 0;
								while (nmea_field[8][i] != '.') i++;

								j = i;
								k = 1;
								heading_int = 0;

								while (j>0)
								{
									heading_int += (nmea_field[8][j-1]-48) * k;
									k *= 10;
									j--;
								}

								heading_dec = 	(nmea_field[8][i+1]-48 ) * 10
											   +(nmea_field[8][i+2]-48 ) * 1;
							}
							else
							{
								heading_int = 0;
								heading_dec = 0;
							}


							/* my_sprintf((char *)display_heading.message, (char *)"%03d.%02d", heading_int, heading_dec);
							display_heading.pos_x = 3;
							display_heading.pos_y = 0;
							p_display_message = &display_heading;
							xQueueSendToBack(xDisplayQueue, &p_display_message, 0); */

							// Update global variables

							now.hours = 			(uint8_t)  hours;
							now.minutes = 			(uint8_t)  minutes;
							now.seconds = 			(uint8_t)  seconds;
							now.day = 				(uint8_t)  day;
							now.month = 			(uint8_t)  month;
							now.year = 				(uint8_t)  year;

							g_latitude_dd_int =  	(uint8_t)  latitude_dd_int;
							g_latitude_dd_dec =  	(uint32_t) latitude_dd_dec;
							g_longitude_dd_int = 	(uint8_t)  longitude_dd_int;
							g_longitude_dd_dec = 	(uint32_t) longitude_dd_dec;
//
//							g_speed_int = speed_int;
//							g_speed_dec = speed_dec;
//							g_heading_int = heading_int;
//							g_heading_dec = heading_dec;

							// Perform RTC update
							BSP_RTC_Set(&now);

							  my_printf("\r\nHeure\r\n");
							  my_printf("\r\n%02d/%02d/%02d - %02d:%02d:%02d\r\n", now.day, now.month, now.year, now.hours, now.minutes, now.seconds);
							  my_printf("\r\nGPS\r\n");
							  my_printf("%d",g_latitude_dd_int);
							  my_printf("\r\n;\r\n");
							  my_printf("%d",g_latitude_dd_dec);
							  my_printf("\r\nLatitude\r\n");
							  my_printf("%d",g_longitude_dd_int);
							  my_printf("\r\n;\r\n");
							  my_printf("%d",g_longitude_dd_dec);
							  my_printf("\r\nLongitude\r\n");
							  my_printf("\r\n");

						}


						// Process $GPGGA messages

						if ((nmea_sentence[2]=='G') && (nmea_sentence[3]=='G') && (nmea_sentence[4]=='A'))
						{	c=1;
							//vTracePrintF(user_event_channel, "GPGGA");

							sat_in_view =  (nmea_field[7][0] - 48) * 10
									      +(nmea_field[7][1] - 48);
							  my_printf("%d",sat_in_view);
							  my_printf("\r\nSattelite\r\n");


						}

						// End parsing

						parser_state = 0;
						break;
					}

					default :
					{
						parser_state = 0;
					}

				} 						// Switch case

				bufIndex++;

			}							// Parser
			if(a==1&b==1&c==1){
				  my_sprintf((char *)logString,(char *)"[%02d/%02d/%02d- %02d:%02d:%02d] - longitude int:%d - longitude dec:%d - latitude int:%d - latitude dec:%d \r\n",now.day,now.month,now.year,now.hours,now.minutes,now.seconds,g_longitude_dd_int,g_longitude_dd_dec, g_latitude_dd_int,g_latitude_dd_dec);
				  fresult = f_open((FIL *)&myfile, (TCHAR *)"logsbis.txt", (BYTE) FA_OPEN_APPEND | FA_WRITE);

					if (fresult == FR_OK)
					{
						my_printf("Logs ready !\r\n", (char*)filename);
					}
					else
					{
						// File creation failed
						my_printf("Logs opening failed\r\n");
						BSP_LED_On(RED);

						// Catch application here until Watchdog resets
						while(1);
					}

					my_printf((char*)logString);
					fresult = f_write(&myfile, (const void*)logString, 84, NULL);
					fresult = f_close(&myfile);

					if (fresult == FR_OK)
					{
						my_printf("Logs closed successfully\r\n");
					}
					else
					{
						my_printf("Failed to close logs\r\n");
					}

				xSemaphoreGive(xSem4);}
			else
				xSemaphoreGive(xSem5);
		} 								// While(1)



}




void shutdownstate(void *pvParameters)
{
	while(1){
		xSemaphoreTake(xSem6, portMAX_DELAY);
		my_printf("shutdownstate");

		my_printf("Preparing to enter SHUTDOWN mode...");

				// Disable SPI2
				SPI2->CR1 &= ~SPI_CR1_SPE;
//réveil avec timer///////////////////////////////////////////////////////////////////
				//BSP_RTC_EXTI_Init();

				//BSP_RTC_SetAlarmA2(&now);

				//BSP_NVIC_Init();

/////////////////////////////////////////////////////////////////////////////////




				// Enable PWR module clock
				RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
//réveil avec impulsion //////////////////////////////////////////////////////////////

				// Disable internal WKUP line
				PWR->CR3 &= ~PWR_CR3_EIWF;

				// Enable the WKUP2 pin (PC13)
				PWR->CR3 |= PWR_CR3_EWUP2;

				// Select falling edge of WKUP2 pin
				PWR->CR4 &= ~PWR_CR4_WP2;
				PWR->CR4 |= PWR_CR4_WP2;

				// Clear all WUFx flags (including WKUP2)
				PWR->SCR |= 0x0000011F;
///////////////////////////////////////////////////////////////////////////////////////

				// Set Low-Power Shutdown mode
				PWR->CR1 &= ~(PWR_CR1_LPMS_Msk);
				PWR->CR1 |= PWR_CR1_LPMS_SHUTDOWN;

				// Set SLEEPDEEP bit in the Cortex-M System Control Register
				SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

				my_printf("Je vais me coucher\r\n");
				BSP_LED_On(BLUE);
				vTaskDelay(1000);
				my_printf("Bonne nuit\r\n");
				// Disable LPUART
				LPUART1->CR1 &= ~USART_CR1_UE;
				__WFI();
			}
}


void blestate     (void *pvParameters)
{
	xSemaphoreTake(xSem4, portMAX_DELAY);
	uint8_t z;
	my_printf("init_BLE!!!!!!!!!!!!!!!!!!\r\n");


	BSP_PC5_Init();
	BSP_PB1_Init();
	BSP_PB1_On();
	BSP_ADC_Init();

	DELAY_TIM2_init();
	NVIC_SetPriority(TIM2_IRQn, 5);
	NVIC_EnableIRQ(TIM2_IRQn);
	vTaskDelay(500);

	HCI_TL_SPI_Init();

	BlueNRG_MS_Init();


	while(1){

	my_printf("BLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
	for (z=0;z<200;z++){

	BlueNRG_MS_Process();

	vTaskDelay(200);
	}

	BSP_RTC_EXTI_Init();

	BSP_RTC_SetAlarmA2(&now);

	BSP_NVIC_Init();
	xSemaphoreGive(xSem6);
	}
}







/*
 * 	Clock configuration for the Nucleo STM32L496ZG board
 * 	MSI clock						-> 4 MHz (with LSE calibration)
 *	SYSCLK, AHB, APB1, APB2			-> 80MHz
 *
 *  PA8 as MCO with MSI output 		-> 4MHz
 *
 */









int8_t my_strcmp(const uint8_t *s1, const uint8_t *s2)
{
  while (*s1 != '\0' && *s1 == *s2)
    {
      s1++;
      s2++;
    }
  return (*(uint8_t *) s1) - (*(uint8_t *) s2);
}

static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout;
	uint32_t	temp;

	// Start LSE (for MSI PLL hardware calibration)
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;	// Enable writing of Battery/Backup domain
	PWR->CR1 |= PWR_CR1_DBP;
	RCC->BDCR &= ~RCC_BDCR_LSEDRV;			// Set LSE driving to medium effort
	RCC->BDCR |= (0x00 <<3);
	RCC->BDCR |= RCC_BDCR_LSEON;			// Start LSE

	// Wait until LSE is ready
	timeout = 1000;

	do
	{
		status = RCC->BDCR & RCC_BDCR_LSERDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));
	if (timeout == 0) return (1);	// LSE error

	// Enable MSI PLL auto-calibration
	RCC->CR |= RCC_CR_MSIPLLEN;

	// Set MSI range
	RCC->CR |= RCC_CR_MSIRANGE_6;	// For 4MHz
	RCC->CR |= RCC_CR_MSIRGSEL;		// MSI range is provided by CR register

	// Start MSI
	RCC->CR |= RCC_CR_MSION;

	// Wait until MSI is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_MSIRDY;
		timeout--;
	} while ((status == 0) && (timeout >0));

	if (timeout == 0) return (2);	// MSI error


	// Make sure PLL is off before configuration
	RCC->CR &= ~RCC_CR_PLLON;

	// Wait until PLL is off
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status != 0) && (timeout > 0));

	if (timeout == 0) return (3);	// PLL error


	// Configure the main PLL for 80MHz output
	//			    PLL_M=1	     PLL_N=40     PLL_P=7       PLL_Q=4        PLL_R=2
	RCC->PLLCFGR = (0x00 <<4) | (0x28 <<8) | (0x00 <<17) | (0x01 <<21) |  (0x00 <<25);

	// Set MSI as PLL input
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_MSI;

	// Enable PLL
	RCC->CR |= RCC_CR_PLLON;

	// Enable PLL_R output (system clock)
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

	// Set voltage range 1 as MCU will run at 80MHz
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	temp = PWR->CR1;
	temp |=  (0x01<<9);
	temp &= ~(0x02<<9);
	PWR->CR1 = temp;

	// Wait until VOSF bit is cleared (regulator ready)
	timeout = 1000;

	do
	{
		status = PWR->SR2 & PWR_SR2_VOSF;
		timeout--;
	} while ((status != 0) && (timeout > 0));

	if (timeout == 0) return (4);	// PWR error

	// Configure FLASH with prefetch and 4 WS
	FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_4WS;

	// Configure AHB/APB prescalers
	// AHB  Prescaler = /1	-> 80 MHz
	// APB1 Prescaler = /1  -> 80 MHz
	// APB2 Prescaler = /2  -> 40 MHz
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (3);	// PLL error


	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	//  Wait until PLL becomes main switch input
	timeout = 1000;

	do
	{
		status = RCC->CFGR & RCC_CFGR_SWS;
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (5);	// SW error


	// Start GPIOA clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure PA8 as AF mode
	GPIOA->MODER &= ~(GPIO_MODER_MODER8);
	GPIOA->MODER |= (0x02 << 16);

	// Connect to MCO (AF0)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=   0x00000000;

	// Set MCO divider
	RCC->CFGR &= ~RCC_CFGR_MCOPRE;
	RCC->CFGR |= RCC_CFGR_MCOPRE_DIV1;

	// Select Master Clock Output (MCO) source
	RCC->CFGR &= ~RCC_CFGR_MCOSEL;
	RCC->CFGR |= (0x02 <<24);			// 0x01 for SYSCLK; 0x02 for MSI

	// Start RTC clock
	RCC->BDCR |= RCC_BDCR_LSCOSEL;	// Select LSE as Low Speed Clock for RTC
	RCC->BDCR |= (0x01 <<8U);		// Select LSE as RTC clock
	RCC->BDCR |= RCC_BDCR_RTCEN;	// Enable RTC clock

	// Update System core clock
	SystemCoreClockUpdate();
	return (0);
}


