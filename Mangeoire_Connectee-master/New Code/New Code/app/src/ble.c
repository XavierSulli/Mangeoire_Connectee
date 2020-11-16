/*
 * ble.c
 *
 *  Created on: 6 avr. 2019
 *      Author: Laurent
 */

#include "stm32l4xx.h"
#include "main.h"
#include "delay.h"

#include "ble.h"
#include "bluenrg_types.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_conf.h"

#include "hci_tl_interface.h" //me
#include "hci.h"
#include "hci_le.h"
#include "util.h"

#include "bluenrg_gatt_aci.h"

#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"

#include "sm.h"

#include "sensor_service.h"


extern volatile uint8_t set_connectable;

void BlueNRG_MS_Init(void)
{

  const char *name = "BlueNRG";
  //uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03};
  //uint8_t SERVER_BDADDR[] = {0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
  //uint8_t SERVER_BDADDR[] = {0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0x02};
  uint8_t SERVER_BDADDR[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

  uint8_t  hwVersion;
  uint16_t fwVersion;
  int ret;

  /* Initialize the peripherals and the BLE Stack */
  hci_init(user_notify, NULL);
  //hci_init();
  //DELAY_TIM_ms(100);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);
  my_printf("HW version = 0x%02x\n\rFW version = 0x%04x\r\n", hwVersion, fwVersion);
  DELAY_TIM_ms(100);

  /*
   * Reset BlueNRG again otherwise we won't be able to change its MAC address
   * aci_hal_write_config_data() must be the first command after reset otherwise it will fail
   */

  hci_reset();
  //DELAY_TIM_ms(100);

  /* The Nucleo board must be configured as SERVER */

  //SERVER_BDADDR[5] = 0x02;											// For HW version > 0x30
  BLUENRG_memcpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  //aci_hal_read_config_data(0x80, uint16_t data_len, uint8_t *data_len_out_p, uint8_t *data)

  if (ret)
  {
    my_printf("Setting BD_ADDR failed\r\n");
  }
  else
  {
	  my_printf("Setting BD_ADDR succeeded\r\n");
  }


  ret = aci_gatt_init();

  if(ret)
  {
	  my_printf("GATT_Init failed\r\n");
  }
  else
  {
	  my_printf("GATT_Init succeeded\r\n");
  }


  //ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    my_printf("GAP_Init failed\r\n");
  }
  else
  {
	  my_printf("GAP_Init succeeded\r\n");
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(name), (uint8_t *)name);

  if (ret)
  {
   my_printf("aci_gatt_update_char_value failed\r\n");
    while(1);
  }
  else
  {
	my_printf("aci_gatt_update_char_value succeeded\r\n");
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED, OOB_AUTH_DATA_ABSENT, NULL, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, BONDING);

  if (ret == BLE_STATUS_SUCCESS)
  {
    my_printf("BLE Stack Initialized\r\n");
  }

  my_printf("SERVER: BLE Stack Initialized\r\n");


  ret = Add_Acc_Service();

  if (ret == BLE_STATUS_SUCCESS)
  {
	  my_printf("Acc service added successfully\r\n");
  }

  else
  {
	  my_printf("Error while adding Acc service\r\n");
  }

  ret = Add_Environmental_Sensor_Service();

  if (ret == BLE_STATUS_SUCCESS)
  {
	  my_printf("Environmental Sensor service added successfully\r\n");
  }

  else
  {
	  my_printf("Error while adding Environmental Sensor service\r\n");
  }



  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

  if (ret == BLE_STATUS_SUCCESS)
  {
	  my_printf("TX power set\r\n");
  }

  else
  {
	  my_printf("Error while setting TX power\r\n");
  }

}


/*
 * BlueNRG-MS background task
 */
void BlueNRG_MS_Process(void)
{
	if (set_connectable)
	{
		setConnectable();
		set_connectable = FALSE;
	}

	hci_user_evt_proc();
}




