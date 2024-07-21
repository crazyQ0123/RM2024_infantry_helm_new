#include "USB_Task.h"


uint8_t Autoaim_Mode = AUTOAIM_MODE_NORMAL;
//uint8_t record_cmd[6]={0xAA,0x21,0x01,0x00,0x01};

void usb_task(void const * argument)
{
	cmd_id_init();
	cmd_id_task_create(GIMBAL_AND_CONFIG_SEND_ID,999);
//	cmd_id_task_create(0x21,999);
//	record_cmd[5]=CRC_Calculation(record_cmd,5);
	while(1)
	{
//		if(KEYB_RECORD)

		Nuc_data_unpacked();
		cmd_id_queue_handle();
		osDelay(1);
	}
}

