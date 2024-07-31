#include "USB_Task.h"


uint8_t Autoaim_Mode = AUTOAIM_MODE_NORMAL;

void usb_task(void const * argument)
{
	cmd_id_init();
	cmd_id_task_create(GIMBAL_AND_CONFIG_SEND_ID,999);
	cmd_id_task_create(NUC_START_RECORD,999);
	
	while(1)
	{
		Nuc_data_unpacked();
		cmd_id_queue_handle();
		osDelay(1);
	}
}

