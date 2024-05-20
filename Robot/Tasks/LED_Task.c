#include "LED_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "INS_Task.h"
#include "can.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"



void LED_Task(void const * argument)
{
	TIM3->CCR1=1000;
	
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
		vTaskDelay(5);
//		CAN2_Transmit();
//		CDC_Transmit_FS("ylj\r\n",sizeof("ylj\r\n"));   	
	}
}
