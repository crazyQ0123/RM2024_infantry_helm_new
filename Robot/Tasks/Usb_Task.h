#ifndef USB_TASK_H
#define USB_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "stdlib.h"

#include "bsp_usb.h"

void usb_task(void const * argument);

//自瞄模式
#define AUTOAIM_MODE_AUTO							0x00		//自瞄模式：自动
#define AUTOAIM_MODE_SMALL_ENERGY			0x01		//自瞄模式：小能量机关
#define AUTOAIM_MODE_BIG_ENERGY				0x02		//自瞄模式：大能量机关


extern uint8_t Autoaim_Mode;
extern nuc_receive_data_t	nuc_receive_data;
 
#endif
