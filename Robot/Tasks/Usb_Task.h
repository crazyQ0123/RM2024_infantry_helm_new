#ifndef USB_TASK_H
#define USB_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "stdlib.h"

#include "bsp_usb.h"

void usb_task(void const * argument);

//����ģʽ
#define AUTOAIM_MODE_NORMAL						0x00		//����ģʽ����ͨ
#define AUTOAIM_MODE_ANTI_TOP					0x01		//����ģʽ����С����
#define AUTOAIM_MODE_SMALL_ENERGY			0x02		//����ģʽ��С��������
#define AUTOAIM_MODE_BIG_ENERGY				0x03		//����ģʽ������������

extern uint8_t Autoaim_Mode;
extern nuc_receive_data_t	nuc_receive_data;
 
#endif
