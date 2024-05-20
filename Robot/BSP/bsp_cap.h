#ifndef BSP_CAP_H
#define BSP_CAP_H
#include "main.h"

#define CAN_CAP_TX_ID 0x140
#define CAN_CAP_RX_ID 0x130

typedef struct
{
	float cap_per;
	float chassis_power;
	
	float max_power;
	float actual_power;
	float buffer_power;
}cap_measure_t;

extern cap_measure_t cap_data;
extern uint8_t cap_recieve_flag;
void update_cap(uint8_t * data);
void CAN_Cap_CMD(float data1,float data2,float data3,float data4);

#endif
