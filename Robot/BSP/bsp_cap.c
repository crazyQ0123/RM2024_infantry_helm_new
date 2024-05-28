#include "bsp_cap.h"
#include "can.h"

cap_measure_t cap_data;

CAN_TxHeaderTypeDef  cap_tx_message;
uint8_t              cap_can_send_data[8];

void update_cap(uint8_t * data)
{
	cap_data.cap_per=(float)((data)[1]<<8|(data)[0])/32768.0f;
	cap_data.chassis_power=(float)((data)[3]<<8|(data)[2])/100.0f;
	cap_data.cap_recieve_flag=1;
	cap_data.cap_recieve_count=0;
}

void CAN_Cap_CMD(float data1,float data2,float data3,float data4)
{
	uint32_t send_mail_box;
	cap_tx_message.StdId = CAN_CAP_TX_ID;
	cap_tx_message.IDE = CAN_ID_STD;
	cap_tx_message.RTR = CAN_RTR_DATA;
	cap_tx_message.DLC = 0x08;
	
	uint16_t temp;
	
	temp=data1*100;
	cap_can_send_data[0] = temp;
	cap_can_send_data[1] = temp>> 8;
	temp=data2*100;
	cap_can_send_data[2] = temp;
	cap_can_send_data[3] = temp>> 8;
	temp=data3*100;
	cap_can_send_data[4] = temp;
	cap_can_send_data[5] = temp>> 8;
	temp=data4*100;
	cap_can_send_data[6] = temp;
	cap_can_send_data[7] = temp>> 8;

	HAL_CAN_AddTxMessage(&hcan2, &cap_tx_message, cap_can_send_data, &send_mail_box);
}