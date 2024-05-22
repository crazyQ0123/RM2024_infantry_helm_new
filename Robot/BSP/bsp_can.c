#include "bsp_can.h"
#include "main.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "detect_task.h"
#include "bsp_cap.h"
#include "string.h"
#include "helm_ctrl.h"

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = 	(int16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#define motor_measure_LK(ptr, data)                                 						\
    {                                                                   				\
        (ptr)->last_ecd = 			(ptr)->ecd;                             				\
        (ptr)->ecd = 						(uint16_t)((data)[7] << 8 | (data)[6]); 				\
        (ptr)->speed_rpm = 			((int16_t)((data)[5] << 8 | (data)[4]))/60.0f; 	\
        (ptr)->given_current = 	(uint16_t)((data)[3] << 8 | (data)[2]); 				\
        (ptr)->temperate = 			(data)[1];                              				\
    }

#define motor_measure_error(ptr, data)                                					\
    {                                                                   				\
        (ptr)->temperate = 			(data)[1];                             					\
        (ptr)->voltage = 			  (uint16_t)((data)[4] << 8 | (data)[3]); 				\
			  (ptr)->error_State = 		(data)[7];                              				\
    }
		
#define get_can_data_16(ptr, data)															\
    {																														\
        (ptr)[0] = (uint16_t)((data)[0] << 8 | (data)[1]);			\
        (ptr)[1] = (uint16_t)((data)[2] << 8 | (data)[3]);			\
        (ptr)[2] = (uint16_t)((data)[4] << 8 | (data)[5]);			\
        (ptr)[3] = (uint16_t)((data)[6] << 8 | (data)[7]);			\
    }	

void get_can_data_fp32(float* ptr, uint8_t* data)		
{																
		memcpy((ptr+0),(data+0),4);	
		memcpy((ptr+1),(data+4),4);		
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
		CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
uint8_t rx_data[8];

motor_measure_t motor_measure_gimbal[3];
motor_error_t motor_error_gimbal[2];
uint32_t id=0;

int32_t dial_angle=0;
CAN_RxHeaderTypeDef rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	if(hcan==&hcan1)
	{
		if(rx_header.StdId!=CAN_3508_M1_ID&&rx_header.StdId!=CAN_3508_M2_ID&&rx_header.StdId!=CAN_3508_M3_ID&&rx_header.StdId!=CAN_3508_M4_ID)
			id=rx_header.StdId;
		
		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_3508_M1_ID;	
				get_motor_measure(&helm[i].M3508, rx_data);
				break;
			}
			
			case CAN_2006_M1_ID:
			{
				get_motor_measure(&motor_measure_gimbal[2], rx_data);
				if(motor_measure_gimbal[2].ecd-motor_measure_gimbal[2].last_ecd>4096)
					dial_angle+=-8192+motor_measure_gimbal[2].ecd-motor_measure_gimbal[2].last_ecd;		
				else if(motor_measure_gimbal[2].ecd-motor_measure_gimbal[2].last_ecd<-4096)
					dial_angle+=8192+motor_measure_gimbal[2].ecd-motor_measure_gimbal[2].last_ecd;
				else
					dial_angle+=motor_measure_gimbal[2].ecd-motor_measure_gimbal[2].last_ecd;
				break;
			}
			
			case CAN_LK_M1_ID:
			case CAN_LK_M2_ID:
			{
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_LK_M1_ID;	
				if(rx_data[0]==0xA1)
				{
					motor_measure_LK(&motor_measure_gimbal[i], rx_data);
				}
				else if(rx_data[0]==0x9B)
				{
					motor_measure_error(&motor_error_gimbal[i],rx_data);
				}
				break;
			}
			
			
			default:
			{
				break;
			}
		}
	}
	
	//CAN2
	if(hcan==&hcan2)
	{
		switch (rx_header.StdId)
		{
			//¶æÂÖ6020
			case CAN_helm_M1_ID:
			case CAN_helm_M2_ID:
			case CAN_helm_M3_ID:
			case CAN_helm_M4_ID:
			{
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_helm_M1_ID;	
				get_motor_measure(&helm[i].M6020, rx_data);
				break;
			}

			case CAN_CAP_RX_ID:
			{
				update_cap(rx_data);
				break;
			}
			
			default:
			{
					break;
			}
		}
	}
	
}
