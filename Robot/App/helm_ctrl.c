#include "helm_ctrl.h"
#include "pid.h"
#include "can.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Vofa_send.h"
#include "Chassis_Task.h"

#define sin_45 0.7071067811865475244f
#define cos_45 0.7071067811865475244f

//舵轮电机编号（直角坐标系四象限）
//FRONT电管
//1|0
//2|3
//BACK灯条
//舵轮电机零位（转子朝两侧）
//FRONT
//-| |-
//-| |-
//BACK

helm_state_t helm[4];
chassis_helm_t chassis_helm;

/**
  * @brief  转角限制到±PI
  * @param  输入转角
  * @retval 输出转角
  */
fp32 limit_pi(fp32 in)
{
	while(in < -PI || in > PI)
	{
		if (in < -PI)
			in = in + PI + PI;
		if (in > PI)
			in = in - PI - PI;
	}
	return in;
}



void PID_init_s(pid_type_def *pid, uint8_t mode, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout)
{
	fp32 pid_para[3];
	pid_para[0]=kp;
	pid_para[1]=ki;
	pid_para[2]=kd;
	PID_init(pid,mode,pid_para,max_out,max_iout);
}

void helm_pid_init()
{
	helm[0].ecd_offset=HELM_OFFSET_0;
	helm[1].ecd_offset=HELM_OFFSET_1;
	helm[2].ecd_offset=HELM_OFFSET_2;
	helm[3].ecd_offset=HELM_OFFSET_3;
	for(uint8_t i=0;i<4;i++)
	{
//		PID_init_s(&helm[i].M6020_angle_pid	,0			,300		,0			,20000			,1000		,0);
//		PID_init_s(&helm[i].M6020_speed_pid	,0			,150		,1			,0			,25000	,10000);
//		PID_init_s(&helm[i].M3508_speed_pid	,1			,10			,1		,0			,16000	,5000);
		PID_init_s(&helm[i].M6020_angle_pid	,0			,150		,0			,100			,1000		,0);
		PID_init_s(&helm[i].M6020_speed_pid	,0			,100		,12			,0			,15000	,7000);
		PID_init_s(&helm[i].M3508_speed_pid	,0			,6			,0.5		,0			,16000	,5000);
	}
		helm[0].angle_set =  45/57.3f;
		helm[1].angle_set = -45/57.3f;
		helm[2].angle_set =  45/57.3f;
		helm[3].angle_set = -45/57.3f;
	
}

void helm_pid_update()
{
	helm[0].speed_set *= -1;
	helm[3].speed_set *= -1;
	
	for(uint8_t i=0;i<4;i++)
	{
		if(KEYB_FLY)
		{
			helm[i].angle_err = limit_pi((helm[i].M6020.ecd-helm[i].ecd_offset)/1303.797294f+helm[i].angle_set);
		}
		else 
		{
			fp32 angle_err1=limit_pi((helm[i].M6020.ecd-helm[i].ecd_offset)/1303.797294f+helm[i].angle_set);
			fp32 angle_err2=limit_pi((helm[i].M6020.ecd-helm[i].ecd_offset+4096)/1303.797294f+helm[i].angle_set);
			if(fabs(angle_err2)<fabs(angle_err1))
			{
				helm[i].angle_err = angle_err2;
				helm[i].speed_set *= -1;
			}
			else helm[i].angle_err=angle_err1;
		}
		
		PID_calc(&helm[i].M6020_angle_pid,helm[i].angle_err,0);
		PID_calc(&helm[i].M6020_speed_pid,helm[i].M6020.speed_rpm,helm[i].M6020_angle_pid.out);
		PID_calc(&helm[i].M3508_speed_pid,helm[i].M3508.speed_rpm,helm[i].speed_set);
//		PID_calc(&helm[i].M3508_speed_pid,helm[i].M3508.speed_rpm,helm[i].speed_set*arm_cos_f32(helm[i].angle_err));
		helm[i].M3508_given_current=helm[i].M3508_speed_pid.out;
		helm[i].M6020_given_current=helm[i].M6020_speed_pid.out;
	}
//	Vofa_Send_Data2(helm[0].M6020.ecd,helm[0].angle_err);
}

void CAN_Chassis_CMD(CAN_HandleTypeDef* hcan,uint32_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  chassis_tx_message = {0};
	uint8_t              chassis_can_send_data[8];
	chassis_tx_message.StdId = id;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;
  
	HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void helm_current_send()
{
	CAN_Chassis_CMD(&hcan1,0x200,helm[0].M3508_given_current,helm[1].M3508_given_current,helm[2].M3508_given_current,helm[3].M3508_given_current);
	vTaskDelay(1);
	CAN_Chassis_CMD(&hcan2,0x1FE,helm[0].M6020_given_current,helm[1].M6020_given_current,helm[2].M6020_given_current,helm[3].M6020_given_current);
	vTaskDelay(1);
}

void helm_current_off()
{
	CAN_Chassis_CMD(&hcan1,0x200,0,0,0,0);
	vTaskDelay(1);
	CAN_Chassis_CMD(&hcan2,0x1FE,0,0,0,0);
	vTaskDelay(1);
}

void helm_solve()
{
	/*
	//反解算底盘
	float helm_vx[4],helm_vy[4];
	for(uint8_t i=0;i<4;i++)
	{
		helm_vx[i]=helm[i].M3508.speed_rpm*cosf(helm[i].angle_err);
		helm_vy[i]=helm[i].M3508.speed_rpm*sinf(helm[i].angle_err);
	}
	helm_vx[0]=-helm_vx[0];
	helm_vy[0]=-helm_vy[0];
	helm_vx[3]=-helm_vx[3];
	helm_vy[3]=-helm_vy[3];
	chassis_helm.real_vx=(helm_vx[0]+helm_vx[1]+helm_vx[2]+helm_vx[3])/4;
	chassis_helm.real_vy=(helm_vy[0]+helm_vy[1]+helm_vy[2]+helm_vy[3])/4;
	chassis_helm.real_wz=sqrt(pow(helm_vx[0]-helm_vx[1]-helm_vx[2]+helm_vx[3],2.0f)+pow(helm_vy[0]+helm_vy[1]-helm_vy[2]-helm_vy[3],2.0f))/4.0f;
	*/
	
	if(chassis_control.vx==0&&chassis_control.vy==0&&chassis_control.wz==0&&(chassis_helm.vx<=300||chassis_helm.vy<=300||chassis_helm.wz<=300))
	{
	}
	else 
	{
		arm_atan2_f32(chassis_helm.vy+chassis_helm.wz*sin_45,chassis_helm.vx+chassis_helm.wz*cos_45,&helm[0].angle_set );
		arm_atan2_f32(chassis_helm.vy+chassis_helm.wz*sin_45,chassis_helm.vx-chassis_helm.wz*cos_45,&helm[1].angle_set );
		arm_atan2_f32(chassis_helm.vy-chassis_helm.wz*sin_45,chassis_helm.vx-chassis_helm.wz*cos_45,&helm[2].angle_set );
		arm_atan2_f32(chassis_helm.vy-chassis_helm.wz*sin_45,chassis_helm.vx+chassis_helm.wz*cos_45,&helm[3].angle_set );
	}
	if(chassis_helm.vx!=0||chassis_helm.vy!=0||chassis_helm.wz!=0)
	{
		helm[0].speed_set = sqrt(pow(chassis_helm.vx+chassis_helm.wz*cos_45,2.0f)+pow(chassis_helm.vy+chassis_helm.wz*sin_45,2.0f));
		helm[1].speed_set = sqrt(pow(chassis_helm.vx-chassis_helm.wz*cos_45,2.0f)+pow(chassis_helm.vy+chassis_helm.wz*sin_45,2.0f));
		helm[2].speed_set = sqrt(pow(chassis_helm.vx-chassis_helm.wz*cos_45,2.0f)+pow(chassis_helm.vy-chassis_helm.wz*sin_45,2.0f));
		helm[3].speed_set = sqrt(pow(chassis_helm.vx+chassis_helm.wz*cos_45,2.0f)+pow(chassis_helm.vy-chassis_helm.wz*sin_45,2.0f));
		
//		//限制3508最大转速
//		fp32 max_speed=0;
//		for(uint8_t i=0;i<4;i++)
//		{
//			if(fabs(helm[i].speed_set)>max_speed)
//			{
//				max_speed=fabs(helm[i].speed_set);
//			}
//		}
//		if(max_speed>8500.0f)
//		{
//			for(uint8_t i=0;i<4;i++)
//			{
//				helm[i].speed_set*=8500/max_speed;
//			}
//		}
	}
	else
	{
		helm[0].speed_set = 0;
		helm[1].speed_set = 0;
		helm[2].speed_set = 0;
		helm[3].speed_set = 0;
	}
}