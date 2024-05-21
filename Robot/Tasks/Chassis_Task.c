#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "arm_math.h"
#include "referee.h"
#include "INS_Task.h"
#include "helm_ctrl.h"
#include "bsp_cap.h"
#include "referee_usart_task.h"

#define ADJUST_VX_MAX 660*3
#define ADJUST_VY_MAX 660*3
#define ADJUST_WZ_MAX 660*3
#define SLOW_VX_MAX 660*6
#define SLOW_VY_MAX 660*6
#define SLOW_WZ_MAX 660*6
#define NORMAL_VX_MAX 660*8
#define NORMAL_VY_MAX 660*8
#define NORMAL_WZ_MAX 660*8
#define FAST_VX_MAX 660*10
#define FAST_VY_MAX 660*10
#define FAST_WZ_MAX 660*10
#define FLY_VX_MAX 660*12  //约为3.5m/s
#define FLY_VY_MAX 660
#define CHASSIS_SLOW_ADJ 0.78

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f
//#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       22000.0f
#define WARNING_POWER_BUFF  60.0f

#define POWER_HELM_CURRENT_LIMIT       8000.0f

#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 37964
#define GIMBAL_ECD_RANGE 65536

#define sin_45 0.7071067811865475244f
#define cos_45 0.7071067811865475244f


uint8_t Speed_Mode=0; //0:slow  1:normal  2:fast 
chassis_control_t chassis_control;
float chassis_power=0,chassis_power_limit=0,chassis_power_buffer=0;
kf_data_t helm_speed[3];


void CAN_Send_Gimbal_Reset(uint32_t Id)
{
	CAN_TxHeaderTypeDef  motor_tx_message={0};
	uint8_t              motor_can_send_data[8]={0};
	uint32_t send_mail_box=0;
	motor_tx_message.StdId = Id;
	motor_tx_message.IDE = CAN_ID_STD;
	motor_tx_message.RTR = CAN_RTR_DATA;
	motor_tx_message.DLC = 0x08;
	motor_can_send_data[0] = 0x9B;
	motor_can_send_data[1] = 0x00;
	motor_can_send_data[2] = 0x00;
	motor_can_send_data[3] = 0x00;
	motor_can_send_data[4] = 0x00;
	motor_can_send_data[5] = 0x00;
	motor_can_send_data[6] = 0x00;
	motor_can_send_data[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

static void chassis_vector_set(void)
{
	chassis_control.vx=0;
	chassis_control.vy=0;
	chassis_control.wz=0;	
	if(Game_Robot_Status.chassis_power_limit>=100)
	{
		chassis_control.vx=DEADBAND(rc_ctrl.rc.ch[3],50)*12;
		chassis_control.vy=-DEADBAND(rc_ctrl.rc.ch[2],50)*12;
		chassis_control.wz=rc_ctrl.rc.ch[4]>10 ? DEADBAND(rc_ctrl.rc.ch[4],50)*10 : 0;
	}
	else if(Game_Robot_Status.chassis_power_limit>=80)
	{
		chassis_control.vx=DEADBAND(rc_ctrl.rc.ch[3],50)*10;
		chassis_control.vy=-DEADBAND(rc_ctrl.rc.ch[2],50)*10;
		chassis_control.wz=rc_ctrl.rc.ch[4]>10 ? DEADBAND(rc_ctrl.rc.ch[4],50)*8 : 0;
	}
	else
	{
		chassis_control.vx=DEADBAND(rc_ctrl.rc.ch[3],50)*8;
		chassis_control.vy=-DEADBAND(rc_ctrl.rc.ch[2],50)*8;
		chassis_control.wz=rc_ctrl.rc.ch[4]>10 ? DEADBAND(rc_ctrl.rc.ch[4],50)*6 : 0;
	}
//	chassis_control.vx=DEADBAND(rc_ctrl.rc.ch[3],50)*8;
//	chassis_control.vy=-DEADBAND(rc_ctrl.rc.ch[2],50)*8;
//	chassis_control.wz=rc_ctrl.rc.ch[4]>10 ? DEADBAND(rc_ctrl.rc.ch[4],50)*6 : 0;
}

static void chassis_pc_ctrl(void)
{
	chassis_control.vx=0;
	chassis_control.vy=0;
	chassis_control.wz=0;
	float temp_vx = 0,temp_vy = 0,temp_wz = 0;
	
	if(KEYB_FLY)	//飞坡模式
	{
		chassis_control.vx=FLY_VX_MAX;
		if(KEY_A)
		{
			chassis_control.vy = FLY_VY_MAX;
		}
		if(KEY_D)
		{
			chassis_control.vy = -FLY_VY_MAX;
		}
		return;
	}
	
	if(KEY_C)
	{
		temp_vx = ADJUST_VX_MAX;
		temp_vy = ADJUST_VY_MAX;
		temp_wz = ADJUST_WZ_MAX;
	}
	else
	{
		if(Speed_Mode==0)
		{
			temp_vx = SLOW_VX_MAX;
			temp_vy = SLOW_VY_MAX;
			temp_wz = SLOW_WZ_MAX;
		}
		else if(Speed_Mode==1)
		{
			temp_vx = NORMAL_VX_MAX;
			temp_vy = NORMAL_VY_MAX;
			temp_wz = NORMAL_WZ_MAX;
		}
		else if(Speed_Mode==2)
		{
			temp_vx = FAST_VX_MAX;
			temp_vy = FAST_VY_MAX;
			temp_wz = FAST_WZ_MAX;
		}
		
	}
	if(KEY_W)
	{
		chassis_control.vx = temp_vx;
	}
	if(KEY_S)
	{
		chassis_control.vx = -temp_vx;
	}
	if(KEY_A)
	{
		chassis_control.vy = temp_vy;
	}
	if(KEY_D)
	{
		chassis_control.vy = -temp_vy;
	}
	if(KEY_SHIFT)
	{
		chassis_control.wz = temp_wz;
	}
}


/**
  * @brief  数据范围限制
  * @param  输入数据,最小值,最大值
  * @retval 输出数据
  */
fp32 limit(float data, float min, float max)
{
	if (data >= max)
		return max;
	if (data <= min)
		return min;
	return data;
}

void chassis_power_control()
{
	fp32 total_current_limit=0;
	fp32 Wheel_current_limit=0;
	fp32 power_scale=0;
	fp32 Helm_current=0,Wheel_current=0;
	
	if(Game_Robot_Status.chassis_power_limit==0)
		total_current_limit=NO_JUDGE_TOTAL_CURRENT_LIMIT;
	else
		total_current_limit=POWER_TOTAL_CURRENT_LIMIT*Game_Robot_Status.chassis_power_limit/100;
	
	Wheel_current_limit=total_current_limit;
	
	for(uint8_t i=0;i<4;i++)
	{
		Helm_current+=0.15f*fabs(helm[i].M6020_speed_pid.out);
	}
	if(Helm_current>POWER_HELM_CURRENT_LIMIT)
	{
		power_scale=POWER_HELM_CURRENT_LIMIT/Helm_current;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M6020_speed_pid.out*=power_scale;
		}
		Helm_current=POWER_HELM_CURRENT_LIMIT;
	}
	Wheel_current_limit-=Helm_current;
	
	for(uint8_t i=0;i<4;i++)
	{
		Wheel_current+=fabs(helm[i].M3508_speed_pid.out);
	}
	if(Wheel_current>Wheel_current_limit)
	{
		power_scale=Wheel_current_limit/Wheel_current;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M3508_speed_pid.out*=power_scale;
		}
	}
	
	
	if(Game_Robot_Status.chassis_power_limit!=0)
	{
		power_scale=Power_Heat_Data.buffer_energy/WARNING_POWER_BUFF;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M3508_speed_pid.out*=power_scale;
		}
	}
}


void chassis_cap_power_control()
{
	fp32 total_current_limit=0;
	fp32 Wheel_current_limit=0;
	fp32 power_scale=0;
	fp32 Helm_current=0,Wheel_current=0;
	
	if(Game_Robot_Status.chassis_power_limit==0)
		total_current_limit=NO_JUDGE_TOTAL_CURRENT_LIMIT;
	else
		total_current_limit=POWER_TOTAL_CURRENT_LIMIT*Game_Robot_Status.chassis_power_limit/100;
	
	Wheel_current_limit=total_current_limit;
	
	for(uint8_t i=0;i<4;i++)
	{
		Helm_current+=0.15f*fabs(helm[i].M6020_speed_pid.out);
	}
	if(Helm_current>POWER_HELM_CURRENT_LIMIT)
	{
		power_scale=POWER_HELM_CURRENT_LIMIT/Helm_current;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M6020_speed_pid.out*=power_scale;
		}
		Helm_current=POWER_HELM_CURRENT_LIMIT;
	}
	Wheel_current_limit-=Helm_current;
	
	for(uint8_t i=0;i<4;i++)
	{
		Wheel_current+=fabs(helm[i].M3508_speed_pid.out);
	}
	if(Wheel_current>Wheel_current_limit)
	{
		power_scale=Wheel_current_limit/Wheel_current;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M3508_speed_pid.out*=power_scale;
		}
	}
	
	
	if(cap_data.cap_per<0.3f)
	{
		power_scale=cap_data.cap_per/0.3f;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M3508_speed_pid.out*=pow(power_scale,4);
		}
	}
}

float ang_offset=6.0f;
//车体至底盘解算
void chassis_solve()
{
//	chassis_helm.vx=chassis_control.vx*arm_cos_f32(ang_err)+chassis_control.vy*arm_sin_f32(ang_err);
//	chassis_helm.vy=-chassis_control.vx*arm_sin_f32(ang_err)+chassis_control.vy*arm_cos_f32(ang_err);
//	chassis_helm.wz=chassis_control.wz;
	
//	chassis_helm.vx=KalmanFilterCalc(&helm_speed[0], chassis_control.vx*arm_cos_f32(ang_err)+chassis_control.vy*arm_sin_f32(ang_err));
//	chassis_helm.vy=KalmanFilterCalc(&helm_speed[1], -chassis_control.vx*arm_sin_f32(ang_err)+chassis_control.vy*arm_cos_f32(ang_err));
//	chassis_helm.wz=KalmanFilterCalc(&helm_speed[2], chassis_control.wz);
//	chassis_helm.vx=DEADBAND(chassis_helm.vx,50);
//	chassis_helm.vy=DEADBAND(chassis_helm.vy,50);
//	chassis_helm.wz=DEADBAND(chassis_helm.wz,50);
	
	KalmanFilterCalc(&helm_speed[0], chassis_control.vx);
	KalmanFilterCalc(&helm_speed[1], chassis_control.vy);
	KalmanFilterCalc(&helm_speed[2], chassis_control.wz);
	float ang_err = helm_speed[2].out*ang_offset*0.00001f+(chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*2*PI;
	chassis_helm.vx=helm_speed[0].out*arm_cos_f32(ang_err)+helm_speed[1].out*arm_sin_f32(ang_err);
	chassis_helm.vy=-helm_speed[0].out*arm_sin_f32(ang_err)+helm_speed[1].out*arm_cos_f32(ang_err);
	chassis_helm.wz=helm_speed[2].out;
	chassis_helm.vx=DEADBAND(chassis_helm.vx,20);
	chassis_helm.vy=DEADBAND(chassis_helm.vy,20);
	chassis_helm.wz=DEADBAND(chassis_helm.wz,20);
	
	if(chassis_follow_gimbal_changing==0&&!KEY_SHIFT&&rc_ctrl.rc.s[1]==RC_SW_UP&&fabs((float)rc_ctrl.rc.ch[4])<100)
	{
		float ang_err_all[4];
		ang_err_all[0] = limit_pi(0.0f*PI+(chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*2*PI);
		ang_err_all[1] = limit_pi(0.5f*PI+(chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*2*PI);
		ang_err_all[2] = limit_pi(1.0f*PI+(chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*2*PI);
		ang_err_all[3] = limit_pi(1.5f*PI+(chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*2*PI);
		float ang_err=ang_err_all[0];
		for(uint8_t i=0;i<4;i++)
			if(fabs(ang_err_all[i])<fabs(ang_err))ang_err=ang_err_all[i];
			
		PID_calc(&chassis_control.chassis_psi,ang_err,0);
		chassis_helm.wz=chassis_control.chassis_psi.out;
	}
}

void Chassis_Task(void const * argument)
{
	//vTaskDelete(Chassis_TASKHandle);
	vTaskDelay(200);	
	chassis_control.chassis_follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
	for(uint8_t i=0;i<3;i++)
		Kalman_Init(&helm_speed[i],0.1,100);
	helm_pid_init();
//	PID_init_s(&chassis_control.chassis_psi,0,8000,0,48000,10000,0);
	PID_init_s(&chassis_control.chassis_psi,0,5000,0,5000,10000,0);

	while(1)
	{
		if(rc_ctrl.key.v==0x0000)
		{
			chassis_vector_set();		
		}
		else
		{
			chassis_pc_ctrl();
		}
//		if(rc_ctrl.rc.s[1]==RC_SW_UP)
//		{
//			chassis_control.wz = ROTATE_WZ_MAX;
//		}
		
		chassis_solve();
		helm_solve();
		helm_pid_update();
		if(rc_ctrl.rc.s[1]==RC_SW_DOWN)
			helm_current_off();
		else
		{
			if(KEYB_FLY)	//飞坡
			{
				helm_current_send();
			}
			else
			{
				//功率限制
				if(cap_data.cap_per!=0)
					chassis_cap_power_control();
				else
					chassis_power_control();
				helm_current_send();
			}
		}
		CAN_Cap_CMD(Game_Robot_Status.chassis_power_limit,Power_Heat_Data.chassis_power,Power_Heat_Data.buffer_energy,0);
		vTaskDelay(2);
		//software reset mcu
		if(KEYB_MCU_RESET)
		{
			#ifdef GIMBAL_MOTOR_SINGLE_CONTROL
			CAN_Send_Gimbal_Reset(0x141);
			CAN_Send_Gimbal_Reset(0x142);
			#endif
			__disable_irq();
			NVIC_SystemReset();
		}
		
	}
}
