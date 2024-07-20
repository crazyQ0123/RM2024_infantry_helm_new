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


uint8_t Speed_Mode=0; //0:slow  1:normal  2:fast 
chassis_control_t chassis_control;
fp32 chassis_power=0,chassis_power_limit=0,chassis_power_buffer=0;
fp32 RotAngle_Acc=0,SlantAngle_Acc=0;

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
	int8_t rot_sign=-1;
	if(rc_ctrl.rc.s[1]==RC_SW_UP)
		rot_sign=1;
	if(Game_Robot_Status.chassis_power_limit==0||Game_Robot_Status.chassis_power_limit==65535)
	{
		chassis_control.power_limit_set=FLY_POWER_LIMIT;
		chassis_control.vx=DEADBAND(rc_ctrl.rc.ch[3],50)*12;
		chassis_control.vy=-DEADBAND(rc_ctrl.rc.ch[2],50)*12;
		chassis_control.wz=rc_ctrl.rc.ch[4]<-10 ? rot_sign*DEADBAND(rc_ctrl.rc.ch[4],50)*10 : 0;
	}
	else if(Game_Robot_Status.chassis_power_limit>=100)
	{
		chassis_control.power_limit_set=Game_Robot_Status.chassis_power_limit*1.0f;
		chassis_control.vx=DEADBAND(rc_ctrl.rc.ch[3],50)*10;
		chassis_control.vy=-DEADBAND(rc_ctrl.rc.ch[2],50)*10;
		chassis_control.wz=rc_ctrl.rc.ch[4]<-10 ? rot_sign*DEADBAND(rc_ctrl.rc.ch[4],50)*10 : 0;
	}
	else if(Game_Robot_Status.chassis_power_limit>=80)
	{
		chassis_control.power_limit_set=Game_Robot_Status.chassis_power_limit*1.0f;
		chassis_control.vx=DEADBAND(rc_ctrl.rc.ch[3],50)*8;
		chassis_control.vy=-DEADBAND(rc_ctrl.rc.ch[2],50)*8;
		chassis_control.wz=rc_ctrl.rc.ch[4]<-10 ? rot_sign*DEADBAND(rc_ctrl.rc.ch[4],50)*8 : 0;
	}
	else
	{
		chassis_control.power_limit_set=Game_Robot_Status.chassis_power_limit*1.0f;
		chassis_control.vx=DEADBAND(rc_ctrl.rc.ch[3],50)*6;
		chassis_control.vy=-DEADBAND(rc_ctrl.rc.ch[2],50)*6;
		chassis_control.wz=rc_ctrl.rc.ch[4]<-10 ? rot_sign*DEADBAND(rc_ctrl.rc.ch[4],50)*6 : 0;
	}
}

static void chassis_pc_ctrl(void)
{
	chassis_control.vx=0;
	chassis_control.vy=0;
	chassis_control.wz=0;
	float temp_vx = 0,temp_vy = 0,temp_wz = 0;
	
	if(KEYB_FLY)	//飞坡模式
	{
		chassis_control.power_limit_set=FLY_POWER_LIMIT;
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
	else
	{
		if(Speed_Mode==0)
		{
			chassis_control.power_limit_set=(Game_Robot_Status.chassis_power_limit*0.2f<20.0f) ? 
				Game_Robot_Status.chassis_power_limit*0.8f :Game_Robot_Status.chassis_power_limit-20.0f;
			temp_vx = SLOW_VX_MAX;
			temp_vy = SLOW_VY_MAX;
			temp_wz = SLOW_WZ_MAX;
		}
		else if(Speed_Mode==1)
		{
			chassis_control.power_limit_set=Game_Robot_Status.chassis_power_limit*1.0f;
			temp_vx = NORMAL_VX_MAX;
			temp_vy = NORMAL_VY_MAX;
			temp_wz = NORMAL_WZ_MAX;
		}
		else if(Speed_Mode==2)
		{
			chassis_control.power_limit_set=Game_Robot_Status.chassis_power_limit+30.0f;
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
//	if(KEY_C)
//	{
//		chassis_control.vx = ADJUST_VX_MAX;
//	}
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
	fp32 Total_current_limit=0;
	fp32 Wheel_current_limit=0;
	fp32 power_scale=0;
	fp32 Helm_current=0,Wheel_current=0;
	
	chassis_control.power_limit_set=Game_Robot_Status.chassis_power_limit;
	if(Power_Heat_Data.buffer_energy<LIMIT_POWER_BUFF)
	{
		power_scale=Power_Heat_Data.buffer_energy/LIMIT_POWER_BUFF;
		chassis_control.power_limit_set*=power_scale;
	}
	Total_current_limit=20000+PID_calc(&chassis_control.give_current_pid,cap_data.chassis_power,chassis_control.power_limit_set);
	
	for(uint8_t i=0;i<4;i++)
	{
		Helm_current+=0.15f*fabs(helm[i].M6020_given_current);
	}
	if(Helm_current>POWER_HELM_CURRENT_LIMIT)
	{
		power_scale=POWER_HELM_CURRENT_LIMIT/Helm_current;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M6020_given_current*=power_scale;
		}
		Helm_current=POWER_HELM_CURRENT_LIMIT;
	}
	Wheel_current_limit=Total_current_limit-Helm_current;
	Wheel_current_limit=(Wheel_current_limit>=0) ? Wheel_current_limit : 0;
	for(uint8_t i=0;i<4;i++)
	{
		Wheel_current+=fabs(helm[i].M3508_given_current);
	}
	if(Wheel_current>Wheel_current_limit)
	{
		fp32 error_sum=0;
		for(uint8_t i=0;i<4;i++)
			error_sum+=fabs(helm[i].M3508_speed_pid.error[0]);
		if(error_sum>0)
		{
			for(uint8_t i=0;i<4;i++)
			{
				power_scale=fabs(helm[i].M3508_speed_pid.error[0])/error_sum;
				if(helm[i].M3508_given_current>=0)
				{
					helm[i].M3508_given_current=Wheel_current_limit*power_scale;
				}
				else
				{
					helm[i].M3508_given_current=-Wheel_current_limit*power_scale;
				}
			}
		}
		else
		{
			power_scale=Wheel_current_limit/Wheel_current;
			for(uint8_t i=0;i<4;i++)
			{
				helm[i].M3508_given_current*=power_scale;
			}
		}
	}
}


void chassis_cap_power_control()
{
	fp32 Total_current_limit=0;
	fp32 Wheel_current_limit=0;
	fp32 power_scale=0;
	fp32 Helm_current=0,Wheel_current=0;
	
	if(cap_data.cap_per<0.5f)
	{
		power_scale=cap_data.cap_per/0.5;
		chassis_control.power_limit_set*=power_scale;
	}
	if(Power_Heat_Data.buffer_energy<LIMIT_POWER_BUFF)
	{
		power_scale=pow(Power_Heat_Data.buffer_energy/LIMIT_POWER_BUFF,3.0f);
		chassis_control.power_limit_set=Game_Robot_Status.chassis_power_limit;
		chassis_control.power_limit_set*=power_scale;
	}
	Total_current_limit=20000+PID_calc(&chassis_control.give_current_pid,cap_data.chassis_power,chassis_control.power_limit_set);
	
	for(uint8_t i=0;i<4;i++)
	{
		Helm_current+=0.15f*fabs(helm[i].M6020_given_current);
	}
	if(Helm_current>POWER_HELM_CURRENT_LIMIT)
	{
		power_scale=POWER_HELM_CURRENT_LIMIT/Helm_current;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M6020_given_current*=power_scale;
		}
		Helm_current=POWER_HELM_CURRENT_LIMIT;
	}
	Wheel_current_limit=Total_current_limit-Helm_current;
	Wheel_current_limit=(Wheel_current_limit>=0) ? Wheel_current_limit : 0;
	for(uint8_t i=0;i<4;i++)
	{
		Wheel_current+=fabs(helm[i].M3508_given_current);
	}
	if(Wheel_current>Wheel_current_limit)
	{
		power_scale=Wheel_current_limit/Wheel_current;
		for(uint8_t i=0;i<4;i++)
		{
			helm[i].M3508_given_current*=power_scale;
		}
	}
	
	if(cap_data.cap_per<0.4f)
	{
		PID_calc(&chassis_control.buffer_energy_pid,20.0f-Power_Heat_Data.buffer_energy,0);
		CAN_Cap_CMD(Game_Robot_Status.chassis_power_limit+chassis_control.buffer_energy_pid.out,Power_Heat_Data.chassis_power,Power_Heat_Data.buffer_energy,0);
	}
	else if(cap_data.cap_per<0.7f)
	{
		PID_calc(&chassis_control.buffer_energy_pid,40.0f-Power_Heat_Data.buffer_energy,0);
		CAN_Cap_CMD(Game_Robot_Status.chassis_power_limit+chassis_control.buffer_energy_pid.out,Power_Heat_Data.chassis_power,Power_Heat_Data.buffer_energy,0);
	}
	else 
	{
		PID_clear(&chassis_control.buffer_energy_pid);
		CAN_Cap_CMD(Game_Robot_Status.chassis_power_limit,Power_Heat_Data.chassis_power,Power_Heat_Data.buffer_energy,0);
	}
	
	cap_data.cap_recieve_count++;
	if(cap_data.cap_recieve_count>=250)	//1秒清空超电标志位
	{
		cap_data.cap_recieve_flag=0;
	}
}

float acc_IncludedAngle_calc(float x1,float y1,float x2,float y2)
{
	float L1=sqrt(fabs((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
	float L2=sqrt(fabs(x1*x1+y1*y1));
	float L3=sqrt(fabs(x2*x2+y2*y2));
	if(L1==0||L2==0||L3==0)
		return 1.0f;
	else 
	{
		float cos_Thet=((L2*L2+L3*L3-L1*L1)/2/L2/L3);
		if(cos_Thet>=1||cos_Thet<=-1)
			return 1;
		else 
		{
			float Thet=acosf(cos_Thet);
			float upper_limit=20.0f/180.0f*PI;
			if(Thet<upper_limit && (!isnan(Thet)))
				return 1-Thet/upper_limit;
			else 
				return 0;
		}
	}
}

float acc_SlantAngle_calc(float pitch,float roll)
{
	float Thet;
	float tan_Thet;
	float upper_limit=15.0f/180.0f*PI;
	arm_sqrt_f32(tanf(pitch/180.0f*PI)*tanf(pitch/180.0f*PI)+tanf(roll/180.0f*PI)*tanf(roll/180.0f*PI),&tan_Thet);
	arm_atan2_f32(tan_Thet,1,&Thet);
	chassis_helm.slant_angle=Thet/PI*180;
	if(Thet<upper_limit)
		return 1-Thet/upper_limit;
	else 
		return 0;
}

float ang_offset=4.5;
//车体至底盘解算
void chassis_solve()
{
	if(KEYB_FLY||Game_Robot_Status.chassis_power_limit==0||Game_Robot_Status.chassis_power_limit==65535)//飞坡模式提高加速度
	{
		chassis_control.ramp_vx=RAMP(chassis_control.ramp_vx,chassis_control.vx,100.0f);
		chassis_control.ramp_vy=RAMP(chassis_control.ramp_vy,chassis_control.vy,50.0f);
		chassis_control.ramp_wz=RAMP(chassis_control.ramp_wz,chassis_control.wz,35.0f);
	}
	else
	{
		chassis_control.ramp_vx=RAMP(chassis_control.ramp_vx,chassis_control.vx,15.0f+20.0f*RotAngle_Acc+40.0f*SlantAngle_Acc);
		chassis_control.ramp_vy=RAMP(chassis_control.ramp_vy,chassis_control.vy,15.0f+20.0f*RotAngle_Acc+40.0f*SlantAngle_Acc);
		chassis_control.ramp_wz=RAMP(chassis_control.ramp_wz,chassis_control.wz,35.0f);
	}
	float temp_speed_vx=DEADBAND(chassis_control.ramp_vx,120);
	float temp_speed_vy=DEADBAND(chassis_control.ramp_vy,120);
	float temp_speed_wz=DEADBAND(chassis_control.ramp_wz,80);
	float ang_err = temp_speed_wz*ang_offset*0.00001f+(chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*2*PI;
	chassis_helm.vx=temp_speed_vx*arm_cos_f32(ang_err)+temp_speed_vy*arm_sin_f32(ang_err);
	chassis_helm.vy=-temp_speed_vx*arm_sin_f32(ang_err)+temp_speed_vy*arm_cos_f32(ang_err);
	chassis_helm.wz=temp_speed_wz;
	
	//加速度计算
	chassis_helm.vx_target=chassis_control.vx*arm_cos_f32(ang_err)+chassis_control.vy*arm_sin_f32(ang_err);
	chassis_helm.vy_target=-chassis_control.vx*arm_sin_f32(ang_err)+chassis_control.vy*arm_cos_f32(ang_err);
	RotAngle_Acc=acc_IncludedAngle_calc(chassis_helm.vx,chassis_helm.vy,chassis_helm.vx_target,chassis_helm.vy_target);
	SlantAngle_Acc=acc_SlantAngle_calc(gimbal_LK[1].chassis_pitch,ROLL);
	chassis_control.chassis_follow_gimbal_err = limit_pi((chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*2*PI);

	
	if(	(ABS(chassis_control.vx)>100||ABS(chassis_control.vy)>100) &&
			!chassis_follow_gimbal_changing &&
			!KEY_SHIFT &&
			rc_ctrl.rc.s[1]==RC_SW_UP &&
			fabs((float)rc_ctrl.rc.ch[4])<100	)
	{
		float ang_err_all[4];
		ang_err_all[0] = limit_pi(0.0f*PI+chassis_control.chassis_follow_gimbal_err);
		ang_err_all[1] = limit_pi(0.5f*PI+chassis_control.chassis_follow_gimbal_err);
		ang_err_all[2] = limit_pi(1.0f*PI+chassis_control.chassis_follow_gimbal_err);
		ang_err_all[3] = limit_pi(1.5f*PI+chassis_control.chassis_follow_gimbal_err);
		float ang_err=ang_err_all[0];
		for(uint8_t i=0;i<4;i++)
			if(fabs(ang_err_all[i])<fabs(ang_err))ang_err=ang_err_all[i];
			
		PID_calc(&chassis_control.chassis_psi,ang_err,0);
		chassis_helm.wz=chassis_control.chassis_psi.out;
		if(KEYB_ACROSS_TUNNEL)
			return;
		if(sqrt(temp_speed_vx*temp_speed_vx+temp_speed_vy*temp_speed_vy)/3000.0f<1)
			chassis_helm.wz*=sqrt(temp_speed_vx*temp_speed_vx+temp_speed_vy*temp_speed_vy)/3000.0f;
	}
}

void Chassis_Task(void const * argument)
{
	vTaskDelay(200);	
	chassis_control.chassis_follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
	helm_pid_init();
//	PID_init_s(&chassis_control.chassis_psi,0,8000,0,48000,10000,0);
	PID_init_s(&chassis_control.chassis_psi,0,5000,0,5000,10000,0);
	PID_init_s(&chassis_control.give_current_pid,1,50,3,50,65000,20000);
	PID_init_s(&chassis_control.buffer_energy_pid,1,20,5,5,20,10);
	
	while(1)
	{
		chassis_control.vx_last=chassis_control.vx;
		chassis_control.vy_last=chassis_control.vy;
		chassis_control.wz_last=chassis_control.wz;
		if(rc_ctrl.key.v==0x0000)
		{
			chassis_vector_set();		
		}
		else
		{
			chassis_pc_ctrl();
		}
		if(chassis_helm.slant_angle>8.0f)//检测坡上增加50W底盘功率
		{
			chassis_control.power_limit_set+=50.0f;
		}
		if(chassis_control.vx_last==0&&chassis_control.vy_last==0&&chassis_control.wz_last==0&&(chassis_control.vx!=0||chassis_control.vy!=0||chassis_control.wz!=0))//起步清零功率pid
		{
			PID_clear(&chassis_control.give_current_pid);
		}
		
		
		chassis_solve();
		helm_solve();
		helm_pid_update();

		if(rc_ctrl.rc.s[1]==RC_SW_DOWN)
		{
			helm_current_off();
		}
		else
		{
			//功率限制
			if(cap_data.cap_recieve_flag==1)
				chassis_cap_power_control();
			else
				chassis_power_control();
			
			helm_current_send();
		}
		
		vTaskDelay(2);
		//software reset mcu
		if(KEYB_MCU_RESET)
		{
			#ifdef GIMBAL_MOTOR_SINGLE_CONTROL
			CAN_Send_Gimbal_Reset(0x141);
			CAN_Send_Gimbal_Reset(0x142);
			while(motor_error_gimbal[0].temperate==0 || motor_error_gimbal[1].temperate==0 || motor_error_gimbal[0].error_State!=0 || motor_error_gimbal[1].error_State!=0)
			{
				CAN_Send_Gimbal_Reset(0x141);
				vTaskDelay(1);
				CAN_Send_Gimbal_Reset(0x142);
				vTaskDelay(1);
				if(KEYB_MCU_RESET)
					break;
			}
			#endif
			__disable_irq();
			NVIC_SystemReset();
		}
		
	}
}
