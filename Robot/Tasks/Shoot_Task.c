#include "Shoot_Task.h"
#include "referee_usart_task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include "referee.h"
#include "math.h"
#include "Usb_Task.h"


#define SHOOT_MOTOR_SPEED_PID_KP 12.0f//8.0f
#define SHOOT_MOTOR_SPEED_PID_KI 0.8f//0.3f
#define SHOOT_MOTOR_SPEED_PID_KD 0.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_IOUT 8000.0f

#define SHOOT_MOTOR_ANGLE_PID_KP 0.3f//0.6f
#define SHOOT_MOTOR_ANGLE_PID_KI 0.0f
#define SHOOT_MOTOR_ANGLE_PID_KD 0.3f//0.8f
#define SHOOT_MOTOR_ANGLE_PID_MAX_OUT 5000.0f
#define SHOOT_MOTOR_ANGLE_PID_MAX_IOUT 10000.0f

shoot_motor_t shoot_m2006[1];
uint8_t fric_state=0;//0:off,1:on
uint8_t dial_mode=0,if_single_hit=0;//mode:0 continue,1 single
int16_t dial_speed=0;
extern int32_t dial_angle;
float dial_err=0;
uint16_t fric_speed=1005;
uint8_t  fric_actual_speed=15;
extern fifo_s_t Referee_FIFO;

uint16_t textspeed=2000;

#ifndef fric_15m
uint16_t fric_15m = 1475;
uint16_t fric_18m = 1515;
uint16_t fric_30m = 1690;//1750;
#endif

void Shoot_Motor_Init(void)
{
	const static fp32 shoot_motor_speed_pid[3] = {SHOOT_MOTOR_SPEED_PID_KP, SHOOT_MOTOR_SPEED_PID_KI, SHOOT_MOTOR_SPEED_PID_KD};
	const static fp32 shoot_motor_angle_pid[3] = {SHOOT_MOTOR_ANGLE_PID_KP, SHOOT_MOTOR_ANGLE_PID_KI, SHOOT_MOTOR_ANGLE_PID_KD};
	
	shoot_m2006[0].speed=0;
	shoot_m2006[0].speed_set=0;
	shoot_m2006[0].angle=0;
	shoot_m2006[0].angle_set=0;
	shoot_m2006[0].ENC_angle=0;
	shoot_m2006[0].give_current=0;
	
	PID_init(&shoot_m2006[0].speed_pid,PID_DELTA,shoot_motor_speed_pid,SHOOT_MOTOR_SPEED_PID_MAX_OUT,SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_m2006[0].angle_pid,PID_POSITION,shoot_motor_angle_pid,SHOOT_MOTOR_ANGLE_PID_MAX_OUT,SHOOT_MOTOR_ANGLE_PID_MAX_IOUT);
}

void Shoot_Motor_Data_Update(void)
{
	shoot_m2006[0].speed=motor_measure_gimbal[2].speed_rpm;
	shoot_m2006[0].angle=dial_angle;
}

//extern ext_shoot_data_t shoot_data_t;
//float bullet_speed_max=0,bullet_speed_min=30;
void Fric_Motor_Control(void)
{
//	if(Game_Robot_Status.shooter_id1_17mm_speed_limit>=30)
//	{
////		if(dial_mode==1)
			fric_speed=fric_30m;
		  fric_actual_speed=30;
////		else
////			fric_speed=fric_15m;
//	}
//	else if(Game_Robot_Status.shooter_id1_17mm_speed_limit>=18)
//	{
//		fric_speed=fric_18m;
//	  fric_actual_speed=18;
//	}
//	else
//	{
//		fric_speed=fric_15m;
//		fric_actual_speed=15;
//	}
    

//		fric_speed=fric_15m;
//	  FRIC_PWM_R=textspeed;
//	  FRIC_PWM_L=textspeed;
    Fric_PWR(fric_state);
	
//	if(shoot_data_t.bullet_speed>bullet_speed_max)
//		bullet_speed_max=shoot_data_t.bullet_speed;
//	if(shoot_data_t.bullet_speed<bullet_speed_min&&shoot_data_t.bullet_speed!=0)
//		bullet_speed_min=shoot_data_t.bullet_speed;
}

uint16_t dial_single_cnt=0;
uint32_t dial_stop_cnt=0;
uint8_t  preload_flag=1;
uint32_t dial_load_cnt=0;
void Dial_Motor_Control(void)
{
	if(Game_Robot_Status.power_management_shooter_output==0x01)
	{
		if(abs(shoot_m2006[0].give_current)>12000&&fabs(shoot_m2006[0].speed)<100)//防堵转
		{
			dial_stop_cnt++;
			if(dial_stop_cnt>100)
			{
				if(shoot_m2006[0].give_current>0)
					shoot_m2006[0].give_current=-3000;
				else
					shoot_m2006[0].give_current=3000;
				vTaskDelay(200);
//				Shoot_Motor_Init();
				PID_clear(&shoot_m2006[0].speed_pid);
				PID_clear(&shoot_m2006[0].angle_pid);
				dial_stop_cnt=0;
				shoot_m2006[0].angle_set=dial_angle;
			}
		}
	}
	
	if(fric_state==1)
	{
		if(dial_mode==0)//continue
		{
			if(rc_ctrl.mouse.press_r||rc_ctrl .rc .ch [4]>=300)
			{
//				if(ABS(gimbal_LK[0].auto_aim_pid.error[0])>1.0f)
//					shoot_m2006[0].speed_set=0;
//				else 
					shoot_m2006[0].speed_set=dial_speed;
			}
			else
				shoot_m2006[0].speed_set=dial_speed;
		}
		else//single
		{	
			if(if_single_hit==1)
			{
				dial_single_cnt++;
				if(dial_single_cnt>50)
				{
					if_single_hit=0;
					dial_single_cnt=0;
				}
			}
			
			PID_calc(&shoot_m2006[0].angle_pid,shoot_m2006[0].angle,shoot_m2006[0].angle_set);
			shoot_m2006[0].speed_set=shoot_m2006[0].angle_pid.out;
		}
	}
	else
	{
		shoot_m2006[0].speed_set=0;
	}
//	if((rc_ctrl.mouse.press_r||rc_ctrl .rc .ch [4]>=300)&&AutoAim_Data_Receive .Shoot_Freq==0&&dial_speed!=0)
//	{ 
//		if(preload_flag)
//	  { 
//			shoot_m2006[0].speed_set=dial_speed*0.25f;
//			if(shoot_m2006[0].give_current<-1500)
//			{ dial_load_cnt++;
//				if(dial_load_cnt>3)
//				{
//					shoot_m2006[0].speed_set=0;
//				  preload_flag=0;
//					dial_load_cnt=0;
//				}
//			}
//	  }
//		else
//		shoot_m2006[0].speed_set=0;
//	}
//	else
//	preload_flag=1;
//	if(Heat_allow_shoot<=15&&Heat_allow_shoot!=-1)
//	{
//		shoot_m2006[0].speed_set = 0; 
//		shoot_m2006[0].angle_set=shoot_m2006[0].angle;
//	}
	PID_calc(&shoot_m2006[0].speed_pid,shoot_m2006[0].speed,shoot_m2006[0].speed_set);
	shoot_m2006[0].give_current=shoot_m2006[0].speed_pid.out;
}

void Shoot_Task(void const * argument)
{
	Shoot_Motor_Init();
	vTaskDelay(200);
	
	while(1)
	{		
		Shoot_Motor_Data_Update();

		Fric_Motor_Control();
		Dial_Motor_Control();
		
		vTaskDelay(2);
	}
}
//摩擦轮缓启动
kf_data_t fric_kf={.Q=0.01,.R=100,.Last_P=1};
void Fric_PWR(uint8_t power){
//	if(power){
//		if(FRIC_PWM_L < fric_speed){						//摩擦轮PWM值未达到
//			FRIC_PWM_L += 3, FRIC_PWM_R += 3;	//PWM值持续增大，摩擦轮功率逐渐增大
//		}
//	}else{
//		if(FRIC_PWM_L >= 500){					//摩擦轮PWM值未达到 
//			FRIC_PWM_L -= 4,FRIC_PWM_R -= 4;			//PWM值持续减小，摩擦轮功率逐渐减小
//		}
//	}
	
//	if(power)
//	{
//		FRIC_PWM_L = FRIC_PWM_R = fric_speed;
//	}
//	else
//	{
//		FRIC_PWM_L = FRIC_PWM_R = 500;
//	}
//	
	if(power)
	{
		KalmanFilterCalc(&fric_kf, fric_speed);
	}
	else
	{
		KalmanFilterCalc(&fric_kf, 500);
	}
	FRIC_PWM_L = FRIC_PWM_R = (uint16_t)fric_kf.out;
}