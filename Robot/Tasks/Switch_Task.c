#include "Switch_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Shoot_Task.h"
#include "remote_control.h"
#include "referee.h"
#include "Chassis_Task.h"
#include "My_Def.h"
#include "Usb_Task.h"
#include "referee_usart_task.h"

#ifdef INFANTRY_HELM_NEW
#define DIAL_SPEED 4320		//16*270
#endif
#ifdef INFANTRY_HELM_OLD
#define DIAL_SPEED (4320*4/3)  //16*360
#endif

RC_ctrl_t rc_ctrl_last;

//uint8_t if_magazine_open=0;//0:off,1:on
extern shoot_motor_t shoot_m2006[1];
extern uint8_t dial_mode,if_single_hit;//mode:0 continue,1 single;
extern int16_t dial_speed;
extern int32_t dial_angle;
extern uint8_t fric_state;
extern ext_robot_status_t Game_Robot_Status;
extern fifo_s_t Referee_FIFO;
uint32_t shoot_power_good_cnt=0;
uint8_t if_predict=0;//0:no,1:yes
uint8_t autoaim_last_shoot_freq=0;
uint8_t autoaim_shoot_freq=0;
uint8_t dial_mode_last=0;
float auto_aim_pitch_offset=0;

uint32_t switch_180_cnt=501;
uint8_t chassis_follow_gimbal_changing=0;

void Switch_Task(void const * argument)
{
	while(1)
	{	
		//180 rotate
		if(KEYB_TURN_BACK)
		{
			gimbal_LK[0].INS_angle_set=gimbal_LK[0].INS_angle+180;
			switch_180_cnt=0;
			chassis_follow_gimbal_changing=1;
		}	
		if(switch_180_cnt<500)
		{
			switch_180_cnt++;
		}
		if(switch_180_cnt==500)
		{
			chassis_follow_gimbal_changing=0;
			switch_180_cnt++;
		}
		
		//fric
		if((Game_Robot_Status.power_management_shooter_output==0x01)&&shoot_power_good_cnt<=2000) shoot_power_good_cnt++;//delay 2000*2 ms
		else if(Game_Robot_Status.power_management_shooter_output==0x00) shoot_power_good_cnt=0;
		if((rc_ctrl.rc.s[0]==RC_SW_MID||rc_ctrl.rc.s[0]==RC_SW_UP)&&shoot_power_good_cnt>=2000)
		{
			fric_state=1;
		}
		else
		{
			fric_state=0;	
		}
    
		
		if(fric_state==1)
		{
			//dial
			if(dial_mode==0)//Á¬·¢
			{
				if((rc_ctrl.rc.s[0]==RC_SW_UP||rc_ctrl.mouse.press_l)&&(Power_Heat_Data.shooter_17mm_1_barrel_heat<Game_Robot_Status.shooter_barrel_heat_limit-30||KEYB_FORCED_FIRING))    
				{
					dial_speed = DIAL_SPEED;
				}
				else
					dial_speed=0;
			}
			else               
			{
				if((!(rc_ctrl_last.mouse.press_l)&&(rc_ctrl.mouse.press_l))||
					 (!(rc_ctrl_last.rc.s[0]==RC_SW_UP)&&(rc_ctrl.rc.s[0]==RC_SW_UP)))//pc or remote control single shot
				{
					if(if_single_hit==0)
					{
						if_single_hit=1;
						#ifdef INFANTRY_HELM_NEW
						shoot_m2006[0].angle_set=shoot_m2006[0].angle+8192*36/8;
						#endif
						#ifdef INFANTRY_HELM_OLD
						shoot_m2006[0].angle_set=shoot_m2006[0].angle+8192*36/6;
						#endif
					}
				}
				
//				if((Autoaim_Mode==0 || Autoaim_Mode==1) && autoaim_last_shoot_freq==0 && nuc_receive_data.aim_data_received.target_rate!=0 && (rc_ctrl.rc.s[0]==RC_SW_UP||rc_ctrl.mouse.press_l))//antitop automantic firing
//				{
//					autoaim_shoot_freq=nuc_receive_data.aim_data_received.target_rate;//autoaim shoot frequence update
//					if(if_single_hit==0)
//					{
//						if_single_hit=1;
//						shoot_m2006[0].angle_set=shoot_m2006[0].angle+8192*60/10*(float)(autoaim_shoot_freq/10);//autoaim_shoot_freq/10=firing times per armor plate
//					}
//				}
			}
		}
		
		if(KEYB_SPEED_GEAR)
		{
			Speed_Mode++;
			if(Speed_Mode>2) Speed_Mode=0; 
		}
		
		if(KEYB_AIM_MODE)
		{
			//autoaim mode
			Autoaim_Mode++;
			if(Autoaim_Mode>3) Autoaim_Mode=0;
		}
		//dial mode
		if(Autoaim_Mode==0||(Autoaim_Mode==1&&nuc_receive_data.aim_data_received.target_number<9)) dial_mode=0;
		else if(Autoaim_Mode==2||Autoaim_Mode==3) dial_mode=1;
		if(dial_mode==1&&dial_mode_last==0) shoot_m2006[0].angle_set=shoot_m2006[0].angle;
		autoaim_last_shoot_freq =nuc_receive_data.aim_data_received.target_rate;
		rc_ctrl_last=rc_ctrl;	
		dial_mode_last=dial_mode;
		vTaskDelay(2);
	}
}
