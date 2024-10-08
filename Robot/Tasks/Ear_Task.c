#include "Ear_Task.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "tim.h"
#include "remote_control.h"
#include "chassis_task.h"

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : (-x))

/* 耳朵动作变量 */
uint8_t EarL_PWMSet   = 120;
uint8_t EarL_PWMReset = 200;
uint8_t EarL_PWMval;
uint8_t EarR_PWMSet   = 200;
uint8_t EarR_PWMReset = 120;
uint8_t EarR_PWMval;
uint8_t Ear_Direction; //0向RESET靠近 1向SET靠近
uint8_t Ear_Move_Flag;
uint8_t Ear_Cnt;

/* 耳朵动作参考变量 */
extern shoot_motor_t shoot_m2006[1];

void Ear_Task(void const * argument)
{
	/* 设置耳朵定时器PWM */
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	EarL_PWMval = EarL_PWMReset;
	EarR_PWMval = EarR_PWMReset;
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, EarR_PWMval);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, EarL_PWMval);
	Ear_Direction = 1;
	Ear_Move_Flag = 5;
	
	/* 标定耳朵位置 */
//	while(1)
//	{
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, EarR_PWMval);
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, EarL_PWMval);
//		vTaskDelay(100);
//	}
	
//	/* 播放开机音乐 */
//	vTaskDelay(500);
//	Buzzer_PlayMusic(Music_MiXueBingCheng);
//	vTaskDelay(500);
	
	while(1)
	{
		if(Ear_Cnt > 0) Ear_Cnt--;
		
		/* 打弹、底盘无力、开弹仓时耳朵低垂 */
		if(abs(shoot_m2006[0].speed_set) > 0 && abs(shoot_m2006[0].speed) > 1000) Ear_Cnt = 5;
		if(rc_ctrl.rc.s[1] != RC_SW_UP && rc_ctrl.rc.s[1] != RC_SW_MID) Ear_Cnt = 10;
		if(rc_ctrl.rc.s[0] == RC_SW_DOWN || (rc_ctrl.key.v & KEY_PRESSED_OFFSET_R) != 0 ) Ear_Cnt = 20;
		
		if(Ear_Cnt >  0)
		{
			EarL_PWMval = EarL_PWMSet;
			EarR_PWMval = EarR_PWMSet;
			Ear_Direction = 0;
			Ear_Move_Flag = 5;
		}
		else
		{
			/* 旋转时耳朵动作优先级1 */
			     if(rc_ctrl.rc.ch[0] > 10 || rc_ctrl.mouse.x > 10)
			{
				EarL_PWMval = EarL_PWMReset - 20;
				EarR_PWMval = EarR_PWMSet - 20;
				Ear_Direction = 0;
				Ear_Move_Flag = 5;
			}
			else if(rc_ctrl.rc.ch[0] < -10 || rc_ctrl.mouse.x < -10)
			{
				EarL_PWMval = EarL_PWMSet + 20;
				EarR_PWMval = EarR_PWMReset + 20;
				Ear_Direction = 1;
				Ear_Move_Flag = 5;
			}
			/* 前后时耳朵动作优先级2 */
			else if(rc_ctrl.rc.ch[3] > 10 || (rc_ctrl.key.v&KEY_PRESSED_OFFSET_W) != 0)
			{
				EarL_PWMval = EarL_PWMSet;
				EarR_PWMval = EarR_PWMSet;
				Ear_Direction = 0;
				Ear_Move_Flag = 5;
			}
			else if(rc_ctrl.rc.ch[3] < -10 || (rc_ctrl.key.v&KEY_PRESSED_OFFSET_S) != 0)
			{
				EarL_PWMval = EarL_PWMReset;
				EarR_PWMval = EarR_PWMReset;
				Ear_Direction = 1;
				Ear_Move_Flag = 5;
			}
			/* 横移时耳朵动作优先级3 */
			else if(rc_ctrl.rc.ch[2] > 10 || (rc_ctrl.key.v&KEY_PRESSED_OFFSET_A) != 0)
			{
				EarL_PWMval = (EarL_PWMReset + EarL_PWMSet) >> 1;
			  EarR_PWMval = (EarR_PWMReset + EarR_PWMSet) >> 1;
				Ear_Direction = 0;
				Ear_Move_Flag = 5;
			}
			else if(rc_ctrl.rc.ch[2] < -10 || (rc_ctrl.key.v&KEY_PRESSED_OFFSET_D) != 0)
			{
				EarL_PWMval = (EarL_PWMReset + EarL_PWMSet) >> 1;
			  EarR_PWMval = (EarR_PWMReset + EarR_PWMSet) >> 1;
				Ear_Direction = 1;
				Ear_Move_Flag = 5;
			}
			/* 静止时耳朵动作优先级4 */
			else
			{
				if(Ear_Move_Flag > 0)
				{
					EarL_PWMval = (EarL_PWMReset + EarL_PWMSet) >> 1;
					EarR_PWMval = (EarR_PWMReset + EarR_PWMSet) >> 1;
				  Ear_Move_Flag --;
				}
				else
				{
					if(Ear_Direction == 0)
					{
						EarL_PWMval = min(EarL_PWMval + 1, EarL_PWMReset);
						EarR_PWMval = max(EarR_PWMval - 1, EarR_PWMReset);
						if(EarL_PWMval == EarL_PWMReset && EarR_PWMval == EarR_PWMReset) Ear_Direction = 1;
					}
					if(Ear_Direction == 1)
					{
						EarL_PWMval = max(EarL_PWMval - 1, EarL_PWMSet);
						EarR_PWMval = min(EarR_PWMval + 1, EarR_PWMSet);
						if(EarL_PWMval == EarL_PWMSet && EarR_PWMval == EarR_PWMSet) Ear_Direction = 0;
					}
				}
			}
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, EarR_PWMval);
	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, EarL_PWMval);
		
		vTaskDelay(10);
	}
}
