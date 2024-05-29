#ifndef _MY_DEF_
#define _MY_DEF_

#include "main.h"
#include "remote_control.h"
#include "tim.h"
extern RC_ctrl_t rc_ctrl_last;

//Model Switch
//#define INFANTRY_HELM_OLD  
#define INFANTRY_HELM_NEW
//LED
#define LED_PIN_PORT		GPIOH	

#define LED_RED_PIN			GPIO_PIN_12		
#define LED_GREEN_PIN		GPIO_PIN_11		
#define LED_BLUE_PIN		GPIO_PIN_10		

//buzzer
#define TIM_BUZZER 				htim4
#define CHANNEL_BUZZER 		TIM_CHANNEL_3
#define BUZZER_CCR 				TIM4->CCR3
//ÔËËãºê
#define SIGNAL(n)										((n)>=0 ? 1 : -1)
#define ABS(n)											((n)>=0 ? (n) : -(n))
#define DATA_LIMIT(n,min,max)				((n)>(max) ? (max) : ((n)<(min) ? (min) : (n)))
#define DEADBAND(n,db)							(ABS(n) >= ABS(db) ? (n) : 0)
#define RAMP_CTRL(ref,set,acc)	((ref) + DATA_LIMIT((acc),0,1) * ((set) - (ref)))
#define RAMP(ref,set,add)	((ref)<(set) ? (ref)+(add) : (ref)-(add))

//RMÒ£¿ØÆ÷
#define RollWheel							rc_ctrl.rc.ch[4]			//Ò£¿ØÆ÷×ó²à¹öÂÖ£¬×ó¡úÓÒ£º660¡ú-660
#define RockingBar_Left_V			rc_ctrl.rc.ch[3]			//Ò£¿ØÆ÷×óÒ¡¸Ë£¨´¹Ö±£©£¬ÉÏ¡úÏÂ£º660¡ú-660
#define RockingBar_Left_H			rc_ctrl.rc.ch[2]			//Ò£¿ØÆ÷×óÒ¡¸Ë£¨Ë®Æ½£©£¬×ó¡úÓÒ£º-660¡ú660
#define RockingBar_Right_V		rc_ctrl.rc.ch[1]			//Ò£¿ØÆ÷ÓÒÒ¡¸Ë£¨´¹Ö±£©£¬ÉÏ¡úÏÂ£º660¡ú-660
#define RockingBar_Right_H		rc_ctrl.rc.ch[0]			//Ò£¿ØÆ÷ÓÒÒ¡¸Ë£¨Ë®Æ½£©£¬×ó¡úÓÒ£º-660¡ú660
#define Switch_Left						rc_ctrl.rc.s[1]				//Ò£¿ØÆ÷×ó²¦¸Ë£¬ÉÏ1£¬ÖÐ3£¬ÏÂ2
#define Switch_Right					rc_ctrl.rc.s[0]				//Ò£¿ØÆ÷ÓÒ²¦¸Ë£¬ÉÏ1£¬ÖÐ3£¬ÏÂ2

//¿Í»§¶Ë¼üÊó
#define Mouse_Left					(rc_ctrl.mouse.press_l)		//Êó±ê×ó¼ü
#define Mouse_Right					(rc_ctrl.mouse.press_r)		//Êó±êÓÒ¼ü
#define MOUSE_X							rc_ctrl.mouse.x						//Êó±êÖ¸ÕëX
#define MOUSE_Y							rc_ctrl.mouse.y						//Êó±êÖ¸ÕëY
#define MOUSE_Z							rc_ctrl.mouse.z						//Êó±êÖ¸ÕëZ
#define KEY_W								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)				//¼üÅÌ°´¼üW
#define KEY_S								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)				//¼üÅÌ°´¼üS
#define KEY_A								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)				//¼üÅÌ°´¼üA
#define KEY_D								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)				//¼üÅÌ°´¼üD
#define KEY_SHIFT						(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)		//¼üÅÌ°´¼üSHIFT
#define KEY_CTRL						(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)			//¼üÅÌ°´¼üCTRL
#define KEY_Q								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)				//¼üÅÌ°´¼üQ
#define KEY_E								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)				//¼üÅÌ°´¼üE
#define KEY_R								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)				//¼üÅÌ°´¼üR
#define KEY_F								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)				//¼üÅÌ°´¼üF
#define KEY_G								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)				//¼üÅÌ°´¼üG
#define KEY_Z								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)				//¼üÅÌ°´¼üZ
#define KEY_X								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)				//¼üÅÌ°´¼üX
#define KEY_C								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)				//¼üÅÌ°´¼üC
#define KEY_V								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)				//¼üÅÌ°´¼üV
#define KEY_B								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)				//¼üÅÌ°´¼üB
#define KEY_W_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_W)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üW
#define KEY_S_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_S)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üS
#define KEY_A_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_A)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üA
#define KEY_D_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_D)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üD
#define KEY_Q_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_Q)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üQ
#define KEY_E_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_E)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üE
#define KEY_R_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_R)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üR
#define KEY_F_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_F)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üF
#define KEY_G_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_G)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üG
#define KEY_Z_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_Z)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üZ
#define KEY_X_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_X)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üX
#define KEY_C_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_C)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üC
#define KEY_V_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_V)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üV
#define KEY_B_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_B)				//Ç°Ò»Ê±¿Ì¼üÅÌ°´¼üB
//¼üÅÌ¹¦ÄÜ
#define KEYB_TURN_BACK		(!KEY_Z_LAST && KEY_Z)									//µ÷Í·
#define KEYB_AIM_MODE			(!KEY_Q_LAST && KEY_Q)									//×ÔÃéÄ£Ê½
#define KEYB_AIM_ARMOR		(!KEY_F_LAST && KEY_F)									//×ÔÃé×°¼×°å
#define KEYB_BOX_LID			(!KEY_CTRL && KEY_R)										//µ¯²Õ¸Ç
#define KEYB_FLASH_UI			(KEY_CTRL && KEY_R)											//Ë¢ÐÂUI
#define KEYB_SPEED_GEAR		(!KEY_V_LAST && KEY_V)									//ËÙ¶Èµ²Î»
#define KEYB_FALL_RESUME	(!KEY_CTRL && KEY_B)										//ÆðÁ¢(±£»¤ºó)
#define KEYB_MCU_RESET		(KEY_CTRL && KEY_B)											//¸´Î»
#define KEYB_FLY					(KEY_X)																	//·ÉÆÂ
#define KEYB_TALL_STAND		(!KEY_C_LAST && KEY_C)									//Õ¾Á¢
#define KEYB_JUMP_OVER		(!KEY_G_LAST && KEY_G)									//Ìø
#define KEYB_ACROSS_TUNNEL  (KEY_F)																//×ê¶´


//×ËÌ¬½Ç
#define PITCH							INS_angle_deg[2]			//¸©Ñö½Ç
#define ROLL							INS_angle_deg[1]			//·­¹ö½Ç
#define YAW								INS_angle_deg[0]			//Æ«º½½Ç
#define GYROX							INS_gyro[0]
#define GYROY							INS_gyro[1]						
#define GYROZ							INS_gyro[2]
#define ACCELX						accel_fliter_3[0]
#define ACCELY						accel_fliter_3[1]						
#define ACCELZ						accel_fliter_3[2]
#define QUATX							INS_quat[0]
#define QUATY							INS_quat[1]						
#define QUATZ							INS_quat[2]
#define QUATW							INS_quat[3]

//·¢Éä»ú¹¹
#define LASER							TIM3->CCR3					//ºìµã¼¤¹âÆ÷
#define FRIC_PWM_L				TIM1->CCR2					//Ä¦²ÁÂÖ£¨×ó£©
#define FRIC_PWM_R				TIM1->CCR3					//Ä¦²ÁÂÖ£¨ÓÒ£©
#define BOX_LID(n)		TIM1->CCR1 = (n ? 1010 : 2560) 	//µ¯²Ö¸Ç£¬1¿ª£¬0±Õ

#endif
