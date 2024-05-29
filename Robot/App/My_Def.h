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
//�����
#define SIGNAL(n)										((n)>=0 ? 1 : -1)
#define ABS(n)											((n)>=0 ? (n) : -(n))
#define DATA_LIMIT(n,min,max)				((n)>(max) ? (max) : ((n)<(min) ? (min) : (n)))
#define DEADBAND(n,db)							(ABS(n) >= ABS(db) ? (n) : 0)
#define RAMP_CTRL(ref,set,acc)	((ref) + DATA_LIMIT((acc),0,1) * ((set) - (ref)))
#define RAMP(ref,set,add)	((ref)<(set) ? (ref)+(add) : (ref)-(add))

//RMң����
#define RollWheel							rc_ctrl.rc.ch[4]			//ң���������֣�����ң�660��-660
#define RockingBar_Left_V			rc_ctrl.rc.ch[3]			//ң������ҡ�ˣ���ֱ�����ϡ��£�660��-660
#define RockingBar_Left_H			rc_ctrl.rc.ch[2]			//ң������ҡ�ˣ�ˮƽ��������ң�-660��660
#define RockingBar_Right_V		rc_ctrl.rc.ch[1]			//ң������ҡ�ˣ���ֱ�����ϡ��£�660��-660
#define RockingBar_Right_H		rc_ctrl.rc.ch[0]			//ң������ҡ�ˣ�ˮƽ��������ң�-660��660
#define Switch_Left						rc_ctrl.rc.s[1]				//ң�����󲦸ˣ���1����3����2
#define Switch_Right					rc_ctrl.rc.s[0]				//ң�����Ҳ��ˣ���1����3����2

//�ͻ��˼���
#define Mouse_Left					(rc_ctrl.mouse.press_l)		//������
#define Mouse_Right					(rc_ctrl.mouse.press_r)		//����Ҽ�
#define MOUSE_X							rc_ctrl.mouse.x						//���ָ��X
#define MOUSE_Y							rc_ctrl.mouse.y						//���ָ��Y
#define MOUSE_Z							rc_ctrl.mouse.z						//���ָ��Z
#define KEY_W								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)				//���̰���W
#define KEY_S								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)				//���̰���S
#define KEY_A								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)				//���̰���A
#define KEY_D								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)				//���̰���D
#define KEY_SHIFT						(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)		//���̰���SHIFT
#define KEY_CTRL						(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)			//���̰���CTRL
#define KEY_Q								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)				//���̰���Q
#define KEY_E								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)				//���̰���E
#define KEY_R								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)				//���̰���R
#define KEY_F								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)				//���̰���F
#define KEY_G								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)				//���̰���G
#define KEY_Z								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)				//���̰���Z
#define KEY_X								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)				//���̰���X
#define KEY_C								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)				//���̰���C
#define KEY_V								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)				//���̰���V
#define KEY_B								(rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)				//���̰���B
#define KEY_W_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_W)				//ǰһʱ�̼��̰���W
#define KEY_S_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_S)				//ǰһʱ�̼��̰���S
#define KEY_A_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_A)				//ǰһʱ�̼��̰���A
#define KEY_D_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_D)				//ǰһʱ�̼��̰���D
#define KEY_Q_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_Q)				//ǰһʱ�̼��̰���Q
#define KEY_E_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_E)				//ǰһʱ�̼��̰���E
#define KEY_R_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_R)				//ǰһʱ�̼��̰���R
#define KEY_F_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_F)				//ǰһʱ�̼��̰���F
#define KEY_G_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_G)				//ǰһʱ�̼��̰���G
#define KEY_Z_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_Z)				//ǰһʱ�̼��̰���Z
#define KEY_X_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_X)				//ǰһʱ�̼��̰���X
#define KEY_C_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_C)				//ǰһʱ�̼��̰���C
#define KEY_V_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_V)				//ǰһʱ�̼��̰���V
#define KEY_B_LAST					(rc_ctrl_last.key.v & KEY_PRESSED_OFFSET_B)				//ǰһʱ�̼��̰���B
//���̹���
#define KEYB_TURN_BACK		(!KEY_Z_LAST && KEY_Z)									//��ͷ
#define KEYB_AIM_MODE			(!KEY_Q_LAST && KEY_Q)									//����ģʽ
#define KEYB_AIM_ARMOR		(!KEY_F_LAST && KEY_F)									//����װ�װ�
#define KEYB_BOX_LID			(!KEY_CTRL && KEY_R)										//���ո�
#define KEYB_FLASH_UI			(KEY_CTRL && KEY_R)											//ˢ��UI
#define KEYB_SPEED_GEAR		(!KEY_V_LAST && KEY_V)									//�ٶȵ�λ
#define KEYB_FALL_RESUME	(!KEY_CTRL && KEY_B)										//����(������)
#define KEYB_MCU_RESET		(KEY_CTRL && KEY_B)											//��λ
#define KEYB_FLY					(KEY_X)																	//����
#define KEYB_TALL_STAND		(!KEY_C_LAST && KEY_C)									//վ��
#define KEYB_JUMP_OVER		(!KEY_G_LAST && KEY_G)									//��
#define KEYB_ACROSS_TUNNEL  (KEY_F)																//�궴


//��̬��
#define PITCH							INS_angle_deg[2]			//������
#define ROLL							INS_angle_deg[1]			//������
#define YAW								INS_angle_deg[0]			//ƫ����
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

//�������
#define LASER							TIM3->CCR3					//��㼤����
#define FRIC_PWM_L				TIM1->CCR2					//Ħ���֣���
#define FRIC_PWM_R				TIM1->CCR3					//Ħ���֣��ң�
#define BOX_LID(n)		TIM1->CCR1 = (n ? 1010 : 2560) 	//���ָǣ�1����0��

#endif
