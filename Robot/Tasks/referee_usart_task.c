#include "referee_usart_task.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "shoot_task.h"
#include "Gimbal_Task.h"
#include "Chassis_Task.h"
#include "bsp_cap.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "Usb_Task.h"
#include "INS_task.h"
#include "arm_math.h"

/* Private define ------------------------------------------------------------*/
#define Max(a,b) ((a) > (b) ? (a) : (b))
//#define Robot_ID_Current Robot_ID_Blue_Infantry3
//#define Robot_ID_Current Robot_ID_Red_Infantry3
/* Private variables ---------------------------------------------------------*/
/* ����ϵͳ����˫������ */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* ����ϵͳ�������ݶ��� */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol�������ṹ�� */
unpack_data_t Referee_Unpack_OBJ;

/* ��̬UI���ݱ��� */
#ifndef	Robot_ID_Current
uint8_t Robot_ID_Current;
#endif

#ifndef GIMBAL_ECD_RANGE
#define GIMBAL_ECD_RANGE 65536
#endif
uint8_t UI_Update_Flag = 0;    //�Ƿ��������־λ
float   UI_Kalman_Speed = 0;    //������Ԥ���ٶ�
float   UI_Gimbal_Pitch = 0.0f; //��̨Pitch��Ƕ�
float   UI_Gimbal_Roll = 0.0f; //��̨Pitch��Ƕ�
float   UI_Gimbal_Yaw   = 0.0f; //��̨Yaw��Ƕ�
float   UI_Chassis_Follow_Gimbal_Angle=0.0f;//���̸�����̨�Ƕ�
uint8_t UI_attack_direction=0;			//�ܻ���������
uint8_t UI_attack_flag=0;						//�ܻ�����ͼ���Ƿ��ѻ���
uint16_t UI_attack_direction_Counter=0;	//�ܻ�����ͼ��ɾ����ʱ
uint8_t UI_HP_deduction_reason=0;
uint8_t UI_Hurt_armor_id=0;
uint16_t UI_robot_current_HP=0;					//�����˵�ǰѪ��
uint16_t UI_robot_last_HP=0;						//�������ϴ�Ѫ��
uint8_t UI_AutoAim_Mode=0;
uint8_t UI_AutoAim_Mode_last=0;
uint8_t UI_Speed_Mode=0;
uint8_t UI_Speed_Mode_last=0;
uint8_t UI_Capacitance  = 10;   //����ʣ������
uint8_t UI_fric_mode   = 0;     //Ħ�����Ƿ���
uint8_t UI_fric_flag		=0;			//Ħ����ͼ���Ƿ��ѻ���
uint8_t UI_lock_mode   = 0;     //���������Ƿ���
uint8_t UI_lock_flag		=0;			//��������ͼ���Ƿ��ѻ���

uint16_t UI_Move_Start_X=611;
uint16_t UI_Move_End_X=850;
uint16_t UI_Move_End_Y=300;
float UI_Move_K=0;

/* �����߸߶ȱ��� */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;
/*С̹��λ�ò���*/
uint16_t tank_x =600;
uint16_t tank_y =200;
/*Ѫ������λ��*/
uint16_t HP_x = 600;
uint16_t HP_y = 750;
extern uint8_t if_predict;

uint8_t statics=0;

/**
  * @brief  ת�����Ƶ�0~360
  * @param  ����ת��
  * @retval ���ת��
  */
uint16_t limit_360deg(int16_t in)
{
	while(in < 0 || in > 360)
	{
		if (in < 0)
			in = in + 360;
		if (in > 360)
			in = in - 360;
	}
	return in;
}
int16_t angle_err=0;
void referee_usart_task(void const * argument)
{
	/* ��̬UI���Ʊ��� */
	float    Capacitance_X;
	UI_Update_Flag=1;
	/* ����ϵͳ��ʼ�� */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		/* ��������ϵͳ���� */
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		#ifndef	Robot_ID_Current
		Robot_ID_Current = Game_Robot_Status.robot_id;
		#endif
		UI_Gimbal_Pitch=gimbal_LK[1].INS_angle;
		UI_Gimbal_Roll =INS_angle_deg[1];
		UI_fric_mode	=	fric_state;
		UI_lock_mode  =	nuc_receive_data.aim_data_received.success;
		UI_HP_deduction_reason=Robot_Hurt.HP_deduction_reason;
		UI_Hurt_armor_id=Robot_Hurt.armor_id;
		UI_robot_last_HP=UI_robot_current_HP;
		UI_robot_current_HP=Game_Robot_Status.current_HP;
		UI_AutoAim_Mode_last=UI_AutoAim_Mode;
		UI_AutoAim_Mode=Autoaim_Mode;
		UI_Speed_Mode_last=UI_Speed_Mode;
		UI_Speed_Mode=Speed_Mode;
    UI_Capacitance=cap_data.cap_per * 100;
		UI_Chassis_Follow_Gimbal_Angle=chassis_control.chassis_follow_gimbal_zero;
		if(UI_Gimbal_Pitch<=0&&UI_Gimbal_Pitch>=-25)
		{
			UI_Move_Start_X=10.693f*UI_Gimbal_Pitch + 624.75f;
			UI_Move_K=-0.00005f*UI_Gimbal_Pitch*UI_Gimbal_Pitch*UI_Gimbal_Pitch - 0.0016f*UI_Gimbal_Pitch*UI_Gimbal_Pitch - 0.023f*UI_Gimbal_Pitch + 1.275f;
			UI_Move_End_X=UI_Move_Start_X+UI_Move_End_Y/UI_Move_K;
		}
		/* UI���� */
		if(KEYB_FLASH_UI)
		{
			//��̬UIԤ���� ������1
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  840,   y01,  920,   y01); //��һ�������
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 1,  950,   y01,  970,   y01); //��һ��ʮ�ֺ�
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //��һ���Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //��һ��ʮ����
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //�ڶ��������
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //�ڶ������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  989,   y02,  1049,  y02); //�ڶ��������
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			osDelay(30);
			
			//��̬UIԤ���� ������2
			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //�����������
			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //���������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //�������Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //�����������
			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //���������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //�������Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //������������
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);		
			osDelay(30);
			
			//��̬UIԤ���� С����Ԥ���� AUTO_AIM_MODE
			UI_Draw_Line(&UI_Graph7.Graphic[0], "015", UI_Graph_Add, 0, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[1], "016", UI_Graph_Add, 0, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[2], "017", UI_Graph_Add, 0, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[3], "018", UI_Graph_Add, 0, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Draw_Line(&UI_Graph7.Graphic[4], "019", UI_Graph_Add, 0, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_Draw_Arc	(&UI_Graph7.Graphic[5], "020", UI_Graph_Add, 0, UI_Color_Yellow, 150, 30, 3, 1880,  550,80,80);		//����ģʽԲ��
			UI_Draw_Arc	(&UI_Graph7.Graphic[6], "021", UI_Graph_Add, 0, UI_Color_Yellow, 150, 30, 3, 1880,  550,180,180); //����ģʽԲ��
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			osDelay(30);
		
			//��̬UIԤ���� �ַ���1
			UI_Draw_String(&UI_String.String, "101", UI_Graph_Add, 1, UI_Color_Green,  26, 6, 3,  50, 785, "Fric  "); //Ħ�����Ƿ���
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			
			//��̬UIԤ���� �ַ���2
			UI_Draw_String(&UI_String.String, "102", UI_Graph_Add, 1, UI_Color_Green,  26, 6, 3,  50, 735, "Lock  "); //�����Ƿ�����
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			
			//��̬UIԤ���� �ַ���2
			UI_Draw_String(&UI_String.String, "103", UI_Graph_Add, 1, UI_Color_Green,  26, 6, 3,  665, 172, "Cap   "); //��������
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			
			
			//��̬UIԤ���� ͼ�� and Move_Predict
			UI_Draw_Float (&UI_Graph7.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pitch��Ƕ�
			UI_Draw_Line  (&UI_Graph7.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Green , 2 , 960-300, 600, 960+300, 600);//Roll��Ƕ�
			UI_Draw_Line  (&UI_Graph7.Graphic[2], "209", UI_Graph_Add, 2, UI_Color_White , 2 , UI_Move_Start_X, 0, UI_Move_End_X, UI_Move_End_Y);//ǰ����
			UI_Draw_Line  (&UI_Graph7.Graphic[3], "210", UI_Graph_Add, 2, UI_Color_White , 2 , 1920-UI_Move_Start_X, 0, 1920-UI_Move_End_X, UI_Move_End_Y);//ǰ����
			UI_Draw_Int   (&UI_Graph7.Graphic[4], "211", UI_Graph_Add, 2, UI_Color_Green , 22, 3, 1180,173,100);		 		   //��������
			UI_Draw_Line  (&UI_Graph7.Graphic[5], "212", UI_Graph_Add, 2, UI_Color_Green, 20, 760, 160, 1160, 160);				 //��������ͼ��
			UI_Draw_Rectangle(&UI_Graph7.Graphic[6], "213", UI_Graph_Add, 2, UI_Color_White, 2,754 , 145, 1164, 175);			 //��������ͼ��
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			osDelay(30);
			
			//��̬UIԤ����  shoot 
			UI_Draw_Rectangle(&UI_Graph5.Graphic[0], "204", UI_Graph_Add, 2, UI_Color_White, 2,	161 , 758, 191,	788);			 	//Ħ����ͼ��
			UI_Draw_Line		 (&UI_Graph5.Graphic[1], "205", UI_Graph_Add, 2, UI_Color_Green, 22, 176, 784,  176, 763);		 	//Ħ����ͼ��
			UI_Draw_Rectangle(&UI_Graph5.Graphic[2], "206", UI_Graph_Add, 2, UI_Color_White, 2,	161 , 708, 191,	738);			 	//����ͼ��
			UI_Draw_Line		 (&UI_Graph5.Graphic[3], "207", UI_Graph_Add, 2, UI_Color_Green, 22, 176, 734,  176, 713);			//����ͼ��
			UI_Draw_Arc			 (&UI_Graph5.Graphic[4], "208", UI_Graph_Add, 2, UI_Color_Main, 300, 60, 5, 960,  740,100,50);	//�ܻ�����
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			UI_attack_flag=1;
			UI_fric_flag=1;
			UI_lock_flag=1;
			osDelay(30);
			
			//��̬UIԤ���� AUTO_AIM_MODE
			UI_Draw_String(&UI_String.String, "104", UI_Graph_Add, 1, UI_Color_Yellow,  20, 6, 3,  1840, 575, "AIM   "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			UI_Draw_String(&UI_String.String, "105", UI_Graph_Add, 1, UI_Color_Yellow,  20, 6, 3,  1835, 545, "MODE  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			//��̬UIԤ���� AUTO_AIM_MODE
			UI_Draw_String(&UI_String.String, "106", UI_Graph_Add, 1, UI_Color_White,  20, 6, 3,  1830, 685, "NORM  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			UI_Draw_String(&UI_String.String, "107", UI_Graph_Add, 1, UI_Color_Black,  20, 6, 3,  1725, 605, "ANTI ");
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			UI_Draw_String(&UI_String.String, "108", UI_Graph_Add, 1, UI_Color_Black,  20, 6, 3,  1720, 505, "SMALL "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			UI_Draw_String(&UI_String.String, "109", UI_Graph_Add, 1, UI_Color_Black,  20, 6, 3,  1845, 430, "BIG  ");
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			UI_AutoAim_Mode_last=0;
			
			//��̬UIԤ���� SPEED_MODE
			UI_Draw_String(&UI_String.String, "110", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  1680,830, "SPEED_"); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			UI_Draw_String(&UI_String.String, "111", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  1820,830, "MODE  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
			//��̬UIԤ���� SPEED_MODE
			UI_Draw_String(&UI_String.String, "112", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  1760,790, "SLOW  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			UI_Speed_Mode_last=0;
			osDelay(30);
		}
		
		//��̬UI���� �ܻ�����
		if(UI_Hurt_armor_id>=0&&UI_Hurt_armor_id<=3&&(UI_HP_deduction_reason==0||UI_HP_deduction_reason==5)&&UI_robot_last_HP-UI_robot_current_HP>=2)
		{
			UI_attack_direction_Counter=0;
			UI_attack_direction=(limit_360deg(((chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*360+45))/90+UI_Hurt_armor_id)%4;
			uint8_t UI_Operate=0;
			
			if(UI_attack_flag==0)
			{
				UI_Operate=UI_Graph_Add;
			}
			else if(UI_attack_flag==1)
			{
				UI_Operate=UI_Graph_Delete;
			}

			if(UI_attack_direction==0)				//ǰ
			{
				UI_Draw_Arc(&UI_Graph1.Graphic[0], "208", UI_Operate, 2, UI_Color_Main, 300, 60, 5, 960,  740,100,50);
			}
			else if(UI_attack_direction==1)		//��
			{
				UI_Draw_Arc(&UI_Graph1.Graphic[0], "208", UI_Operate, 2, UI_Color_Main, 210, 330, 5, 760,  540,50,100);
			}
			else if(UI_attack_direction==2)		//��
			{
				UI_Draw_Arc(&UI_Graph1.Graphic[0], "208", UI_Operate, 2, UI_Color_Main, 120, 240, 5, 960,  340,100,50);
			}
			else if(UI_attack_direction==3)		//��
			{
				UI_Draw_Arc(&UI_Graph1.Graphic[0], "208", UI_Operate, 2, UI_Color_Main, 30, 150, 5, 1160,  540,50,100);
			}
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			UI_attack_flag=1;
			osDelay(30);
		}
		else
		{
			if(UI_attack_flag==1)
			{
				UI_attack_direction_Counter++;
				if(UI_attack_direction_Counter==10)
				{
					UI_Draw_Arc(&UI_Graph1.Graphic[0], "208", UI_Graph_Delete, 2, UI_Color_Main, 30, 150, 5, 1160,  540,50,100);
					UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
					UI_attack_direction_Counter=0;
					UI_attack_flag=0;
					osDelay(30);
				}
			}
		}
			
		if(UI_fric_mode	!=	UI_fric_flag ) //��̬UI���� Ħ����
		{
			uint8_t UI_Operate=0;
			if(UI_fric_mode == 1) 
			{	
				UI_Operate=UI_Graph_Add;
			}
			else if(UI_fric_mode == 0) 
			{
				UI_Operate=UI_Graph_Delete;
			}
			UI_Draw_Line(&UI_Graph1.Graphic[0], "205", UI_Operate, 2, UI_Color_Green, 22, 176, 784,  176, 763);			 //Ħ����ͼ��
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			UI_fric_flag=!UI_fric_flag;
			osDelay(30);
		}
		
		if(UI_lock_mode	!=	UI_lock_flag ) //��̬UI���� Lock
		{
			uint8_t UI_Operate=0;
			if(UI_lock_mode == 1) 
			{
				UI_Operate=UI_Graph_Add;
			}
			else if(UI_lock_mode == 0) 
			{
				UI_Operate=UI_Graph_Delete;
			}
			UI_Draw_Line(&UI_Graph1.Graphic[0], "207", UI_Operate, 2, UI_Color_Green, 22, 176, 734,  176, 713);			 //����ͼ��
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			UI_lock_flag=!UI_lock_flag;
			osDelay(30);
		}
		
		if(UI_Speed_Mode!=UI_Speed_Mode_last)
		{
			if(UI_Speed_Mode == 0) 
			{
				UI_Draw_String(&UI_String.String, "112", UI_Graph_Change, 1, UI_Color_Green,  22, 6, 3,  1760,790, "SLOW  "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
			else if(UI_Speed_Mode == 1) 
			{
				UI_Draw_String(&UI_String.String, "112", UI_Graph_Change, 1, UI_Color_Yellow,  22, 6, 3,  1740,790, "NORMAL"); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
			else if(UI_Speed_Mode == 2)
			{
				UI_Draw_String(&UI_String.String, "112", UI_Graph_Change, 1, UI_Color_Orange,  22, 6, 3,  1760,790, "FAST  "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
		}
		
		if(UI_AutoAim_Mode_last!=UI_AutoAim_Mode)
		{
			switch(UI_AutoAim_Mode_last)
			{
				case 0:
					UI_Draw_String(&UI_String.String, "106", UI_Graph_Change, 1, UI_Color_Black,  20, 6, 3,  1830, 685, "NORM  "); 
					break;
				case 1:
					UI_Draw_String(&UI_String.String, "107", UI_Graph_Change, 1, UI_Color_Black,  20, 6, 3,  1725, 605, "ANTI ");
					break;
				case 2:
					UI_Draw_String(&UI_String.String, "108", UI_Graph_Change, 1, UI_Color_Black,  20, 6, 3,  1720, 505, "SMALL "); 
					break;
				case 3:
					UI_Draw_String(&UI_String.String, "109", UI_Graph_Change, 1, UI_Color_Black,  20, 6, 3,  1845, 430, "BIG  "); 
					break;
				default:
					break;
			}
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
					
			switch(UI_AutoAim_Mode)
			{
				case 0:
					UI_Draw_String(&UI_String.String, "106", UI_Graph_Change, 1, UI_Color_White,  20, 6, 3,  1830, 685, "NORM  "); 
					break;
				case 1:
					UI_Draw_String(&UI_String.String, "107", UI_Graph_Change, 1, UI_Color_White,  20, 6, 3,  1725, 605, "ANTI ");
					break;
				case 2:
					UI_Draw_String(&UI_String.String, "108", UI_Graph_Change, 1, UI_Color_White,  20, 6, 3,  1720, 505, "SMALL "); 
					break;
				case 3:
					UI_Draw_String(&UI_String.String, "109", UI_Graph_Change, 1, UI_Color_White,  20, 6, 3,  1845, 430, "BIG  "); 
					break;
				default:
					break;
			}
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(30);
		}
		
		if(UI_Update_Flag==1)
		{
			/* Pitch�ᵱǰ�Ƕ� */
			UI_Draw_Float(&UI_Graph7.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, UI_Gimbal_Pitch);
			/* Roll�ᵱǰ�Ƕ� */
			UI_Draw_Line (&UI_Graph7.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Green , 1 , 960-300.0f*arm_cos_f32(UI_Gimbal_Roll/180.0f*3.14159f) , 600-300*arm_sin_f32(UI_Gimbal_Roll/180.0f*3.14159f), 
										960+300*arm_cos_f32(UI_Gimbal_Roll/180.0f*3.14159f), 600+300*arm_sin_f32(UI_Gimbal_Roll/180.0f*3.14159f));
			
			UI_Draw_Line(&UI_Graph7.Graphic[2], "209", UI_Graph_Change, 2, UI_Color_White , 2 , UI_Move_Start_X, 0, UI_Move_End_X, UI_Move_End_Y);
			UI_Draw_Line(&UI_Graph7.Graphic[3], "210", UI_Graph_Change, 2, UI_Color_White , 2 , 1920-UI_Move_Start_X, 0, 1920-UI_Move_End_X, UI_Move_End_Y);
			/* ������������ */
			UI_Capacitance = Max(UI_Capacitance, 5);
			Capacitance_X  = 760.0f + 4.0f * UI_Capacitance;
			if(50 < UI_Capacitance && UI_Capacitance <= 100) 
			{
				UI_Draw_Int	(&UI_Graph7.Graphic[4], "211", UI_Graph_Change, 2, UI_Color_Green , 22, 3, 1180,173,UI_Capacitance);
				UI_Draw_Line(&UI_Graph7.Graphic[5], "212", UI_Graph_Change, 2, UI_Color_Green , 20,760 , 160, Capacitance_X, 160);
			}
			if(30 < UI_Capacitance && UI_Capacitance <=  60) 
			{
				UI_Draw_Int	(&UI_Graph7.Graphic[4], "211", UI_Graph_Change, 2, UI_Color_Yellow , 22, 3, 1180,173,UI_Capacitance);
				UI_Draw_Line(&UI_Graph7.Graphic[5], "212", UI_Graph_Change, 2, UI_Color_Yellow , 20,760 , 160, Capacitance_X, 160);
			}
			if(0  < UI_Capacitance && UI_Capacitance <=  30) 
			{
				UI_Draw_Int	(&UI_Graph7.Graphic[4], "211", UI_Graph_Change, 2, UI_Color_Orange , 22, 3, 1180,173,UI_Capacitance);
				UI_Draw_Line(&UI_Graph7.Graphic[5], "212", UI_Graph_Change, 2, UI_Color_Orange , 20,760 , 160, Capacitance_X, 160);
			}
			
			UI_Draw_Line(&UI_Graph7.Graphic[6], "212", UI_Graph_invalid, 2, UI_Color_Orange , 20,760 , 160, Capacitance_X, 160);
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			osDelay(30);
		}
		
	}
}

uint16_t this_time_rx_len = 0;
void USART6_IRQHandler_1(void)
{
		if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
        }
    }
}
