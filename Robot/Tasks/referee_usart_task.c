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
#define UI_UPDATE_DELAY 80
//#define Robot_ID_Current Robot_ID_Blue_Infantry3
//#define Robot_ID_Current Robot_ID_Red_Infantry3
/* Private variables ---------------------------------------------------------*/
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* 裁判系统接收数据队列 */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;

/* 动态UI数据变量 */
#ifndef	Robot_ID_Current
uint8_t Robot_ID_Current;
#endif

#ifndef GIMBAL_ECD_RANGE
#define GIMBAL_ECD_RANGE 65536
#endif
uint8_t UI_Update_Flag = 0;    //是否开启自瞄标志位
float   UI_Kalman_Speed = 0;    //卡尔曼预测速度
float   UI_Chassis_Pitch = 0.0f; //云台Pitch轴角度
float		UI_Gimbal_Pitch =0.0f;	//云台Pitch轴角度
float   UI_Gimbal_Roll = 0.0f; 	//云台Roll轴角度
float   UI_Gimbal_Yaw   = 0.0f; //云台Yaw轴角度
float   UI_Chassis_Follow_Gimbal_Angle=0.0f;//底盘跟随云台角度
float   UI_distance=0.0f;
int16_t UI_attack_direction=0;			//受击反馈方向
uint8_t UI_attack_flag=0;						//受击反馈图形是否已绘制
uint16_t UI_attack_direction_Counter=0;	//受击反馈图形删除延时
uint8_t UI_HP_deduction_reason=0;
uint8_t UI_Hurt_armor_id=0;
uint16_t UI_robot_current_HP=0;					//机器人当前血量
uint16_t UI_robot_last_HP=0;						//机器人上次血量
uint8_t UI_AutoAim_Mode=0;
uint8_t UI_AutoAim_Mode_last=0;
uint8_t UI_Speed_Mode=0;
uint8_t UI_Speed_Mode_last=0;
uint8_t UI_Capacitance  = 10;   //电容剩余容量
uint8_t UI_fric_mode   = 0;     //摩擦轮是否开启
uint8_t UI_fric_flag		=0;			//摩擦轮图形是否已绘制
uint8_t UI_lock_mode   = 0;     //自瞄锁定是否开启
uint8_t UI_lock_flag		=0;			//自瞄锁定图形是否已绘制
char UI_lock_target_number[6]  ={0};
char UI_lock_target_HP[6]			 ={0};
uint16_t UI_target_HP_graphic=0;
uint16_t UI_chassis_follow_angle=0;
uint16_t  UI_chassis_positionX=0;
uint16_t  UI_chassis_positionY=0;

uint16_t UI_Move_Start_X=611;
uint16_t UI_Move_End_X=850;
uint16_t UI_Move_End_Y=400;
float UI_Move_K=0;

/* 中央标尺高度变量 */
uint16_t y01 = 480;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;
/*小坦克位置补偿*/
uint16_t tank_x =600;
uint16_t tank_y =200;
/*血条中心位置*/
uint16_t HP_x = 600;
uint16_t HP_y = 750;
extern uint8_t if_predict;

uint8_t statics=0;

/**
  * @brief  转角限制到0~360
  * @param  输入转角
  * @retval 输出转角
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
	/* 动态UI控制变量 */
	float    Capacitance_X;
	UI_Update_Flag=1;
	/* 裁判系统初始化 */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		/* 解析裁判系统数据 */
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		#ifndef	Robot_ID_Current
		Robot_ID_Current = Game_Robot_Status.robot_id;
		#endif
		UI_Chassis_Pitch=gimbal_LK[1].ENC_angle_actual;
		UI_Gimbal_Pitch=gimbal_LK[1].INS_angle;
		UI_Gimbal_Roll =INS_angle_deg[1];
		UI_distance=nuc_receive_data.aim_data_received.distance;
		UI_fric_mode	=	fric_state;
		UI_lock_mode  =	nuc_receive_data.aim_data_received.success;
		itoa(nuc_receive_data.aim_data_received.target_number%9,UI_lock_target_number,10);
//		UI_chassis_follow_angle=limit_360deg(45-(chassis_control.chassis_follow_gimbal_zero-motor_measure_gimbal[0].ecd)/((fp32)GIMBAL_ECD_RANGE)*360);
//		UI_chassis_positionX=-arm_sin_f32(UI_chassis_follow_angle)*20;
//		UI_chassis_positionY=arm_cos_f32(UI_chassis_follow_angle)*20;
		if(Robot_ID_Current<50)
		{
				switch(nuc_receive_data.aim_data_received.target_number%9)
				{
					case 1:
						itoa(Game_Robot_HP.blue_1_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.blue_1_robot_HP;
						break;
					case 2:
						itoa(Game_Robot_HP.blue_2_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.blue_2_robot_HP;
						break;
					case 3:
						itoa(Game_Robot_HP.blue_3_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.blue_3_robot_HP;
						break;
					case 4:
						itoa(Game_Robot_HP.blue_4_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.blue_4_robot_HP;
						break;
					case 5:
						itoa(Game_Robot_HP.blue_5_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.blue_5_robot_HP;
						break;
					case 6:
						itoa(Game_Robot_HP.blue_7_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.blue_7_robot_HP;
						break;
					case 7:
						itoa(Game_Robot_HP.blue_outpost_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.blue_outpost_HP;
						break;
					case 8:
						itoa(Game_Robot_HP.blue_base_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.blue_base_HP;
						break;
					default:
						itoa(0,UI_lock_target_HP,10);
						UI_target_HP_graphic=0;
						break;
				}
		}
		else
		{
			switch(nuc_receive_data.aim_data_received.target_number%9)
			{
					case 1:
						itoa(Game_Robot_HP.red_1_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.red_1_robot_HP;
						break;
					case 2:
						itoa(Game_Robot_HP.red_2_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.red_2_robot_HP;
						break;
					case 3:
						itoa(Game_Robot_HP.red_3_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.red_3_robot_HP;
						break;
					case 4:
						itoa(Game_Robot_HP.red_4_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.red_4_robot_HP;
						break;
					case 5:
						itoa(Game_Robot_HP.red_5_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.red_5_robot_HP;
						break;
					case 6:
						itoa(Game_Robot_HP.red_7_robot_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.red_7_robot_HP;
						break;
					case 7:
						itoa(Game_Robot_HP.red_outpost_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.red_outpost_HP;
						break;
					case 8:
						itoa(Game_Robot_HP.red_base_HP,UI_lock_target_HP,10);
						UI_target_HP_graphic=Game_Robot_HP.red_base_HP;
						break;
					default:
						itoa(0,UI_lock_target_HP,10);
						UI_target_HP_graphic=0;
						break;
			}
		}
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
		if(UI_Chassis_Pitch>3.0f)
			UI_Chassis_Pitch=3.0f;
		else if(UI_Chassis_Pitch<-25.0f)
			UI_Chassis_Pitch=-25.0f;
		UI_Move_Start_X=0.0037f*pow(UI_Chassis_Pitch,3.0f)+0.0368f*pow(UI_Chassis_Pitch,2.0f)-11.648f*UI_Chassis_Pitch+1326.2f;
		UI_Move_K=-6E-06f*pow(UI_Chassis_Pitch,3.0f)-0.0003f*pow(UI_Chassis_Pitch,2.0f)+0.0046f*UI_Chassis_Pitch-1.2952f;
		UI_Move_End_X=UI_Move_Start_X+UI_Move_End_Y/UI_Move_K;
		/* UI更新 */
		if(KEYB_FLASH_UI)
		{
			//静态UI预绘制 中央标尺1
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 4,  840,   y01,  920,   y01); //第一行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 4,  950,   y01,  970,   y01); //第一行十字横
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 4, 1000,   y01, 1080,   y01); //第一行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 4,  960,y01-10,  960,y01+10); //第一行十字竖
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //第二行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //第二行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  989,   y02,  1049,  y02); //第二行左横线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			
//			//静态UI预绘制 中央标尺2
//			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //第三行左横线
//			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //第三行中心点
//			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //第三行右横线
//			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //第四行左横线
//			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //第四行中心点
//			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //第四行右横线
//			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //第四行下竖线
//			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);		
//			osDelay(UI_UPDATE_DELAY);
			
			//静态UI预绘制 小陀螺预警线 AUTO_AIM_MODE
			UI_Draw_Line(&UI_Graph7.Graphic[0], "015", UI_Graph_Add, 0, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[1], "016", UI_Graph_Add, 0, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[2], "017", UI_Graph_Add, 0, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[3], "018", UI_Graph_Add, 0, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Draw_Line(&UI_Graph7.Graphic[4], "019", UI_Graph_Add, 0, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_Draw_Arc	(&UI_Graph7.Graphic[5], "020", UI_Graph_Add, 0, UI_Color_Yellow, 150, 30, 3, 1880,  550,80,80);		//自瞄模式圆弧
			UI_Draw_Arc	(&UI_Graph7.Graphic[6], "021", UI_Graph_Add, 0, UI_Color_Yellow, 150, 30, 3, 1880,  550,180,180); //自瞄模式圆弧
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
		
			//静态UI预绘制 字符串1
			UI_Draw_String(&UI_String.String, "101", UI_Graph_Add, 1, UI_Color_Green,  26, 6, 3,  50, 785, "Fric  "); //摩擦轮是否开启
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			
			//静态UI预绘制 字符串2
			UI_Draw_String(&UI_String.String, "102", UI_Graph_Add, 1, UI_Color_Green,  26, 6, 3,  50, 735, "Lock  "); //自瞄是否锁定
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			
			//静态UI预绘制 字符串2
			UI_Draw_String(&UI_String.String, "103", UI_Graph_Add, 1, UI_Color_Green,  26, 6, 3,  665, 172, "Cap   "); //超级电容
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			
			//动态UI预绘制 图形 and Move_Predict
			UI_Draw_Float (&UI_Graph7.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pitch轴角度
			UI_Draw_Line  (&UI_Graph7.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Green , 2 , 960-300, 600, 960+300, 600);//Roll轴角度
			UI_Draw_Line  (&UI_Graph7.Graphic[2], "209", UI_Graph_Add, 2, UI_Color_White , 2 , UI_Move_Start_X, 0, UI_Move_End_X, UI_Move_End_Y);//前进线
			UI_Draw_Line  (&UI_Graph7.Graphic[3], "210", UI_Graph_Add, 2, UI_Color_White , 2 , 1920-UI_Move_Start_X, 0, 1920-UI_Move_End_X, UI_Move_End_Y);//前进线
			UI_Draw_Int   (&UI_Graph7.Graphic[4], "211", UI_Graph_Add, 2, UI_Color_Green , 22, 3, 1180,173,100);		 		   //电容容量5
			UI_Draw_Line  (&UI_Graph7.Graphic[5], "212", UI_Graph_Add, 2, UI_Color_Green, 20, 760, 160, 1160, 160);				 //电容容量图形
			UI_Draw_Rectangle(&UI_Graph7.Graphic[6], "213", UI_Graph_Add, 2, UI_Color_White, 2,754 , 145, 1164, 175);			 //电容容量图形
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			
			//动态UI预绘制  shoot and distance
			UI_Draw_Rectangle(&UI_Graph7.Graphic[0], "203", UI_Graph_Add, 2, UI_Color_White,  2,	161 , 758, 191,	788);			 	//摩擦轮图形
			UI_Draw_Line		 (&UI_Graph7.Graphic[1], "204", UI_Graph_Add, 2, UI_Color_Green, 22, 176, 784,  176, 763);		 	//摩擦轮图形
			UI_Draw_Rectangle(&UI_Graph7.Graphic[2], "205", UI_Graph_Add, 2, UI_Color_White,  2,	161 , 708, 191,	738);			 	//自瞄图形
			UI_Draw_Line		 (&UI_Graph7.Graphic[3], "206", UI_Graph_Add, 2, UI_Color_Green, 22, 176, 734,  176, 713);			//自瞄图形
			UI_Draw_Rectangle(&UI_Graph7.Graphic[4], "207", UI_Graph_Add, 2, UI_Color_White, 2,  681, 743, 1230, 275);	 //自瞄框
			UI_Draw_Arc			 (&UI_Graph7.Graphic[5], "208", UI_Graph_Add, 2, UI_Color_Orange, 300, 60, 5, 960,  740,100,50);	//受击反馈
			UI_Draw_Float		 (&UI_Graph7.Graphic[6], "214", UI_Graph_Add, 2, UI_Color_White, 18, 4, 3, 920, 50,0.000f);	//自瞄距离
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			UI_attack_flag=1;
			UI_fric_flag=1;
			UI_lock_flag=1;
			osDelay(UI_UPDATE_DELAY);
			
//			//动态UI预绘制 底盘位置
//			UI_Draw_Line(&UI_Graph2.Graphic[0], "220", UI_Graph_Add, 2, UI_Color_Yellow, 2,  100+UI_chassis_positionX,   650+UI_chassis_positionY,  100-UI_chassis_positionX,   650-UI_chassis_positionY); //第三行左横线
//			UI_Draw_Line(&UI_Graph2.Graphic[1], "221", UI_Graph_Add, 2, UI_Color_Yellow, 2,  100-UI_chassis_positionY,   650+UI_chassis_positionX,  100+UI_chassis_positionY,   650-UI_chassis_positionX); //第三行中心点
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);		
//			osDelay(UI_UPDATE_DELAY);
			//动态UI预绘制 底盘位置
			UI_Draw_Circle(UI_Graph1.Graphic,"220",UI_Graph_Add,2,UI_Color_White,8,960,735,25);
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			
			//静态UI预绘制 AUTO_AIM_MODE
			UI_Draw_String(&UI_String.String, "104", UI_Graph_Add, 1, UI_Color_Yellow,  20, 6, 3,  1840, 575, "AIM   "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			UI_Draw_String(&UI_String.String, "105", UI_Graph_Add, 1, UI_Color_Yellow,  20, 6, 3,  1835, 545, "MODE  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			//动态UI预绘制 AUTO_AIM_MODE
			UI_Draw_String(&UI_String.String, "106", UI_Graph_Add, 1, UI_Color_White,  20, 6, 3,  1830, 685, "AUTO  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
//			UI_Draw_String(&UI_String.String, "107", UI_Graph_Add, 1, UI_Color_Black,  20, 6, 3,  1725, 605, "ANTI ");
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
//			osDelay(UI_UPDATE_DELAY);
			UI_Draw_String(&UI_String.String, "108", UI_Graph_Add, 1, UI_Color_Black,  20, 6, 3,  1720, 505, "SMALL "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			UI_Draw_String(&UI_String.String, "109", UI_Graph_Add, 1, UI_Color_Black,  20, 6, 3,  1845, 430, "BIG  ");
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			UI_AutoAim_Mode_last=0;
			
			//静态UI预绘制 SPEED_MODE
			UI_Draw_String(&UI_String.String, "110", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  1680,830, "SPEED_"); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			UI_Draw_String(&UI_String.String, "111", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  1820,830, "MODE  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			//动态UI预绘制 SPEED_MODE
			UI_Draw_String(&UI_String.String, "112", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  1760,790, "SLOW  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			UI_Speed_Mode_last=0;
			osDelay(UI_UPDATE_DELAY);
			
			//静态UI预绘制 aim_target
			UI_Draw_String(&UI_String.String, "113", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  900,885, "AIM:  "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			UI_Draw_String(&UI_String.String, "114", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  900,835, "HP:   "); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
			//动态UI预绘制 aim_target
			UI_Draw_String(&UI_String.String, "115", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  1000,885, "000000"); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
//			UI_Draw_String(&UI_String.String, "116", UI_Graph_Add, 1, UI_Color_Green,  22, 6, 3,  1000,835, "000000"); 
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
//			osDelay(UI_UPDATE_DELAY);
			UI_Draw_Line(&UI_Graph1.Graphic[0], "116", UI_Graph_Add, 1, UI_Color_Green, 22, 960, 825, 960+UI_target_HP_graphic, 825);
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
		}
		
		//动态UI更新 受击反馈
		if(UI_Hurt_armor_id>=0&&UI_Hurt_armor_id<=3&&(UI_HP_deduction_reason==0||UI_HP_deduction_reason==5)&&UI_robot_last_HP-UI_robot_current_HP>=2)
		{
			UI_attack_direction_Counter=0;
			UI_attack_direction=360-chassis_control.chassis_follow_gimbal_err/PI*180-UI_Hurt_armor_id*90;
			uint8_t UI_Operate=0;
			
			if(UI_attack_flag==0)
			{
				UI_Operate=UI_Graph_Add;
			}
			else if(UI_attack_flag==1)
			{
				UI_Operate=UI_Graph_Delete;
			}
			
			UI_Draw_Arc(&UI_Graph1.Graphic[0], "208", UI_Operate, 2, UI_Color_Orange, limit_360deg(UI_attack_direction-30), limit_360deg(UI_attack_direction+30), 5, 960,  540,200,200);
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			UI_attack_flag=1;
			osDelay(UI_UPDATE_DELAY);
		}
		else
		{
			if(UI_attack_flag==1)
			{
				UI_attack_direction_Counter++;
				if(UI_attack_direction_Counter==5)
				{
					UI_Draw_Arc(&UI_Graph1.Graphic[0], "208", UI_Graph_Delete, 2, UI_Color_Orange, 30, 150, 5, 1160,  540,50,100);
					UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
					UI_attack_direction_Counter=0;
					UI_attack_flag=0;
					osDelay(UI_UPDATE_DELAY);
				}
			}
		}
			
		if(UI_fric_mode	!=	UI_fric_flag ) //动态UI更新 摩擦轮
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
			UI_Draw_Line(&UI_Graph1.Graphic[0], "204", UI_Operate, 2, UI_Color_Green, 22, 176, 784,  176, 763);			 //摩擦轮图形
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			UI_fric_flag=!UI_fric_flag;
			osDelay(UI_UPDATE_DELAY);
		}
		
		if(UI_lock_mode	!=	UI_lock_flag ) //动态UI更新 Lock
		{
			uint8_t UI_Operate=0;
			if(UI_lock_mode == 1) 
			{
				UI_Operate=UI_Graph_Add;
				UI_Draw_Rectangle(&UI_Graph2.Graphic[1], "207", UI_Graph_Change, 2, UI_Color_Purple, 2,  681, 743, 1230, 275);	 //自瞄框
				UI_Draw_String(&UI_String.String, "115", UI_Operate, 1, UI_Color_Green,  22, 6, 3,  1000,885, UI_lock_target_number); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
				osDelay(UI_UPDATE_DELAY);
//				UI_Draw_String(&UI_String.String, "116", UI_Operate, 1, UI_Color_Green,  22, 6, 3,  1000,835, UI_lock_target_HP); 
//				UI_PushUp_String(&UI_String, Robot_ID_Current);
//				osDelay(UI_UPDATE_DELAY);
				UI_Draw_Line(&UI_Graph1.Graphic[0], "116", UI_Graph_Change, 1, UI_Color_Green, 22, 960, 825, 960+UI_target_HP_graphic, 825);
				UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
				osDelay(UI_UPDATE_DELAY);
			}
			else if(UI_lock_mode == 0) 
			{
				UI_Operate=UI_Graph_Delete;
				UI_Draw_Rectangle(&UI_Graph2.Graphic[1], "207", UI_Graph_Change, 2, UI_Color_White, 2,  681, 743, 1230, 275);	 //自瞄框
				UI_Draw_String(&UI_String.String, "115", UI_Operate, 1, UI_Color_Green,  22, 6, 3,  1000,885, "000000"); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
				osDelay(UI_UPDATE_DELAY);
//				UI_Draw_String(&UI_String.String, "116", UI_Operate, 1, UI_Color_Green,  22, 6, 3,  1000,835, "000000"); 
//				UI_PushUp_String(&UI_String, Robot_ID_Current);
//				osDelay(UI_UPDATE_DELAY);
				UI_Draw_Line(&UI_Graph1.Graphic[0], "116", UI_Graph_Change, 1, UI_Color_Green, 22, 960, 825, 960+UI_target_HP_graphic, 825);
				UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
				osDelay(UI_UPDATE_DELAY);
			}
			UI_Draw_Line(&UI_Graph2.Graphic[0], "206", UI_Operate, 2, UI_Color_Green, 22, 176, 734,  176, 713);			 //自瞄图形
			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
			UI_lock_flag=!UI_lock_flag;
			osDelay(UI_UPDATE_DELAY);
		}
		if(UI_lock_flag == 1)
		{
			UI_Draw_String(&UI_String.String, "115", UI_Graph_Change, 1, UI_Color_Green,  22, 6, 3,  1000,885, UI_lock_target_number); 
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
//			UI_Draw_String(&UI_String.String, "116", UI_Graph_Change, 1, UI_Color_Green,  22, 6, 3,  1000,835, UI_lock_target_HP); 
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
//			osDelay(UI_UPDATE_DELAY);
			UI_Draw_Line(&UI_Graph1.Graphic[0], "116", UI_Graph_Change, 1, UI_Color_Green, 22, 960, 825, 960+UI_target_HP_graphic, 825);
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
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
			osDelay(UI_UPDATE_DELAY);
		}
		
		if(UI_AutoAim_Mode_last!=UI_AutoAim_Mode)
		{
			switch(UI_AutoAim_Mode_last)
			{
				case 0:
					UI_Draw_String(&UI_String.String, "106", UI_Graph_Change, 1, UI_Color_Black,  20, 6, 3,  1830, 685, "AUTO  "); 
					break;
//				case 1:
//					UI_Draw_String(&UI_String.String, "107", UI_Graph_Change, 1, UI_Color_Black,  20, 6, 3,  1725, 605, "ANTI ");
//					break;
				case 1:
					UI_Draw_String(&UI_String.String, "108", UI_Graph_Change, 1, UI_Color_Black,  20, 6, 3,  1720, 505, "SMALL "); 
					break;
				case 2:
					UI_Draw_String(&UI_String.String, "109", UI_Graph_Change, 1, UI_Color_Black,  20, 6, 3,  1845, 430, "BIG  "); 
					break;
				default:
					break;
			}
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
					
			switch(UI_AutoAim_Mode)
			{
				case 0:
					UI_Draw_String(&UI_String.String, "106", UI_Graph_Change, 1, UI_Color_White,  20, 6, 3,  1830, 685, "AUTO  "); 
					break;
//				case 1:
//					UI_Draw_String(&UI_String.String, "107", UI_Graph_Change, 1, UI_Color_White,  20, 6, 3,  1725, 605, "ANTI ");
//					break;
				case 1:
					UI_Draw_String(&UI_String.String, "108", UI_Graph_Change, 1, UI_Color_White,  20, 6, 3,  1720, 505, "SMALL "); 
					break;
				case 2:
					UI_Draw_String(&UI_String.String, "109", UI_Graph_Change, 1, UI_Color_White,  20, 6, 3,  1845, 430, "BIG  "); 
					break;
				default:
					break;
			}
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
		}
		
		if(fabs(gimbal_LK[0].ENC_speed)>20&&KEY_SHIFT)
		{
			UI_Draw_Circle(UI_Graph1.Graphic,"220",UI_Graph_Change,2,UI_Color_Main,8,960,735,25);
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
		}
		else 
		{
			UI_Draw_Circle(UI_Graph1.Graphic,"220",UI_Graph_Change,2,UI_Color_White,8,960,735,25);
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
		}
		
		if(UI_Update_Flag==1)
		{
//			UI_Draw_Line(&UI_Graph2.Graphic[0], "220", UI_Graph_Change, 2, UI_Color_Yellow, 2,  100+UI_chassis_positionX,   650+UI_chassis_positionY,  100-UI_chassis_positionX,   650-UI_chassis_positionY); //第三行左横线
//			UI_Draw_Line(&UI_Graph2.Graphic[1], "221", UI_Graph_Change, 2, UI_Color_Yellow, 2,  100-UI_chassis_positionY,   650+UI_chassis_positionX,  100+UI_chassis_positionY,   650-UI_chassis_positionX); //第三行中心点
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);		
//			osDelay(UI_UPDATE_DELAY);
			
			/* Pitch轴当前角度 */
			UI_Draw_Float(&UI_Graph7.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, UI_Gimbal_Pitch);
			/* Roll轴当前角度 */
			UI_Draw_Line (&UI_Graph7.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Green , 1 , 960-300.0f*arm_cos_f32(UI_Gimbal_Roll/180.0f*3.14159f) , 600-300*arm_sin_f32(UI_Gimbal_Roll/180.0f*3.14159f), 
										960+300*arm_cos_f32(UI_Gimbal_Roll/180.0f*3.14159f), 600+300*arm_sin_f32(UI_Gimbal_Roll/180.0f*3.14159f));
			
			UI_Draw_Line(&UI_Graph7.Graphic[2], "209", UI_Graph_Change, 2, UI_Color_White , 3 , UI_Move_Start_X, 0, UI_Move_End_X, UI_Move_End_Y);
			UI_Draw_Line(&UI_Graph7.Graphic[3], "210", UI_Graph_Change, 2, UI_Color_White , 3 , 1920-UI_Move_Start_X, 0, 1920-UI_Move_End_X, UI_Move_End_Y);
			/* 超级电容容量 */
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
			UI_Draw_Float	(&UI_Graph7.Graphic[6], "214", UI_Graph_Change, 2, UI_Color_White, 18, 4, 3, 920, 50,UI_distance);	//自瞄距离
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			osDelay(UI_UPDATE_DELAY);
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
