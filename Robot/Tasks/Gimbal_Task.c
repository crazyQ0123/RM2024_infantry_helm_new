#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "Chassis_Task.h"
#include "main.h"
#include "arm_math.h"
#include "usart.h"
#include "string.h"
#include "referee.h"
#include "bsp_usb.h"
#include "helm_ctrl.h"
#include "Usb_Task.h"

#define ANGLE_TO_RAD    0.01745f

#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

#define WHEEL_RPM_TO_WZ 1.0f/14.238f*2*PI*60/60/258.0f/2.0f/PI*360/60.0f
//rpm/14.238f*2*PI*0.06/60/0.2580f/2.0f/PI*360

#define YAW_MOUSE_SEN   0.008f
#define PITCH_MOUSE_SEN 0.008f
#define YAW_RC_SEN   0.0025f
#define PITCH_RC_SEN 0.001f

#ifdef INFANTRY_HELM_NEW
#define PITCH_ENC_ANGLE_MAX 8000
#define PITCH_ENC_ANGLE_MIN 450

#define YAW_MOTOR_SPEED_PID_KP 6.0f//182.63f
#define YAW_MOTOR_SPEED_PID_KI 0.005f//6.09f
#define YAW_MOTOR_SPEED_PID_KD 0.0f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 2000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define YAW_MOTOR_ANGLE_PID_KP 15.0f//27.11f
#define YAW_MOTOR_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ANGLE_PID_KD 10.0f//13.55f
#define YAW_MOTOR_ANGLE_PID_MAX_OUT 600.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

float yaw_pid_rate=0.5f;//0.3f
#define YAW_MOTOR_AUTO_AIM_PID_KP 25.0f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0f	 
#define YAW_MOTOR_AUTO_AIM_PID_KD 80.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 1200.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

#define YAW_MOTOR_BIG_ENERGY_PID_KP 25.0f//17.0f
#define YAW_MOTOR_BIG_ENERGY_PID_KI 0.0f	 
#define YAW_MOTOR_BIG_ENERGY_PID_KD 80.0f//160.0f
#define YAW_MOTOR_BIG_ENERGY_PID_MAX_OUT 1200.0f//600.0f
#define YAW_MOTOR_BIG_ENERGY_PID_MAX_IOUT 100.0f//500.0f

#define PITCH_MOTOR_SPEED_PID_KP 250.0f
#define PITCH_MOTOR_SPEED_PID_KI 0.5f
#define PITCH_MOTOR_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 1500.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 100.0f

#define PITCH_MOTOR_ANGLE_PID_KP 0.3f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 1.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 300.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 100.0f

float pitch_pid_rate=0.25f;
#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.5f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.0f		//0.0002f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 0.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 300.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

#define PITCH_MOTOR_BIG_ENERGY_PID_KP 0.5f//0.2f
#define PITCH_MOTOR_BIG_ENERGY_PID_KI 0.0f	 
#define PITCH_MOTOR_BIG_ENERGY_PID_KD 0.0f
#define PITCH_MOTOR_BIG_ENERGY_PID_MAX_OUT 300.0f//150.0f
#define PITCH_MOTOR_BIG_ENERGY_PID_MAX_IOUT 100.0f//50.0f

#define PITCH_ANGLE_SET_MAX 28.0f
#define PITCH_ANGLE_SET_MIN -24.0f

#define GRAVITY_BALANCE(n)			(250*arm_cos_f32((n)*ANGLE_TO_RAD))
#define PITCH_ANGLE_ZERO 318.0f
#endif

#ifdef INFANTRY_HELM_OLD
#define PITCH_ENC_ANGLE_MAX 8000
#define PITCH_ENC_ANGLE_MIN 450

#define YAW_MOTOR_SPEED_PID_KP 6.0f
#define YAW_MOTOR_SPEED_PID_KI 0.001f
#define YAW_MOTOR_SPEED_PID_KD 0.0f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 2000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define YAW_MOTOR_ANGLE_PID_KP 20.0f
#define YAW_MOTOR_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ANGLE_PID_KD 10.0f
#define YAW_MOTOR_ANGLE_PID_MAX_OUT 600.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

float yaw_pid_rate=0.5f;//0.3f
#define YAW_MOTOR_AUTO_AIM_PID_KP 20.0f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define YAW_MOTOR_AUTO_AIM_PID_KD 10.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 600.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

#define YAW_MOTOR_BIG_ENERGY_PID_KP 20.0f
#define YAW_MOTOR_BIG_ENERGY_PID_KI 0.0f	 //0.17f
#define YAW_MOTOR_BIG_ENERGY_PID_KD 10.0f
#define YAW_MOTOR_BIG_ENERGY_PID_MAX_OUT 600.0f
#define YAW_MOTOR_BIG_ENERGY_PID_MAX_IOUT 0.0f

#define PITCH_MOTOR_SPEED_PID_KP 300.0f	//150.0f
#define PITCH_MOTOR_SPEED_PID_KI 4.0f
#define PITCH_MOTOR_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 1500.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define PITCH_MOTOR_ANGLE_PID_KP 0.3f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 1.5f //1.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 200.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

float pitch_pid_rate=0.25f;
#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.4f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.0f		//0.0002f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 5.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 200.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

#define PITCH_MOTOR_BIG_ENERGY_PID_KP 0.3f
#define PITCH_MOTOR_BIG_ENERGY_PID_KI 0.0f	 //0.17f
#define PITCH_MOTOR_BIG_ENERGY_PID_KD 0.0f
#define PITCH_MOTOR_BIG_ENERGY_PID_MAX_OUT 200.0f
#define PITCH_MOTOR_BIG_ENERGY_PID_MAX_IOUT 50.0f

#define PITCH_ANGLE_SET_MAX 38.0f
#define PITCH_ANGLE_SET_MIN -23.0f

#define GRAVITY_BALANCE(n)			(0.0018*(n)*(n)*(n) - 0.0649*(n)*(n) + 6.7329*(n) + 38.363)
#define PITCH_ANGLE_ZERO 117.61f
#endif


gimbal_motor_t gimbal_LK[2];

float yaw_angle_err=0,pitch_angle_err=0;
float auto_aim_err_yaw=0,auto_aim_err_pitch=0;
int auto_aim_vx=0,auto_aim_vz=0;
float aim_adjust_yaw,aim_adjust_pitch;
float MouseX_angle=0,MouseY_angle=0;
float RcX_angle=0,RcY_angle=0;
float Yaw_feedforward=0.7f,Pitch_feedforward=0.03f;
uint8_t gimbal_control_flag=0;

uint8_t identify_flag=1;
fp32 balance_current = 250;
float Pitch_ang_min;

extern ext_robot_status_t Game_Robot_Status;
extern fifo_s_t Referee_FIFO;
extern uint8_t chassis_follow_gimbal_changing;

uint8_t is_valid()
{
	return 	isnan(nuc_receive_data.aim_data_received.is_fire)||isinf(nuc_receive_data.aim_data_received.is_fire)||
					isnan(nuc_receive_data.aim_data_received.yaw)||isinf(nuc_receive_data.aim_data_received.yaw)||
					isnan(nuc_receive_data.aim_data_received.pitch)||isinf(nuc_receive_data.aim_data_received.pitch)||
					isnan(nuc_receive_data.aim_data_received.distance)||isinf(nuc_receive_data.aim_data_received.distance)||
					isnan(nuc_receive_data.aim_data_received.success)||isinf(nuc_receive_data.aim_data_received.success);
}

void CAN_cmd_LK_Boardcast(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
	CAN_TxHeaderTypeDef  motor_tx_message={0};
	uint8_t              motor_can_send_data[8]={0};
	uint32_t send_mail_box=0;
	motor_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	motor_tx_message.IDE = CAN_ID_STD;
	motor_tx_message.RTR = CAN_RTR_DATA;
	motor_tx_message.DLC = 0x08;
	motor_can_send_data[0] = motor1;
	motor_can_send_data[1] = motor1 >> 8;
	motor_can_send_data[2] = motor2;
	motor_can_send_data[3] = motor2 >> 8;
	motor_can_send_data[4] = motor3;
	motor_can_send_data[5] = motor3 >> 8;
	motor_can_send_data[6] = motor4;
	motor_can_send_data[7] = motor4 >> 8;
	HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN_cmd_LK_Yaw(int16_t motor)
{
	CAN_TxHeaderTypeDef  motor_tx_message={0};
	uint8_t              motor_can_send_data[8]={0};
	uint32_t send_mail_box=0;
	motor_tx_message.StdId = CAN_LK_M1_ID;
	motor_tx_message.IDE = CAN_ID_STD;
	motor_tx_message.RTR = CAN_RTR_DATA;
	motor_tx_message.DLC = 0x08;
	motor_can_send_data[0] = 0xA1;
	motor_can_send_data[1] = 0x00;
	motor_can_send_data[2] = 0x00;
	motor_can_send_data[3] = 0x00;
	motor_can_send_data[4] = motor;
	motor_can_send_data[5] = motor >> 8;
	motor_can_send_data[6] = 0x00;
	motor_can_send_data[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN_cmd_LK_Pitch(int16_t motor)
{
	CAN_TxHeaderTypeDef  motor_tx_message={0};
	uint8_t              motor_can_send_data[8]={0};
	uint32_t send_mail_box=0;
	motor_tx_message.StdId = CAN_LK_M2_ID;
	motor_tx_message.IDE = CAN_ID_STD;
	motor_tx_message.RTR = CAN_RTR_DATA;
	motor_tx_message.DLC = 0x08;
	motor_can_send_data[0] = 0xA1;
	motor_can_send_data[1] = 0x00;
	motor_can_send_data[2] = 0x00;
	motor_can_send_data[3] = 0x00;
	motor_can_send_data[4] = motor;
	motor_can_send_data[5] = motor >> 8;
	motor_can_send_data[6] = 0x00;
	motor_can_send_data[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN_cmd_AMMO(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//-30000,+30000
{
	CAN_TxHeaderTypeDef  motor_tx_message={0};
	uint8_t              motor_can_send_data[8]={0};
	uint32_t send_mail_box=0;
	motor_tx_message.StdId = CAN_AMMO_ALL_ID;
	motor_tx_message.IDE = CAN_ID_STD;
	motor_tx_message.RTR = CAN_RTR_DATA;
	motor_tx_message.DLC = 0x08;
	motor_can_send_data[0] = motor1 >> 8;
	motor_can_send_data[1] = motor1;
	motor_can_send_data[2] = motor2 >> 8;
	motor_can_send_data[3] = motor2;
	motor_can_send_data[4] = motor3 >> 8;
	motor_can_send_data[5] = motor3;
	motor_can_send_data[6] = motor4 >> 8;
	motor_can_send_data[7] = motor4;
	HAL_CAN_AddTxMessage(&AMMO_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

fp32 limit_180deg(fp32 in)
{
	while(in < -180 || in > 180)
	{
		if (in < -180)
			in = in + 360;
		if (in > 180)
			in = in - 360;
	}
	return in;
}

void Gimbal_Motor_Init(void)
{
	const static fp32 yaw_motor_speed_pid[3] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD};
	const static fp32 yaw_motor_angle_pid[3] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD};
	const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};
	const static fp32 yaw_motor_big_energy_pid[3] = {YAW_MOTOR_BIG_ENERGY_PID_KP, YAW_MOTOR_BIG_ENERGY_PID_KI, YAW_MOTOR_BIG_ENERGY_PID_KD};
	
	const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
	const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
	const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};
	const static fp32 pitch_motor_big_energy_pid[3] = {PITCH_MOTOR_BIG_ENERGY_PID_KP, PITCH_MOTOR_BIG_ENERGY_PID_KI, PITCH_MOTOR_BIG_ENERGY_PID_KD};
	
	for(uint8_t i=0;i<2;i++)
	{
		gimbal_LK[i].INS_speed=0;
		gimbal_LK[i].INS_speed_set=0;
		gimbal_LK[i].INS_angle=0;
		gimbal_LK[i].INS_angle_set=0;
		gimbal_LK[i].ENC_angle=0;
		gimbal_LK[i].ENC_angle_actual=0;
		gimbal_LK[i].ENC_angle_actual_set=0;
		gimbal_LK[i].ENC_speed =0;
		gimbal_LK[i].give_current=0;
	}
	
	PID_init(&gimbal_LK[0].speed_pid,PID_POSITION,yaw_motor_speed_pid,YAW_MOTOR_SPEED_PID_MAX_OUT,YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_LK[0].angle_pid,PID_POSITION,yaw_motor_angle_pid,YAW_MOTOR_ANGLE_PID_MAX_OUT,YAW_MOTOR_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_LK[0].auto_aim_pid,PID_POSITION,yaw_motor_auto_aim_pid,YAW_MOTOR_AUTO_AIM_PID_MAX_OUT,YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
	PID_init(&gimbal_LK[0].big_energy_pid,PID_POSITION,yaw_motor_big_energy_pid,YAW_MOTOR_BIG_ENERGY_PID_MAX_OUT,YAW_MOTOR_BIG_ENERGY_PID_MAX_IOUT);
	
	PID_init(&gimbal_LK[1].speed_pid,PID_POSITION,pitch_motor_speed_pid,PITCH_MOTOR_SPEED_PID_MAX_OUT,PITCH_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_LK[1].angle_pid,PID_POSITION,pitch_motor_angle_pid,PITCH_MOTOR_ANGLE_PID_MAX_OUT,PITCH_MOTOR_ANGLE_PID_MAX_IOUT);	
	PID_init(&gimbal_LK[1].auto_aim_pid,PID_POSITION,pitch_motor_auto_aim_pid,PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT,PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);	
	PID_init(&gimbal_LK[1].big_energy_pid,PID_POSITION,pitch_motor_big_energy_pid,PITCH_MOTOR_BIG_ENERGY_PID_MAX_OUT,PITCH_MOTOR_BIG_ENERGY_PID_MAX_IOUT);
}

void Gimbal_Motor_Data_Update(void)
{
	//yaw
	//z2+y2
//	fp32 temp;
//	
//	arm_sqrt_f32(bmi088_real_data.gyro[2]*bmi088_real_data.gyro[2]+bmi088_real_data.gyro[0]*bmi088_real_data.gyro[0],&temp);	
//	if(bmi088_real_data.gyro[2]<0)
//	{
//		temp=-temp;
//	}
	
	//yaw
	gimbal_LK[0].INS_speed=bmi088_real_data.gyro[2]*RAD_TO_ANGLE;
	gimbal_LK[0].INS_angle=INS_angle_deg[0];
	gimbal_LK[0].ENC_angle=motor_measure_gimbal[0].ecd;	
	gimbal_LK[0].ENC_speed=motor_measure_gimbal[0].speed_rpm;
	
	//pitch
	gimbal_LK[1].INS_speed=bmi088_real_data.gyro[1];
	gimbal_LK[1].INS_angle=INS_angle_deg[2];
	gimbal_LK[1].ENC_angle=motor_measure_gimbal[1].ecd/((fp32)GIMBAL_ECD_RANGE)*360.0f;
	gimbal_LK[1].ENC_speed=motor_measure_gimbal[1].speed_rpm;
	gimbal_LK[1].ENC_angle_actual = gimbal_LK[1].ENC_angle - PITCH_ANGLE_ZERO;
	gimbal_LK[1].chassis_pitch = gimbal_LK[1].INS_angle-gimbal_LK[1].ENC_angle_actual;
	
	//Pitch限位
	#ifdef INFANTRY_HELM_NEW
	float ang_err1=ABS(chassis_control.chassis_follow_gimbal_err);
	float ang_err2=ang_err1;
	if(ang_err1>PI/2.0f)ang_err1-=PI/2.0f;
	//舵轮
	if(ang_err1>PI/4.0f-0.22f&&ang_err1<PI/4.0f+0.22f)Pitch_ang_min=-23.0f;
	//装甲板
	else if(ang_err2>PI/2.0f-0.35f&&ang_err2<PI/2.0f+0.35f)Pitch_ang_min=-21.5f;
	//灯条
	else if(ang_err2>2.5f&&ang_err2<PI)Pitch_ang_min=11.028f*ang_err2*ang_err2 - 69.514f*ang_err2 + 83.483;
	//其他
	else Pitch_ang_min=-25.5f;
	#endif
	
	#ifdef INFANTRY_HELM_OLD
	Pitch_ang_min=PITCH_ANGLE_SET_MIN;
	#endif
	
	//重力补偿
	balance_current = GRAVITY_BALANCE(gimbal_LK[1].ENC_angle_actual);
}

void Yaw_Motor_Control(void)
{
	if((rc_ctrl.mouse.press_r||rc_ctrl.rc.ch[4]>200)&&nuc_receive_data.aim_data_received.success==1 && !is_valid())
	{		
		yaw_angle_err=nuc_receive_data.aim_data_received.yaw-gimbal_LK[0].INS_angle;
		if(yaw_angle_err>180) yaw_angle_err-=360;
		else if(yaw_angle_err<-180) yaw_angle_err+=360;
		gimbal_LK[0].auto_aim_pid.Kp=YAW_MOTOR_AUTO_AIM_PID_KP;

//		if(gimbal_LK[0].auto_aim_pid.Kp>20.0f) gimbal_LK[0].auto_aim_pid.Kp=20.0f;
//		if(gimbal_LK[0].auto_aim_pid.Kp<0.2f) gimbal_LK[0].auto_aim_pid.Kp=0.2f;
		
		aim_adjust_yaw += -DATA_LIMIT(rc_ctrl.mouse.x,-500,500)/10.0f * 0.001f;
		aim_adjust_yaw = DATA_LIMIT(aim_adjust_yaw,-1.5f,1.5f);
		if(Autoaim_Mode==2 || Autoaim_Mode==3)
		{
			PID_calc(&gimbal_LK[0].big_energy_pid,yaw_angle_err,-aim_adjust_yaw);
			gimbal_LK[0].INS_speed_set=-gimbal_LK[0].big_energy_pid.out;
			gimbal_LK[0].INS_angle_set=nuc_receive_data.aim_data_received.yaw ;
		}
		else
		{
			PID_calc(&gimbal_LK[0].auto_aim_pid,yaw_angle_err,-aim_adjust_yaw);
			gimbal_LK[0].INS_speed_set=-gimbal_LK[0].auto_aim_pid.out;
			gimbal_LK[0].INS_angle_set=nuc_receive_data.aim_data_received.yaw ;
		}
	}
	else
	{
		if(rc_ctrl.mouse.x!=0)
		{	
			MouseX_angle=rc_ctrl.mouse.x;
			gimbal_LK[0].INS_angle_set-=MouseX_angle*YAW_MOUSE_SEN;
			gimbal_LK[0].INS_angle_set=limit_180deg(gimbal_LK[0].INS_angle_set);
			gimbal_control_flag=1;
		}
		else if(rc_ctrl.rc.ch[0]!=0)
		{
			RcX_angle=rc_ctrl.rc.ch[0];
			gimbal_LK[0].INS_angle_set-=RcX_angle*YAW_RC_SEN;
			gimbal_LK[0].INS_angle_set=limit_180deg(gimbal_LK[0].INS_angle_set);
			gimbal_control_flag=1;
		}
		else 
		{
			MouseX_angle=0;
			RcX_angle=0;
		}
		yaw_angle_err=gimbal_LK[0].INS_angle_set-gimbal_LK[0].INS_angle;
		if(yaw_angle_err>180) yaw_angle_err-=360;
		else if(yaw_angle_err<-180) yaw_angle_err+=360;
		PID_calc(&gimbal_LK[0].angle_pid,yaw_angle_err,0);
		gimbal_LK[0].INS_speed_set=-gimbal_LK[0].angle_pid.out-MouseX_angle*Yaw_feedforward;
	}

	PID_calc(&gimbal_LK[0].speed_pid,gimbal_LK[0].INS_speed,gimbal_LK[0].INS_speed_set-chassis_helm.wz*WHEEL_RPM_TO_WZ);
	gimbal_LK[0].give_current=gimbal_LK[0].speed_pid.out;
}


void Pitch_Motor_Control(void)
{
		#ifdef INFANTRY_HELM_NEW
		if(KEYB_ACROSS_TUNNEL)
		{
			gimbal_LK[1].INS_angle_set=-25.6f;
			PID_calc(&gimbal_LK[1].angle_pid, gimbal_LK[1].INS_angle, RAMP_CTRL(gimbal_LK[1].INS_angle,gimbal_LK[1].INS_angle_set,0.4f));
			gimbal_LK[1].INS_speed_set = gimbal_LK[1].angle_pid.out;
			PID_calc(&gimbal_LK[1].speed_pid, gimbal_LK[1].INS_speed, gimbal_LK[1].INS_speed_set);
			gimbal_LK[1].give_current = gimbal_LK[1].speed_pid.out + balance_current;
			
			return;
		}
		#endif
   if((rc_ctrl.mouse.press_r||rc_ctrl.rc.ch[4]>200)&&nuc_receive_data.aim_data_received.success==1 && !is_valid())
   {
        pitch_angle_err = nuc_receive_data.aim_data_received.pitch - gimbal_LK[1].INS_angle;
//        gimbal_LK[1].auto_aim_pid.Kp = pitch_pid_rate / (log((fabs(pitch_angle_err)) + 1.1f));
//        if(gimbal_LK[1].auto_aim_pid.Kp > 20.0f) gimbal_LK[1].auto_aim_pid.Kp = 20.0f;
//        if(gimbal_LK[1].auto_aim_pid.Kp < 0.2f) gimbal_LK[1].auto_aim_pid.Kp = 0.2f;
			
				aim_adjust_pitch += -DATA_LIMIT(rc_ctrl.mouse.y,-500,500)/10.0f * 0.001f;
				aim_adjust_pitch = DATA_LIMIT(aim_adjust_pitch,-1.5f,1.5f);
				if(Autoaim_Mode==2 || Autoaim_Mode==3)
				{
					PID_calc(&gimbal_LK[1].big_energy_pid, gimbal_LK[1].INS_angle, nuc_receive_data.aim_data_received.pitch + aim_adjust_pitch);
					gimbal_LK[1].INS_speed_set = gimbal_LK[1].big_energy_pid.out;
					gimbal_LK[1].INS_angle_set = nuc_receive_data.aim_data_received.pitch;
				}
				else 
				{
					PID_calc(&gimbal_LK[1].auto_aim_pid, gimbal_LK[1].INS_angle, nuc_receive_data.aim_data_received.pitch + aim_adjust_pitch);
					gimbal_LK[1].INS_speed_set = gimbal_LK[1].auto_aim_pid.out;

					gimbal_LK[1].INS_angle_set = nuc_receive_data.aim_data_received.pitch;
				}
    }
    else
    {
				aim_adjust_pitch = 0;
				if(rc_ctrl.mouse.y != 0)
				{
					MouseY_angle=rc_ctrl.mouse.y;
					gimbal_LK[1].INS_angle_set -= MouseY_angle*PITCH_MOUSE_SEN;
				}
				else if(rc_ctrl.rc.ch[1]!=0)
				{
					RcY_angle=(float)rc_ctrl.rc.ch[1];
					gimbal_LK[1].INS_angle_set -= RcY_angle*PITCH_RC_SEN;
				}
				else 
				{
					MouseY_angle=0;
					RcY_angle=0;
				}
				PID_calc(&gimbal_LK[1].angle_pid, gimbal_LK[1].INS_angle, gimbal_LK[1].INS_angle_set);
				gimbal_LK[1].INS_speed_set = gimbal_LK[1].angle_pid.out-MouseY_angle*Pitch_feedforward;
    }
		
    if(gimbal_LK[1].ENC_angle_actual>PITCH_ANGLE_SET_MAX && gimbal_LK[1].INS_speed_set>-0.1f)
		{
			gimbal_LK[1].ENC_angle_actual_set=PITCH_ANGLE_SET_MAX;
			gimbal_LK[1].INS_angle_set=gimbal_LK[1].INS_angle;
			PID_calc(&gimbal_LK[1].angle_pid, gimbal_LK[1].ENC_angle_actual, gimbal_LK[1].ENC_angle_actual_set);
			gimbal_LK[1].INS_speed_set = gimbal_LK[1].angle_pid.out;
		}
		else if(gimbal_LK[1].ENC_angle_actual<Pitch_ang_min && gimbal_LK[1].INS_speed_set<0.1f)
		{
			gimbal_LK[1].ENC_angle_actual_set=Pitch_ang_min;
			gimbal_LK[1].INS_angle_set=gimbal_LK[1].INS_angle;
			PID_calc(&gimbal_LK[1].angle_pid, gimbal_LK[1].ENC_angle_actual, gimbal_LK[1].ENC_angle_actual_set);
			gimbal_LK[1].INS_speed_set = gimbal_LK[1].angle_pid.out;
		}
		PID_calc(&gimbal_LK[1].speed_pid, gimbal_LK[1].INS_speed, gimbal_LK[1].INS_speed_set);
    gimbal_LK[1].give_current = gimbal_LK[1].speed_pid.out + balance_current;

}
/****************************************************system identify***********************************************************/
uint8_t BUFF_niming[30];
void niming_sent_data(float A,float B,float time)
{
	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt=0;
	BUFF_niming[_cnt++]=0xAA;
	BUFF_niming[_cnt++]=0xFF;
	BUFF_niming[_cnt++]=0XF1;
	BUFF_niming[_cnt++]=12;
	BUFF_niming[_cnt++]=Get_BYTE0(A);
	BUFF_niming[_cnt++]=Get_BYTE1(A);
	BUFF_niming[_cnt++]=Get_BYTE2(A);
	BUFF_niming[_cnt++]=Get_BYTE3(A);	
	BUFF_niming[_cnt++]=Get_BYTE0(B);
	BUFF_niming[_cnt++]=Get_BYTE1(B);
	BUFF_niming[_cnt++]=Get_BYTE2(B);
	BUFF_niming[_cnt++]=Get_BYTE3(B);
	BUFF_niming[_cnt++]=Get_BYTE0(time);
	BUFF_niming[_cnt++]=Get_BYTE1(time);
	BUFF_niming[_cnt++]=Get_BYTE2(time);
	BUFF_niming[_cnt++]=Get_BYTE3(time);
	for(i=0;i<BUFF_niming[3]+4;i++) 
	{
		sumcheck+=BUFF_niming[i];
		addcheck+=sumcheck;
	}
	BUFF_niming[_cnt++]=sumcheck;	
	BUFF_niming[_cnt++]=addcheck;	
	
	HAL_UART_Transmit(&huart1,BUFF_niming,_cnt,0xffff);
//	HAL_UART_Transmit_DMA(&huart1,BUFF_niming,_cnt);
}

void Yaw_Motor_Identification()
{
	static float sin_time = 0; 
	static float last_sin_time = 0;
	static uint8_t i_sin = 0;
	static float phtic = 0;
	static float f[64] = {1.000000, 1.499250, 2.000000, 2.500000, 3.003003, 3.496503, 4.000000, 4.504505, 
		5.000000, 5.494505, 5.988024, 6.493506, 6.993007, 7.518797, 8.000000, 8.474576, 9.009009, 9.523810, 
		10.000000, 10.526316, 10.989011, 11.494253, 12.048193, 12.500000, 12.987013, 13.513514, 14.084507, 
		14.492754, 14.925373, 15.384615, 15.873016, 16.393443, 16.949153, 17.543860, 17.857143, 18.518519, 
		18.867925, 19.607843, 20.000000, 20.408163, 20.833333, 21.276596, 22.222222, 23.809524, 26.315789, 
	  27.777778, 30.303030, 32.258065, 34.482759, 35.714286, 38.461538, 40.000000, 50.0000, 58.8235, 71.4286, 
		76.9231, 90.9091, 100.0000, 111.1111, 125.0000, 200.0000, 250.0000, 333.3333, 500.0000};

	if(identify_flag)
	{
		phtic += f[i_sin] * 0.002;
		float speed_set = 160 * sin(phtic);//16000 * sin(phtic);
		sin_time += 0.002;
			
		
		PID_calc(&gimbal_LK[0].speed_pid, gimbal_LK[0].INS_speed, speed_set);
		gimbal_LK[0].give_current = gimbal_LK[0].speed_pid.out;
		CAN_cmd_LK_Yaw((int16_t)speed_set);
		CAN_cmd_LK_Pitch(0);
			
		niming_sent_data(speed_set,gimbal_LK[0].INS_speed,sin_time);

		if(last_sin_time + 3.1415926*4/(float)f[i_sin] <= sin_time)//????????????
		{
			i_sin++;
			last_sin_time = sin_time;
		}
		if(i_sin > 63)
		{
			identify_flag = 0;
		}
	}

}


void Gimbal_Task(void const * argument)
{
	Gimbal_Motor_Init();
//	Kalman_Init(&pitch_kf,0.001,1);
	rc_ctrl.rc.s[1]=RC_SW_DOWN;
	vTaskDelay(200);
	while(1)
	{
		Gimbal_Motor_Data_Update();
//		KalmanFilterCalc(&pitch_kf,gimbal_LK[1].INS_angle);
//			if(rc_ctrl.rc.s[0] == RC_SW_DOWN)
//			{
//				CAN_cmd_LK_Yaw(0);
//				CAN_cmd_LK_Pitch(0);
//				identify_flag = 1;
//			  niming_sent_data(1,2,3);
//			}
//			else
//			{
//				Yaw_Motor_Identification();
//			}

		Yaw_Motor_Control();
		Pitch_Motor_Control();
		Pitch_Motor_protect();
		
    
		if(rc_ctrl.rc.s[1]==RC_SW_DOWN)
		{
			#ifndef GIMBAL_MOTOR_SINGLE_CONTROL
			CAN_cmd_LK_Boardcast(0, 0, 0, 0);
			#endif
			#ifdef GIMBAL_MOTOR_SINGLE_CONTROL
			CAN_cmd_LK_Yaw(0);
			vTaskDelay(1);
				#ifdef INFANTRY_HELM_OLD
				CAN_cmd_LK_Pitch(0);
				#endif
				#ifdef INFANTRY_HELM_NEW
				CAN_cmd_LK_Pitch(balance_current*0.7f);
				#endif
			#endif
			gimbal_LK[0].INS_angle_set = gimbal_LK[0].INS_angle;
			gimbal_LK[1].INS_angle_set = gimbal_LK[1].INS_angle;
//			PID_clear(&gimbal_LK[0].speed_pid);
//			PID_clear(&gimbal_LK[1].speed_pid);
			vTaskDelay(1);
			CAN_cmd_AMMO(0, 0, 0, 0);
		}
		else
		{
			#ifndef GIMBAL_MOTOR_SINGLE_CONTROL
			CAN_cmd_LK_Boardcast(gimbal_LK[0].give_current, gimbal_LK[1].give_current, 0, 0);
			#endif
			#ifdef GIMBAL_MOTOR_SINGLE_CONTROL
			CAN_cmd_LK_Yaw(gimbal_LK[0].give_current);
			vTaskDelay(1);
			CAN_cmd_LK_Pitch(gimbal_LK[1].give_current);
			#endif
			
			vTaskDelay(1);
			CAN_cmd_AMMO(shoot_m2006[0].give_current,0,0,0);
		}

		vTaskDelay(1);
	}
}


uint16_t power_cnt;
void Pitch_Motor_protect()
{
	if(fabs((float)gimbal_LK[1].give_current)>=1500)power_cnt++;
	else if(power_cnt>0)power_cnt--;
	if(power_cnt>=5000)gimbal_LK[1].INS_angle_set=gimbal_LK[1].INS_angle;
}