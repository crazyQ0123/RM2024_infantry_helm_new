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

#define ANGLE_TO_RAD    0.01745f

#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

#define YAW_MOUSE_SEN   0.01f
#define PITCH_MOUSE_SEN 0.01f

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
#define YAW_MOTOR_AUTO_AIM_PID_KP 35.0f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0f//0.17f
#define YAW_MOTOR_AUTO_AIM_PID_KD 320.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 1200.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

#define PITCH_MOTOR_SPEED_PID_KP 150.0f
#define PITCH_MOTOR_SPEED_PID_KI 0.3f
#define PITCH_MOTOR_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 1500.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 200.0f

#define PITCH_MOTOR_ANGLE_PID_KP 0.4f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 1.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 200.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

float pitch_pid_rate=0.25f;
#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.3f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.0f		//0.0002f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 0.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 3.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

#define PITCH_ANGLE_SET_MAX 35.0f
#define PITCH_ANGLE_SET_MIN -24.0f

#ifdef INFANTRY_HELM_NEW
#define GRAVITY_BALANCE(n)			(250)
#endif
#ifdef INFANTRY_HELM_OLD
#define GRAVITY_BALANCE(n)			(0.0018*(n)*(n)*(n) - 0.0649*(n)*(n) + 6.7329*(n) + 38.363)
#endif



gimbal_motor_t gimbal_LK[2];
uint8_t yaw_mode=0,yaw_mode_last=0;//0:speed,1:angle
uint8_t pitch_mode=0,pitch_mode_last=0;//0:speed,1:angle

float yaw_angle_err=0,pitch_angle_err=0;
float auto_aim_err_yaw=0,auto_aim_err_pitch=0;
int auto_aim_vx=0,auto_aim_vz=0;
float aim_adjust_yaw,aim_adjust_pitch;

uint8_t identify_flag=1;
fp32 balance_current = 250;
//kf_data_t pitch_kf;

extern ext_robot_status_t Game_Robot_Status;
extern fifo_s_t Referee_FIFO;
extern uint8_t chassis_follow_gimbal_changing;

uint8_t is_valid()
{
	return 	isnan(nuc_receive_data.aim_data_received.pitch)||isinf(nuc_receive_data.aim_data_received.pitch)||
					isnan(nuc_receive_data.aim_data_received.yaw)||isinf(nuc_receive_data.aim_data_received.yaw)||
					isnan(nuc_receive_data.aim_data_received.target_rate)||isinf(nuc_receive_data.aim_data_received.target_rate)||
					isnan(nuc_receive_data.aim_data_received.target_number)||isinf(nuc_receive_data.aim_data_received.target_number)||
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


void Gimbal_Motor_Init(void)
{
	const static fp32 yaw_motor_speed_pid[3] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD};
	const static fp32 yaw_motor_angle_pid[3] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD};
	const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};
	
	const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
	const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
	const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};
	
	for(uint8_t i=0;i<2;i++)
	{
		gimbal_LK[i].INS_speed=0;
		gimbal_LK[i].INS_speed_set=0;
		gimbal_LK[i].INS_angle=0;
		gimbal_LK[i].INS_angle_set=0;
		gimbal_LK[i].ENC_angle=0;
		gimbal_LK[i].ENC_angle_actual=0;
		gimbal_LK[i].ENC_angle_set=0;
		gimbal_LK[i].ENC_speed =0;
		gimbal_LK[i].give_current=0;
	}
	
	PID_init(&gimbal_LK[0].speed_pid,PID_POSITION,yaw_motor_speed_pid,YAW_MOTOR_SPEED_PID_MAX_OUT,YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_LK[0].angle_pid,PID_POSITION,yaw_motor_angle_pid,YAW_MOTOR_ANGLE_PID_MAX_OUT,YAW_MOTOR_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_LK[0].auto_aim_pid,PID_POSITION,yaw_motor_auto_aim_pid,YAW_MOTOR_AUTO_AIM_PID_MAX_OUT,YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
	
	PID_init(&gimbal_LK[1].speed_pid,PID_POSITION,pitch_motor_speed_pid,PITCH_MOTOR_SPEED_PID_MAX_OUT,PITCH_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_LK[1].angle_pid,PID_POSITION,pitch_motor_angle_pid,PITCH_MOTOR_ANGLE_PID_MAX_OUT,PITCH_MOTOR_ANGLE_PID_MAX_IOUT);	
	PID_init(&gimbal_LK[1].auto_aim_pid,PID_POSITION,pitch_motor_auto_aim_pid,PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT,PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);	
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
	gimbal_LK[1].ENC_angle=motor_measure_gimbal[1].ecd;
	gimbal_LK[1].ENC_speed=motor_measure_gimbal[1].speed_rpm;
	
	//ÖØÁ¦²¹³¥
	balance_current = GRAVITY_BALANCE(gimbal_LK[1].INS_angle);
}

void Yaw_Motor_Control(void)
{
	if((rc_ctrl.mouse.press_r||rc_ctrl.rc.ch[4]<-200)&&nuc_receive_data.aim_data_received.success==1 && !is_valid())
	{		
		yaw_angle_err=nuc_receive_data.aim_data_received.yaw-gimbal_LK[0].INS_angle;
		if(yaw_angle_err>180) yaw_angle_err-=360;
		else if(yaw_angle_err<-180) yaw_angle_err+=360;
		gimbal_LK[0].auto_aim_pid.Kp=YAW_MOTOR_AUTO_AIM_PID_KP;

		if(gimbal_LK[0].auto_aim_pid.Kp>20.0f) gimbal_LK[0].auto_aim_pid.Kp=20.0f;
		if(gimbal_LK[0].auto_aim_pid.Kp<0.2f) gimbal_LK[0].auto_aim_pid.Kp=0.2f;
		
		aim_adjust_yaw += -DATA_LIMIT(rc_ctrl.mouse.x,-500,500)/10.0f * 0.001f;
		aim_adjust_yaw = DATA_LIMIT(aim_adjust_yaw,-1.5f,1.5f);
		PID_calc(&gimbal_LK[0].auto_aim_pid,yaw_angle_err,-aim_adjust_yaw);
		gimbal_LK[0].INS_speed_set=-gimbal_LK[0].auto_aim_pid.out;
		gimbal_LK[0].INS_angle_set=nuc_receive_data.aim_data_received.yaw ;
		yaw_mode=yaw_mode_last=1;
	}
	else
	{
		yaw_mode_last=yaw_mode;
		aim_adjust_yaw = 0;
		if((rc_ctrl.rc.ch[0]>10||rc_ctrl.rc.ch[0]<-10)||(rc_ctrl.mouse.x>10||rc_ctrl.mouse.x<-10))
		{			
			yaw_mode=0;
		}
		else
		{
			yaw_mode=1;
		}
		
		if(chassis_follow_gimbal_changing==1)
		{
			yaw_mode=1;
			yaw_mode_last=yaw_mode;
		}

		if(yaw_mode==0)
		{
			if(rc_ctrl.mouse.x>1||rc_ctrl.mouse.x<-1)
			{	
				gimbal_LK[0].INS_speed_set=-(float)rc_ctrl.mouse.x*YAW_MOUSE_SEN*0.03f*RAD_TO_ANGLE+gimbal_LK[0].INS_speed_set*0.97f;
				PID_clear(&gimbal_LK[0].angle_pid);
			}
			else
			{
				gimbal_LK[0].INS_speed_set=-(float)rc_ctrl.rc.ch[0]/660.0f*6*0.1f*RAD_TO_ANGLE+gimbal_LK[0].INS_speed_set*0.9f;
			}
		}
		else if(yaw_mode==1&&yaw_mode_last==0)
		{ 
			gimbal_LK[0].INS_angle_set=gimbal_LK[0].INS_angle;
		}

		if(yaw_mode==1)
		{
			yaw_angle_err=gimbal_LK[0].INS_angle_set-gimbal_LK[0].INS_angle;
			if(yaw_angle_err>180) yaw_angle_err-=360;
			else if(yaw_angle_err<-180) yaw_angle_err+=360;
			
			PID_calc(&gimbal_LK[0].angle_pid,yaw_angle_err,0);
			gimbal_LK[0].INS_speed_set=-gimbal_LK[0].angle_pid.out;
		}
	}

	PID_calc(&gimbal_LK[0].speed_pid,gimbal_LK[0].INS_speed,gimbal_LK[0].INS_speed_set);
	gimbal_LK[0].give_current=gimbal_LK[0].speed_pid.out;
}


void Pitch_Motor_Control(void)
{
//    gimbal_LK[1].ENC_angle_actual = (float)(((uint16_t)gimbal_LK[1].ENC_angle + (8192 - 5077)) % 8192) / 8192.0f * 360.0f;
//    if(gimbal_LK[1].ENC_angle_actual > 180)
//    {
//        gimbal_LK[1].ENC_angle_actual -= 360;
//    }

   if((rc_ctrl.mouse.press_r||rc_ctrl.rc.ch[4]<-200)&&nuc_receive_data.aim_data_received.success==1 && !is_valid())
    {
        pitch_angle_err = nuc_receive_data.aim_data_received.pitch - gimbal_LK[1].INS_angle;
        gimbal_LK[1].auto_aim_pid.Kp = pitch_pid_rate / (log((fabs(pitch_angle_err)) + 1.1f));
        if(gimbal_LK[1].auto_aim_pid.Kp > 20.0f) gimbal_LK[1].auto_aim_pid.Kp = 20.0f;
        if(gimbal_LK[1].auto_aim_pid.Kp < 0.2f) gimbal_LK[1].auto_aim_pid.Kp = 0.2f;
			
				aim_adjust_pitch += -DATA_LIMIT(rc_ctrl.mouse.y,-500,500)/10.0f * 0.001f;
				aim_adjust_pitch = DATA_LIMIT(aim_adjust_pitch,-1.5f,1.5f);
        PID_calc(&gimbal_LK[1].auto_aim_pid, gimbal_LK[1].INS_angle, nuc_receive_data.aim_data_received.pitch + aim_adjust_pitch);
        gimbal_LK[1].INS_speed_set = gimbal_LK[1].auto_aim_pid.out;

        gimbal_LK[1].INS_angle_set = nuc_receive_data.aim_data_received.pitch;
        yaw_mode = yaw_mode_last = 1;
    }
    else
    {
				aim_adjust_pitch = 0;
        pitch_mode_last = pitch_mode;
        if((rc_ctrl.rc.ch[1] > 5 || rc_ctrl.rc.ch[1] < -5) || (rc_ctrl.mouse.y > 10 || rc_ctrl.mouse.y < -10))
        {
            pitch_mode = 0;
        }
        else
        {
            pitch_mode = 1;
        }


        if(pitch_mode == 0)
        {
            if(rc_ctrl.mouse.y != 0)
                gimbal_LK[1].INS_speed_set = -(float)rc_ctrl.mouse.y * PITCH_MOUSE_SEN * 0.25f + gimbal_LK[1].INS_speed_set * 0.75f;
            else
                gimbal_LK[1].INS_speed_set = -(float)rc_ctrl.rc.ch[1] / 660.0f * 2;
						gimbal_LK[1].INS_angle_set = gimbal_LK[1].INS_angle;
        }
        else if(pitch_mode == 1 && pitch_mode_last == 0)
        {
            gimbal_LK[1].INS_angle_set = gimbal_LK[1].INS_angle;
        }

        if(pitch_mode == 1)
        {
            PID_calc(&gimbal_LK[1].angle_pid, gimbal_LK[1].INS_angle, gimbal_LK[1].INS_angle_set);
            gimbal_LK[1].INS_speed_set = gimbal_LK[1].angle_pid.out;
        }
    }

//    if(gimbal_LK[1].INS_angle_set>PITCH_ANGLE_SET_MAX && gimbal_LK[1].INS_speed_set>0)
//		{
//			gimbal_LK[1].INS_angle_set=PITCH_ANGLE_SET_MAX;
//			PID_calc(&gimbal_LK[1].angle_pid, gimbal_LK[1].INS_angle, gimbal_LK[1].INS_angle_set);
//			gimbal_LK[1].INS_speed_set = gimbal_LK[1].angle_pid.out;
//		}
//		else if(gimbal_LK[1].INS_angle_set<PITCH_ANGLE_SET_MIN && gimbal_LK[1].INS_speed_set<0)
//		{
//			gimbal_LK[1].INS_angle_set=PITCH_ANGLE_SET_MIN;
//			PID_calc(&gimbal_LK[1].angle_pid, gimbal_LK[1].INS_angle, gimbal_LK[1].INS_angle_set);
//			gimbal_LK[1].INS_speed_set = gimbal_LK[1].angle_pid.out;
//		}
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
	
	HAL_UART_Transmit_DMA(&huart1,BUFF_niming,_cnt);
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
		float speed_set = 16000 * sin(phtic);
		sin_time += 0.002;
			
		
		PID_calc(&gimbal_LK[0].speed_pid, gimbal_LK[0].INS_speed, speed_set);
		gimbal_LK[0].give_current = gimbal_LK[0].speed_pid.out;
		CAN_cmd_LK_Boardcast((int16_t)speed_set, 0, 0, 0);
			
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
	//vTaskDelete(Gimbal_TASKHandle);
	while(1)
	{
		Gimbal_Motor_Data_Update();
//		KalmanFilterCalc(&pitch_kf,gimbal_LK[1].INS_angle);
//			if(rc_ctrl.rc.s[0] == RC_SW_DOWN)
//			{
//				CAN_Gimbal_CMD(0, 0, 0, 0);
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
			CAN_cmd_LK_Pitch(balance_current*0.7f);
			#endif
			gimbal_LK[0].INS_angle_set=gimbal_LK[0].INS_angle;
			gimbal_LK[1].INS_angle_set = gimbal_LK[1].INS_angle;
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