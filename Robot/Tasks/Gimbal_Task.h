#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "struct_typedef.h"
#include "pid.h"

#define CAN_AMMO_ALL_ID 0x1FF
#define CAN_2006_M1_ID 0x205

#define CAN_GIMBAL_ALL_ID 0x280
#define CAN_LK_M1_ID 0x141
#define CAN_LK_M2_ID 0x142

#define GIMBAL_CAN hcan1
#define AMMO_CAN hcan1

#define Get_BYTE0(dwTemp)       (*(char *)(&dwTemp))	
#define Get_BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 
#define Get_BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define Get_BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define GIMBAL_MOTOR_SINGLE_CONTROL

typedef struct
{
  fp32 INS_speed;
  fp32 INS_speed_set;
	fp32 INS_angle;
  fp32 INS_angle_set;
	fp32 ENC_angle;
	fp32 ENC_speed;
	fp32 ENC_angle_actual;
	fp32 ENC_angle_actual_set;
	fp32 chassis_pitch;
  int16_t give_current;
	
	pid_type_def speed_pid;
	pid_type_def angle_pid;
	pid_type_def auto_aim_pid;
	pid_type_def big_energy_pid;
} gimbal_motor_t;

extern gimbal_motor_t gimbal_LK[2];

void Gimbal_Task(void const * argument);
void Pitch_Motor_protect();

#endif
