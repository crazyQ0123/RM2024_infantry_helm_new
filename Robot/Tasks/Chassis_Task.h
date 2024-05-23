#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"
#include "cmsis_os.h"

//INFANTRY_HELM_NEW
#ifdef INFANTRY_HELM_NEW
#define HELM_OFFSET_0 4764
#define HELM_OFFSET_1 4718
#define HELM_OFFSET_2 7461
#define HELM_OFFSET_3 2067
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 30560
#endif

//INFANTRY_HELM_OLD
#ifdef INFANTRY_HELM_OLD
#define HELM_OFFSET_0 7485
#define HELM_OFFSET_1 120
#define HELM_OFFSET_2 7460
#define HELM_OFFSET_3 4779
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 24898
#endif

//定义 弧度 转换到 角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204

#define CAN_helm_M1_ID 0x205
#define CAN_helm_M2_ID 0x206
#define CAN_helm_M3_ID 0x207
#define CAN_helm_M4_ID 0x208

#define CHASSIS_CAN hcan1

#define MOTOR_DISTANCE_TO_CENTER 0.2f
#define CHASSIS_WZ_SET_SCALE 0.0f
#define GIMBAL_ECD_RANGE 65536

typedef enum
{
	CHASSIS_FRONT	= 0,
	CHASSIS_BACK	= 1,
	
}chassis_direction_e;


typedef struct
{
  fp32 vx;
	fp32 vy;
	fp32 v;
	fp32 psi;
	fp32 wz;
	int16_t RC_wz_last;
	pid_type_def chassis_psi;
  fp32 chassis_follow_gimbal_zero;	
	fp32 chassis_follow_gimbal_err;	
	uint8_t chassis_mode;
	uint8_t chassis_side;
	uint8_t chassis_direction;
	uint8_t debug_mode;
}chassis_control_t;

extern osThreadId Gimbal_TASKHandle;
extern osThreadId Chassis_TASKHandle;
extern chassis_control_t chassis_control;
void Chassis_Task(void const * argument);
extern uint8_t autoaim_mode;//2:normal,3:small energy,4:big energy
extern uint8_t autoaim_armor;//0x10:auto,0x20:big,0x30:small
extern uint8_t if_predict;
extern uint8_t fric_state;    //摩擦轮是否开启
extern uint8_t chassis_follow_gimbal_changing;
extern uint8_t Speed_Mode;


#endif
