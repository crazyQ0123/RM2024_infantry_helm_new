#ifndef _SHOOT_TASK
#define _SHOOT_TASK

#include "main.h"
#include "Gimbal_Task.h"

//#define fric_15m 1555
//#define fric_18m 1510//1510
//#define fric_30m 1690//1690

#define SHOOT_ALLOW			(FRIC_PWM_L >= fric_speed*0.97f)

typedef struct
{
  fp32 speed;
  fp32 speed_set;
	fp32 angle;
  fp32 angle_set;
	fp32 ENC_angle;
  int16_t give_current;
	
	pid_type_def speed_pid;
	pid_type_def angle_pid;
} shoot_motor_t;

typedef struct
{
  fp32 speed;
  fp32 speed_set;
	int16_t give_pwm;
	
	pid_type_def pid;
} fric_motor_t;

extern shoot_motor_t shoot_m2006[1];
extern uint8_t fric_state;
extern uint8_t dial_mode;//mode:0 continue,1 single;
extern uint8_t  fric_actual_speed;
extern uint16_t fric_speed;
void Fric_PWR(uint8_t power);
void Shoot_Task(void const * argument);

#endif
