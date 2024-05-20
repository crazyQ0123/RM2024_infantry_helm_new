#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H
#include "pid.h"
#include "bsp_can.h"
#include "main.h"

typedef struct
{
	motor_measure_t M6020,M3508;
	pid_type_def M6020_angle_pid;
	pid_type_def M6020_speed_pid;
	pid_type_def M3508_speed_pid;
	lpf_data_t M6020_speed_lpf;
	float ecd_offset;
	float angle_set;
	float angle_err;
	float speed_set;
	
}helm_state_t;

typedef struct
{
	float vx,vy,wz;
}chassis_helm_t;


extern helm_state_t helm[];
extern chassis_helm_t chassis_helm;

fp32 limit_pi(fp32 in);
fp32 limit_4096(fp32 in);
void PID_init_s(pid_type_def *pid, uint8_t mode, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout);

void helm_pid_init();
void helm_pid_update();
void helm_current_send();
void helm_current_off();
void helm_solve();







#endif
