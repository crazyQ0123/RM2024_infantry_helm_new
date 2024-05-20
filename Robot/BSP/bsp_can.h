#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

typedef struct
{
    uint16_t ecd;
    fp32 speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void can_filter_init(void);
extern motor_measure_t motor_measure_gimbal[3];

extern uint8_t rx_data2[8];
extern uint8_t rx_data[8];
extern int16_t rx_chassis_data1[8];
extern int16_t rx_chassis_data2[8];
extern float rx_chassis_debug[4];

#endif
