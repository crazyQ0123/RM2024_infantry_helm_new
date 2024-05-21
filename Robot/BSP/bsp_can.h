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

typedef struct
{
		uint8_t temperate;
		uint16_t voltage;
		uint8_t error_State;
}motor_error_t;

extern void can_filter_init(void);
extern motor_measure_t motor_measure_gimbal[3];
extern motor_error_t motor_error_gimbal[2];

extern uint8_t rx_data[8];

#endif
