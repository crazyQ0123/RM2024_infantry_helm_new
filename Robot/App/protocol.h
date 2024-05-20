/**
  ******************************************************************************
  * @file    protocol.h
  * @author  Karolance Future
  * @version V1.6.1
  * @date    2024/03/18
  * @brief   依据裁判系统 串口协议附录 V1.6.1
  ******************************************************************************
  * @attention
	*
  ******************************************************************************
  */

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "stdint.h"

#define HEADER_SOF                  0xA5

#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE       2
#define REF_PROTOCOL_CRC16_SIZE     2

#define REF_HEADER_CRC_LEN          (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN    (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN        (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
	GAME_STATE_CMD_ID                 = 0x0001,  //比赛状态数据
	GAME_RESULT_CMD_ID                = 0x0002,  //比赛结果数据
	GAME_ROBOT_HP_CMD_ID              = 0x0003,  //机器人血量数据
	
	FIELD_EVENTS_CMD_ID               = 0x0101,  //场地事件数据
	SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,  //补给站动作标识
	REFEREE_WARNING_CMD_ID            = 0x0104,  //裁判警告信息
	DART_SHOOT_RELEVANT_DATA_CMD_ID   = 0x0105,  //飞镖发射相关数据
	
	ROBOT_STATE_CMD_ID                = 0x0201,  //比赛机器人状态
	POWER_HEAT_DATA_CMD_ID            = 0x0202,  //实时功率热量数据
	ROBOT_POS_CMD_ID                  = 0x0203,  //机器人位置
	BUFF_MUSK_CMD_ID                  = 0x0204,  //机器人增益
	AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,  //空中支援时间数据
	ROBOT_HURT_CMD_ID                 = 0x0206,  //伤害状态
	SHOOT_DATA_CMD_ID                 = 0x0207,  //实时射击信息
	BULLET_ALLOWANCE_CMD_ID           = 0x0208,  //允许发弹量
	ROBOT_RFID_STATE_CMD_ID           = 0x0209,  //机器人RFID状态
	
	ROBOT_INTERACTIVE_DATA_CMD_ID   	= 0x0301,  //机器人间通信
  CLIENT_MAP_INTERACTIVE_CMD_ID			= 0x0303,  //选手端小地图交互数据
  CLIENT_MAP_RECIEVE_REDAR_CMD_ID   = 0x0305,  //选手端小地图接收雷达数据
	CLIENT_MAP_RECIEVE_ROBOT_CMD_ID   = 0x0308,  //选手端小地图接收机器人数据
	
	IDCustomData,
}referee_cmd_id_e;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct __attribute__((packed))
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header_struct_t;

typedef struct __attribute__((packed))
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
