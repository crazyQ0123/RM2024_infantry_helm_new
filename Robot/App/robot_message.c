#include "robot_message.h"
#include "Usb_Task.h"
#include "My_Def.h"
#include "INS_Task.h"
#include "referee.h"

//small low high
nuc_receive_data_t	nuc_receive_data;
nuc_transmit_data_t 	nuc_transmit_data;
cmd_id_queue_t cmd_id_queue;

/*************************** SEND ********************************/

void data_update(uint8_t cmd_id)
{
    switch(cmd_id)
		{
        case GIMBAL_AND_CONFIG_SEND_ID:
            /*  Update the value of variables here START*/
						nuc_transmit_data.robot_gimbal_data_send.roll=		ROLL;
            nuc_transmit_data.robot_gimbal_data_send.pitch=		PITCH;
            nuc_transmit_data.robot_gimbal_data_send.yaw	=		YAW;
				
				
						if(Game_Robot_Status.robot_id<100)
						{
							switch(Autoaim_Mode)
							{
								case 0:
									nuc_transmit_data.robot_gimbal_data_send.mode	=1;
									break;
								case 1:
									nuc_transmit_data.robot_gimbal_data_send.mode	=2;
									break;
								case 2:
									nuc_transmit_data.robot_gimbal_data_send.mode	=4;
									break;
								default:
									break;
							}
						}
						else 
						{
							switch(Autoaim_Mode)
							{
								case 0:
									nuc_transmit_data.robot_gimbal_data_send.mode	=0;
									break;
								case 1:
									nuc_transmit_data.robot_gimbal_data_send.mode	=3;
									break;
								case 2:
									nuc_transmit_data.robot_gimbal_data_send.mode	=5;
									break;
								default:
									break;
							}
						}
				//r 0
				//b   1
				//r xfu 2
				//b xfu  3
				//r dfu 4
				//b dfu 5
            /*  Update the value of variables here END*/
            break;
        default:
            break;
    }
}



void send_data_to_nuc(uint8_t cmd_id)
{
    uint16_t len;
//    uint8_t t[10][4];
//		usb_cdc_data.usb_cdc_send_buf[0]=0xAA;
//    
    switch (cmd_id)
    {
        case GIMBAL_AND_CONFIG_SEND_ID:
							len=16;
							usb_cdc_data.usb_cdc_send_buf[0]=0xff;
							usb_cdc_data.usb_cdc_send_buf[1]= nuc_transmit_data.robot_gimbal_data_send.mode;
							memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[2],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.roll,4);
							memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[6],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.pitch,4);
							memcpy((uint8_t*)&usb_cdc_data.usb_cdc_send_buf[10],(uint8_t*)&nuc_transmit_data.robot_gimbal_data_send.yaw,4);
							usb_cdc_data.usb_cdc_send_buf[14]=0x00;
							usb_cdc_data.usb_cdc_send_buf[15]=0x0d;
						break;
        default:
            break;
			}
    usb_data_send(usb_cdc_data.usb_cdc_send_buf,len);
		//send
		// uint8_t 0xff	
		// uint8_t  mode
		// fp32    roll	
		// fp32   pitch	
		// fp32    yaw	
		// uint8_t 0x00	
		// uint8_t 0x0d	
}

//AA 0C 00 13 00 01 02 03 04 05 06 07 08 09 0A XX

/*************************** SEND ********************************/

/*************************** RECV ********************************/
void Nuc_data_unpacked()
{
    if(usb_cdc_data.usb_cdc_rx_flag==1)
		{
			if(usb_cdc_data.usb_cdc_rx_buf[0]==0xff&&usb_cdc_data.usb_cdc_rx_buf[15]==0x0d)
			{
        nuc_receive_data.aim_data_received.is_fire=usb_cdc_data.usb_cdc_rx_buf[1];
				memcpy(&nuc_receive_data.aim_data_received.pitch,&usb_cdc_data.usb_cdc_rx_buf[2],4);
				memcpy(&nuc_receive_data.aim_data_received.yaw,&usb_cdc_data.usb_cdc_rx_buf[6],4);
				memcpy(&nuc_receive_data.aim_data_received.distance,&usb_cdc_data.usb_cdc_rx_buf[10],4);
				nuc_receive_data.aim_data_received.target_number=usb_cdc_data.usb_cdc_rx_buf[14];
				if(nuc_receive_data.aim_data_received.distance>0) 
					nuc_receive_data.aim_data_received.success=1;
				else  
					nuc_receive_data.aim_data_received.success=0;
			}
			usb_cdc_data.usb_cdc_rx_flag=0;
    }
}

/*************************** RECV ********************************/


void float_to_u8(float* float_in,uint8_t* u8_out)
{
    uint8_t farray[4];
    *(float*)farray=*float_in;
    u8_out[3]=farray[3];
    u8_out[2]=farray[2];
    u8_out[1]=farray[1];
    u8_out[0]=farray[0];
}

void u8_to_float(uint8_t* datain,float* dataout)
{
    *dataout=*(float *)datain;
}

void cmd_id_task_create(uint8_t cmd_id,uint16_t freq)
{
	cmd_id_queue.cmd_id_queue[cmd_id_queue.total_num]=cmd_id;
	cmd_id_queue.cmd_id_frq[cmd_id_queue.total_num]=1000/(freq%1000);
  cmd_id_queue.total_num+=1;
}

void cmd_id_queue_handle()
{
		for(int i=0;i<cmd_id_queue.total_num;i++){
				if(cmd_id_queue.now_pos%cmd_id_queue.cmd_id_frq[i]==0){
					data_update(cmd_id_queue.cmd_id_queue[i]);
					send_data_to_nuc(cmd_id_queue.cmd_id_queue[i]);
				}
		}
    cmd_id_queue.now_pos++;
    if(cmd_id_queue.now_pos>=1000){
        cmd_id_queue.now_pos=0;
    }
}

void cmd_id_init()
{
    cmd_id_queue.total_num=0;
    cmd_id_queue.now_pos=0;
}


uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len) 
{
	uint8_t crc = 0xff;
	while (len--) {
		crc = CRC08_Table[crc ^ *ptr++];
	}
	return crc;
}
