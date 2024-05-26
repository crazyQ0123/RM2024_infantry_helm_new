#include "robot_message.h"
#include "Usb_Task.h"
#include "My_Def.h"
#include "INS_Task.h"

//small low high
nuc_receive_data_t	nuc_receive_data;
nuc_transmit_data_t 	nuc_transmit_data;
cmd_id_queue_t cmd_id_queue;

/*************************** SEND ********************************/

void data_update(uint8_t cmd_id)
{
    switch(cmd_id)
		{
//        case IMU_SEND_ID:
//            /*  Update the value of variables here START*/
//            nuc_transmit_data.imu_data_send.x_gyro=GYROX;
//            nuc_transmit_data.imu_data_send.y_gyro=GYROY;
//            nuc_transmit_data.imu_data_send.z_gyro=GYROZ;
//            nuc_transmit_data.imu_data_send.x_accel=ACCELX;
//            nuc_transmit_data.imu_data_send.y_accel=ACCELY;
//            nuc_transmit_data.imu_data_send.z_accel=ACCELZ;
//            nuc_transmit_data.imu_data_send.X=QUATX;
//            nuc_transmit_data.imu_data_send.Y=QUATY;
//            nuc_transmit_data.imu_data_send.Z=QUATZ;
//            nuc_transmit_data.imu_data_send.W=QUATW;
//            /*  Update the value of variables here END*/
//            break;
//        case TYRE_SPEED_SEND_ID:
//            /*  Update the value of variables here START*/

//            /*  Update the value of variables here END*/
//            break;
//        case REFEREE_SEND_ID:
//            /*  Update the 	value of variables here START*/
//            nuc_transmit_data.referee_data_send.data_length=11;
//            for(int i=0;i<11;i++){
//                nuc_transmit_data.referee_data_send.referee_send_data[i]=i;
//            }
//            /*  Update the value of variables here END*/
//            break;
        case GIMBAL_AND_CONFIG_SEND_ID:
            /*  Update the value of variables here START*/
            nuc_transmit_data.robot_gimbal_data_send.camera_id=0;
						nuc_transmit_data.robot_gimbal_data_send.is_pressing=rc_ctrl.mouse.press_r;
            nuc_transmit_data.robot_gimbal_data_send.pitch=PITCH;
            nuc_transmit_data.robot_gimbal_data_send.yaw=YAW;
            nuc_transmit_data.robot_gimbal_data_send.roll=ROLL;
            nuc_transmit_data.robot_gimbal_data_send.mode=Autoaim_Mode/2;
						nuc_transmit_data.robot_gimbal_data_send.mode_config[0]=Autoaim_Mode%2;
            /*  Update the value of variables here END*/
            break;
        default:
            break;
    }
}



void send_data_to_nuc(uint8_t cmd_id)
{
    uint16_t len;
    uint8_t t[10][4];
	usb_cdc_data.usb_cdc_send_buf[0]=0xAA;
    
    switch (cmd_id)
    {
//        case IMU_SEND_ID:
//            len=sizeof(nuc_transmit_data.imu_data_send)+4;
//            float_to_u8(&nuc_transmit_data.imu_data_send.x_gyro,t[0]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.y_gyro,t[1]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.z_gyro,t[2]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.x_accel,t[3]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.y_accel,t[4]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.z_accel,t[5]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.X,t[6]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.Y,t[7]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.Z,t[8]);
//            float_to_u8(&nuc_transmit_data.imu_data_send.W,t[9]);
//            for(int i=0;i<10;i++){
//                for(int j=0;j<4;j++){
//                    usb_cdc_data.usb_cdc_send_buf[4+i*4+j]=t[i][j];
//                }
//            }
//            break;
//        case TYRE_SPEED_SEND_ID:
//            len=sizeof(nuc_transmit_data.tyre_speed_data_send)+4;
//            float_to_u8(&nuc_transmit_data.tyre_speed_data_send.x_pos,t[0]);
//            float_to_u8(&nuc_transmit_data.tyre_speed_data_send.y_pos,t[1]);
//            float_to_u8(&nuc_transmit_data.tyre_speed_data_send.robot_yaw_angle,t[2]);
//            float_to_u8(&nuc_transmit_data.tyre_speed_data_send.forward_speed,t[3]);
//            float_to_u8(&nuc_transmit_data.tyre_speed_data_send.panning_speed,t[4]);
//            float_to_u8(&nuc_transmit_data.tyre_speed_data_send.rotate_speed,t[5]);
//            for(int i=0;i<6;i++){
//                for(int j=0;j<4;j++){
//                    usb_cdc_data.usb_cdc_send_buf[4+i*4+j]=t[i][j];
//                }
//            }
//            break;
//        case REFEREE_SEND_ID:
//            len=nuc_transmit_data.referee_data_send.data_length+5;
//            memcpy(&usb_cdc_data.usb_cdc_send_buf[4],nuc_transmit_data.referee_data_send.referee_send_data,len-4);
//            break;
        case GIMBAL_AND_CONFIG_SEND_ID:
            len=sizeof(nuc_transmit_data.robot_gimbal_data_send)+4;    
            float_to_u8(&nuc_transmit_data.robot_gimbal_data_send.pitch,t[0]);
            float_to_u8(&nuc_transmit_data.robot_gimbal_data_send.roll,t[1]);
            float_to_u8(&nuc_transmit_data.robot_gimbal_data_send.yaw,t[2]);
            for(int i=0;i<3;i++){
                for(int j=0;j<4;j++){
                    usb_cdc_data.usb_cdc_send_buf[4+i*4+j]=t[i][j];
                }
            }
            usb_cdc_data.usb_cdc_send_buf[16]=nuc_transmit_data.robot_gimbal_data_send.is_pressing;
            usb_cdc_data.usb_cdc_send_buf[17]=nuc_transmit_data.robot_gimbal_data_send.mode;
            for(int i=0;i<8;i++){
                usb_cdc_data.usb_cdc_send_buf[18+i]=nuc_transmit_data.robot_gimbal_data_send.mode_config[i];
            }
            usb_cdc_data.usb_cdc_send_buf[26]=nuc_transmit_data.robot_gimbal_data_send.camera_id;
            break;
        default:
            break;
    }
	
		usb_cdc_data.usb_cdc_send_buf[3]=cmd_id;
		usb_cdc_data.usb_cdc_send_buf[1]=(len);
    usb_cdc_data.usb_cdc_send_buf[2]=((len>>8));
    usb_cdc_data.usb_cdc_send_buf[len-1]=CRC_Calculation(usb_cdc_data.usb_cdc_send_buf,len-1);
    usb_data_send(usb_cdc_data.usb_cdc_send_buf,len);
}

//AA 0C 00 13 00 01 02 03 04 05 06 07 08 09 0A XX

/*************************** SEND ********************************/

/*************************** RECV ********************************/
void Nuc_data_unpacked()
{
    if(usb_cdc_data.usb_cdc_rx_flag==1)
		{
        if(usb_cdc_data.usb_cdc_rx_buf[0]==0xAA)
				{
//			usb_cdc_data.usb_cdc_rx_len=(uint16_t)((usb_cdc_data.usb_cdc_rx_buf[2] << 8) |usb_cdc_data.usb_cdc_rx_buf[1])+4;
					usb_cdc_data.crc_cal=CRC_Calculation(usb_cdc_data.usb_cdc_rx_buf,usb_cdc_data.usb_cdc_rx_len-1);
            if(usb_cdc_data.crc_cal==usb_cdc_data.usb_cdc_rx_buf[usb_cdc_data.usb_cdc_rx_len-1])
						{
                uint8_t t[4][4];
                switch (usb_cdc_data.usb_cdc_rx_buf[3])
                {
                    case AIM_DATA_RECV_ID:
                        for(int i=0;i<4;i++)
												{
                            for(int j=0;j<4;j++)
														{
                                t[i][j]=usb_cdc_data.usb_cdc_rx_buf[4+i*4+j];
                            }
                        }
                        u8_to_float(t[0],&nuc_receive_data.aim_data_received.yaw);
                        u8_to_float(t[1],&nuc_receive_data.aim_data_received.pitch);
												u8_to_float(t[2],&nuc_receive_data.aim_data_received.Omega_yaw);
												u8_to_float(t[3],&nuc_receive_data.aim_data_received.Omega_pitch);
                        nuc_receive_data.aim_data_received.target_rate=usb_cdc_data.usb_cdc_rx_buf[20];
                        nuc_receive_data.aim_data_received.target_number=usb_cdc_data.usb_cdc_rx_buf[21];
                        nuc_receive_data.aim_data_received.success=usb_cdc_data.usb_cdc_rx_buf[22];
                        break;
//                    case CHASSIS_CONTROL_RECV_ID:
//                        nuc_receive_data.chassis_data_received.forward_speed=(int16_t)(usb_cdc_data.usb_cdc_rx_buf[4] | usb_cdc_data.usb_cdc_rx_buf[5]<<8);
//                        nuc_receive_data.chassis_data_received.panning_speed=(int16_t)(usb_cdc_data.usb_cdc_rx_buf[6] | usb_cdc_data.usb_cdc_rx_buf[7]<<8);
//                        nuc_receive_data.chassis_data_received.rotate_speed=(int16_t)(usb_cdc_data.usb_cdc_rx_buf[8] | usb_cdc_data.usb_cdc_rx_buf[9]<<8);                        
//                        break;
//                    case REFEREE_RECV_ID:
//                        nuc_receive_data.referee_data_received.data_length=usb_cdc_data.usb_cdc_rx_len-5;
//                        memcpy(nuc_receive_data.referee_data_received.referee_recv_data,&usb_cdc_data.usb_cdc_rx_buf[4],nuc_receive_data.referee_data_received.data_length);
//                        break;
                    default:
                        break;
                }
            }
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
