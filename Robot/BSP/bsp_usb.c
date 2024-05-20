#include "bsp_usb.h"


usb_cdc_data_t usb_cdc_data;

void usb_init(void)
{
	usb_cdc_data.usb_cdc_rx_len=0;
	usb_cdc_data.usb_cdc_rx_flag=0;
	MX_USB_DEVICE_Init();

}

void usb_data_send(uint8_t *data,uint16_t len)
{
    CDC_Transmit_FS(data,len);
}
