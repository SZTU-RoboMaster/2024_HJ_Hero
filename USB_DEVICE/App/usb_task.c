//
// Created by Shockley on 2022/11/9.
//

#include "usb_task.h"

static uint8_t usb_buf[128];
extern QueueHandle_t CDC_send_queue;
extern void rm_dequeue_send_data(void* buf,uint16_t len);
rc_info_t rc_data;

_Noreturn void USB_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    while(1)
    {
        // rc_data.s[0]='a';
        // rm_queue_data(RC_ID,&rc_data,sizeof (rc_info_t));
        /*if(CDC_send_queue!=NULL&&xQueueReceive( CDC_send_queue, usb_buf, 10 ) == pdTRUE)
        {
            rm_dequeue_send_data(usb_buf,128);
        }*/
        if(xQueueReceive( CDC_send_queue, usb_buf, 10 ) == pdTRUE)
        {
            rm_dequeue_send_data(usb_buf,128);
        }
        osDelay(2);
    }
}