/******************include******************/
#include "Gimbal.h"

static CAN_TxHeaderTypeDef  tx_message;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void Send_Chassis_Speed(int16_t ch1, int16_t ch0, int16_t ch2, uint8_t s_l, int32_t Mouse_l){
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union ctrl_ch temp;
    tx_message.StdId = 0x110;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    temp.value = ch1;
    Send_data[0] = temp.data[0];
    Send_data[1] = temp.data[1];
    temp.value = ch0;
    Send_data[2] = temp.data[0];
    Send_data[3] = temp.data[1];
    temp.value = ch2;
    Send_data[4] = temp.data[0];
    Send_data[5] = temp.data[1];
    Send_data[6] = s_l;
    Send_data[7] = (uint8_t)(Mouse_l & 0xFF);

    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}


void Send_Yaw_Angle(fp32 relative_angle, uint8_t fire_lock, int32_t V){
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union angle temp;
    tx_message.StdId = 0x111;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    temp.value = relative_angle;
    Send_data[0] = temp.data[0];
    Send_data[1] = temp.data[1];
    Send_data[2] = temp.data[2];
    Send_data[3] = temp.data[3];
    Send_data[4] = fire_lock;
    Send_data[5] = (uint8_t)(V & 0xFF);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_Mode(int16_t mode1, int16_t mode2, int16_t mode3, int16_t mode4) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union mode temp;
    tx_message.StdId = 0x112;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    temp.mode = mode1;
    Send_data[0] = temp.data[0];
    Send_data[1] = temp.data[1];
    temp.mode = mode2;
    Send_data[2] = temp.data[0];
    Send_data[3] = temp.data[1];
    temp.mode = mode3;
    Send_data[4] = temp.data[0];
    Send_data[5] = temp.data[1];
    temp.mode = mode4;
    Send_data[6] = temp.data[0];
    Send_data[7] = temp.data[1];

    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_Keyboard(int32_t W, int32_t A, int32_t S, int32_t D, int16_t trigger_mode) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union mode trigger_motion;
    tx_message.StdId = 0x113;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x06;
    Send_data[0] = (uint8_t)(W & 0xFF);
    Send_data[1] = (uint8_t)(A & 0xFF);
    Send_data[2] = (uint8_t)(S & 0xFF);
    Send_data[3] = (uint8_t)(D & 0xFF);

    trigger_motion.mode = trigger_mode;
    Send_data[4] = trigger_motion.data[0];
    Send_data[5] = trigger_motion.data[1];

    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
    vTaskDelay(1);
}

void Send_command(int8_t command) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    tx_message.StdId = 0x113;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    union com temp;
    Send_data[6] = temp.data[0];
    Send_data[7] = temp.data[1];
//    Send_data[7] = (uint8_t)(Mouse_r & 0xFF);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}
