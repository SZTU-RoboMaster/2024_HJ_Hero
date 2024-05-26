/******************include******************/
#include "Chassis.h"

extern key_board_t KeyBoard;
static CAN_TxHeaderTypeDef  tx_message;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void Send_referee(fp32 power){
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union referee temp;
    tx_message.StdId = 0x114;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    temp.referee = power;
    Send_data[0] = temp.data[0];
    Send_data[1] = temp.data[1];
    Send_data[2] = temp.data[2];
    Send_data[3] = temp.data[3];
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_angle(fp32 absolute_angle){
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union angle temp;
    tx_message.StdId = 0x114;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    temp.value = absolute_angle;
    Send_data[4] = temp.data[0];
    Send_data[5] = temp.data[1];
    Send_data[6] = temp.data[2];
    Send_data[7] = temp.data[3];
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}

void Send_id(uint8_t robot_id) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    tx_message.StdId = 0x115;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;

    Send_data[0] = robot_id;
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, Send_data, &send_mail_box);
}