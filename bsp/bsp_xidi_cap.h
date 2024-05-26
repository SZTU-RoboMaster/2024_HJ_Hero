//
// Created by suheng on 2024/5/12.
//

#ifndef OMNI_INFANTRY_BSP_XIDI_CAP_H
#define OMNI_INFANTRY_BSP_XIDI_CAP_H
#include "main.h"
#include "struct_typedef.h"
#include "Referee.h"
#include "string.h"
#include "usart.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "math.h"
#include "Detection.h"
#include "launcher.h"
#include "Cap.h"
#include "Hero.h"



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
typedef enum
{
    Vi = 0,
    Vo = 1,
    Pi = 2,
    Ii = 3,
    Io = 4,
    Pr = 5
}cap_type_e;

typedef  struct
{
    fp32 voltage_in;  //输入电压
    fp32 voltage_out;  //输出电压
    fp32 power_in;   //输入功率（从底盘拿的功率）
    fp32 current_in;  //输入电流
    fp32 current_out;   //输出电流
    fp32 power_reference;   //最大功率限制
}cap_receive_data_t;

void CapRxCallback(uint8_t *ReadFromUsart);
void Cap_info_decode(uint8_t cmd, uint8_t *ReadFromUsart);
bool_t Cap_read_data(uint8_t *ReadFromUsart);
void send_to_cap_xidi(int power_limit,UART_HandleTypeDef *huart);
#endif //OMNI_INFANTRY_BSP_XIDI_CAP_H
