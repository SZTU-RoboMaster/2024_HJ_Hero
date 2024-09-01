#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*void CAN_Gimbal_init(){
    // 云台的CAN初始化
    // 使用过滤器 CAN1: [0~6] CAN2: [14~20]

    // 基本部分
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0xFFFF;
    can_filter_st.FilterMaskIdLow = 0x0000;

    *//****************** CAN1 过滤器 ***********************//*
    can_filter_st.SlaveStartFilterBank = 0;
    // 过滤器0
//    can_filter_st.FilterBank = 0;
//    can_filter_st.FilterIdHigh = 0x0000;
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//
//    // 开启CAN1过滤器
//    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    *//****************** CAN2 过滤器 ***********************//*
    can_filter_st.SlaveStartFilterBank = 14;
    // single_shoot-0x204 单发限位电机
    can_filter_st.FilterIdHigh = (0x204 <<5);               // 接受ID
    can_filter_st.FilterBank = 14;                          // 过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // 将接受的的报文放入FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // Yaw-0x205 Yaw电机
    can_filter_st.FilterBank = 18;
    can_filter_st.FilterIdHigh = (0x205 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // pitch-0x206 Pitch电机
    can_filter_st.FilterIdHigh = (0x206 <<5);               // 接受ID
    can_filter_st.FilterBank = 15;                          // 过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // 将接受的的报文放入FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // fire_r-0x207 右摩擦轮电机
    can_filter_st.FilterIdHigh = (0x207 <<5);               // 接受ID
    can_filter_st.FilterBank = 16;                          // 过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // 将接受的的报文放入FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // fire_l-0x208 左摩擦轮电机
    can_filter_st.FilterIdHigh = (0x208 <<5);               // 接受ID
    can_filter_st.FilterBank = 17;                          // 过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // 将接受的的报文放入FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // 开启CAN2过滤器
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CAN_Chass_init(){
    // 底盘的CAN初始化
    // 使用过滤器 CAN1: [7~13] CAN2: [21~27]

    // 基本部分
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0xFFFF;
    can_filter_st.FilterMaskIdLow = 0x0000;

    *//****************** CAN1 过滤器 ***********************//*
    can_filter_st.SlaveStartFilterBank = 0;

    // motor_chassis[RF]-0x201 右上底盘电机
    can_filter_st.FilterBank = 7;
    can_filter_st.FilterIdHigh = (0x201 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[LF]-0x202 左上底盘电机
    can_filter_st.FilterBank = 8;
    can_filter_st.FilterIdHigh = (0x202 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[LB]-0x203 左下底盘电机
    can_filter_st.FilterBank = 9;
    can_filter_st.FilterIdHigh = (0x203 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[RB]-0x204 右下底盘电机
    can_filter_st.FilterBank = 10;
    can_filter_st.FilterIdHigh = (0x204 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // CAN_IMAGE_2006_TRANSMISSION-0x205 图传电机
    can_filter_st.FilterBank = 11;
    can_filter_st.FilterIdHigh = (0x205 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // TRIGGER-0x207 拨盘电机
    can_filter_st.FilterBank = 12;
    can_filter_st.FilterIdHigh = (0x207 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // 自定义数据集-[0x110~0x117]
    can_filter_st.FilterBank = 13;
    can_filter_st.FilterIdHigh = (0x110 << 5);
    can_filter_st.FilterMaskIdHigh = 0xFF00;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // 开启CAN1过滤器
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    *//****************** CAN2 过滤器 ***********************//*
*//*    can_filter_st.SlaveStartFilterBank = 14;
    // 过滤器0
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // 开启CAN2过滤器
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);*//*
}*/
